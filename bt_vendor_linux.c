/******************************************************************************
 *
 *  Copyright (C) 2013-2016 Intel Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

#define LOG_TAG "bt_vendor"

#include <errno.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdint.h>
#include <poll.h>
#include <unistd.h>
#include <string.h>

#include <sys/socket.h>
#include <sys/ioctl.h>

#include "bt_vendor_lib.h"
#include <utils/Log.h>
#include <cutils/properties.h>
#include "bt_hci_bdroid.h"
#include <assert.h>
#include <semaphore.h>

#define HCI_CMD_MARVELL_SET_SCO_DATA_PATH       0xFC1D
#define HCI_CMD_MARVELL_WRITE_PCM_SYNC_SETTINGS 0xFC28
#define HCI_CMD_MARVELL_WRITE_PCM_LINK_SETTINGS 0xFC29
#define HCI_CMD_MARVELL_WRITE_PCM_SETTINGS      0xFC07
#define HCI_CMD_MARVELL_WRITE_BD_ADDRESS        0xFC22
#define HCI_CMD_MARVELL_WRITE_WBS_SETTINGS      0xFC73

#define HCI_EVT_CMD_CMPL_OPCODE 3

#define WRITE_PCM_SETTINGS_SIZE            1
#define WRITE_PCM_SYNC_SETTINGS_SIZE       3
#define WRITE_PCM_LINK_SETTINGS_SIZE       2
#define SET_SCO_DATA_PATH_SIZE             1
#define WRITE_BD_ADDRESS_SIZE              8
#define WRITE_WBS_SETTINGS_SIZE            1

#define NARROW_BAND_CODEC                  1

#define STREAM_TO_UINT16(u16, p) \
do { \
	u16 = ((uint16_t)(*(p)) + (((uint16_t)(*((p) + 1))) << 8)); \
	(p) += 2; \
} while (0)

#define UINT16_TO_STREAM(p, u16) \
do { \
	*(p)++ = (uint8_t)(u16); \
	*(p)++ = (uint8_t)((u16) >> 8); \
} while (0)

static void hw_mrvl_sco_config_cb(void *p_mem);
static uint8_t write_pcm_settings[WRITE_PCM_SETTINGS_SIZE] = {
	0x12
};

static uint8_t write_pcm_sync_settings[WRITE_PCM_SYNC_SETTINGS_SIZE] = {
	0x03,
	0x00,
	0x03
};

static uint8_t write_pcm_link_settings[WRITE_PCM_LINK_SETTINGS_SIZE] = {
	0x04,
	0x00
};

static uint8_t set_sco_data_path[SET_SCO_DATA_PATH_SIZE] = {
	0x01
};

static uint8_t write_wbs_settings[WRITE_WBS_SETTINGS_SIZE] = {
	0x01
};

struct bt_evt_param_t {
	uint16_t cmd;
	uint8_t cmd_ret_param;
};

static sem_t sem;
static int sco_cfg_status;

#define HCI_CMD_PREAMBLE_SIZE 3
#define BTPROTO_HCI	      1
#define HCI_CHANNEL_USER	1
#define HCI_CHANNEL_CONTROL	3
#define HCI_DEV_NONE	0xffff

#define RFKILL_TYPE_BLUETOOTH	2
#define RFKILL_OP_CHANGE_ALL	3

#define MGMT_OP_INDEX_LIST	0x0003
#define MGMT_EV_INDEX_ADDED	0x0004
#define MGMT_EV_COMMAND_COMP	0x0001
#define MGMT_EV_SIZE_MAX	1024
#define MGMT_EV_POLL_TIMEOUT	3000 /* 3000ms */

#define IOCTL_HCIDEVDOWN	_IOW('H', 202, int)

#ifdef USE_CELLULAR_COEX
int hci_bind_client_init(void);
int hci_bind_client_cleanup(void);
#endif

struct sockaddr_hci {
	sa_family_t    hci_family;
	unsigned short hci_dev;
	unsigned short hci_channel;
};

struct rfkill_event {
	uint32_t idx;
	uint8_t  type;
	uint8_t  op;
	uint8_t  soft, hard;
} __attribute__((packed));

struct mgmt_pkt {
        uint16_t opcode;
        uint16_t index;
        uint16_t len;
        uint8_t  data[MGMT_EV_SIZE_MAX];
} __attribute__((packed));

struct mgmt_event_read_index {
	uint16_t cc_opcode;
	uint8_t  status;
	uint16_t num_intf;
	uint16_t index[0];
} __attribute__((packed));

const bt_vendor_callbacks_t *bt_vendor_callbacks = NULL;
static unsigned char bt_vendor_local_bdaddr[6];
static int bt_vendor_fd = -1;
static int hci_interface = 0;
static int rfkill_en = 0;
static int bt_hwcfg_en = 0;

static int bt_vendor_init(const bt_vendor_callbacks_t *p_cb, unsigned char *local_bdaddr)
{
	char prop_value[PROPERTY_VALUE_MAX];

	ALOGI("%s", __func__);

	if (p_cb == NULL) {
		ALOGE("init failed with no user callbacks!");
		return -1;
	}

	bt_vendor_callbacks = p_cb;

	memcpy(bt_vendor_local_bdaddr, local_bdaddr, sizeof(bt_vendor_local_bdaddr));

	property_get("bluetooth.interface", prop_value, "0");

	errno = 0;
	if (memcmp(prop_value, "hci", 3))
		hci_interface = strtol(prop_value, (char **)NULL, 10);
	else
		hci_interface = strtol(prop_value + 3, (char **)NULL, 10);
	if (errno)
		hci_interface = 0;

	ALOGI("Using interface hci%d", hci_interface);

	property_get("bluetooth.rfkill", prop_value, "0");

	rfkill_en = atoi(prop_value);
	if (rfkill_en)
		ALOGI("RFKILL enabled");

	bt_hwcfg_en = property_get("vendor.bluetooth.hwcfg", prop_value, NULL) > 0 ? 1 : 0;
	if (bt_hwcfg_en)
		ALOGI("HWCFG enabled");

#ifdef USE_CELLULAR_COEX
	hci_bind_client_init();
#endif

	return 0;
}

static int bt_vendor_hw_cfg(int stop)
{
	if (!bt_hwcfg_en)
		return 0;

	/* This is a simple property set, depending the hardware, this should trigger
	 * operation to create or configure the hci device (hciattach...) in device
	 * init rules.
	 */
	if (stop) {
		ALOGI("HWCFG : Stopping hardware config");
		if (property_set("vendor.bluetooth.hwcfg", "stop") < 0) {
			ALOGE("%s cannot stop btcfg service via prop", __func__);
			return 1;
		}
	} else {
		ALOGI("HWCFG : Starting hardware config");
		if (property_set("vendor.bluetooth.hwcfg", "start") < 0) {
			ALOGE("%s cannot start btcfg service via prop", __func__);
			return 1;
		}
	}
	return 0;
}

static int bt_vendor_wait_hcidev(void)
{
	struct sockaddr_hci addr;
	struct pollfd fds[1];
	struct mgmt_pkt ev;
	int fd;
	int ret = 0;

	ALOGI("%s", __func__);

	fd = socket(PF_BLUETOOTH, SOCK_RAW, BTPROTO_HCI);
	if (fd < 0) {
		ALOGE("Bluetooth socket error: %s", strerror(errno));
		return -1;
	}

	memset(&addr, 0, sizeof(addr));
	addr.hci_family = AF_BLUETOOTH;
	addr.hci_dev = HCI_DEV_NONE;
	addr.hci_channel = HCI_CHANNEL_CONTROL;

	if (bind(fd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
		ALOGE("HCI Channel Control: %s", strerror(errno));
		close(fd);
		return -1;
	}

	fds[0].fd = fd;
	fds[0].events = POLLIN;

	/* Read Controller Index List Command */
	ev.opcode = MGMT_OP_INDEX_LIST;
	ev.index = HCI_DEV_NONE;
	ev.len = 0;
	if (write(fd, &ev, 6) != 6) {
		ALOGE("Unable to write mgmt command: %s", strerror(errno));
		ret = -1;
		goto end;
	}

	while (1) {
		int n = poll(fds, 1, MGMT_EV_POLL_TIMEOUT);
		if (n == -1) {
			ALOGE("Poll error: %s", strerror(errno));
			ret = -1;
			break;
		} else if (n == 0) {
			ALOGE("Timeout, no HCI device detected");
			ret = -1;
			break;
		}

		if (fds[0].revents & POLLIN) {
			n = read(fd, &ev, sizeof(struct mgmt_pkt));
			if (n < 0) {
				ALOGE("Error reading control channel");
				ret = -1;
				break;
			}

			if (ev.opcode == MGMT_EV_INDEX_ADDED &&
			    ev.index == hci_interface) {
				goto end;
			} else if (ev.opcode == MGMT_EV_COMMAND_COMP) {
				struct mgmt_event_read_index *cc;
				int i;

				cc = (struct mgmt_event_read_index *)ev.data;

				if ((cc->cc_opcode != MGMT_OP_INDEX_LIST)
				    || (cc->status != 0))
					continue;

				for (i = 0; i < cc->num_intf; i++) {
					if (cc->index[i] == hci_interface)
						goto end;
				}
			}
		}
	}

end:
	close(fd);
	return ret;
}

static int bt_vendor_open(void *param)
{
	int (*fd_array)[] = (int (*) []) param;
	int fd;

	ALOGI("%s", __func__);

	fd = socket(AF_BLUETOOTH, SOCK_RAW, BTPROTO_HCI);
	if (fd < 0) {
		ALOGE("socket create error %s", strerror(errno));
		return -1;
	}

	(*fd_array)[CH_CMD] = fd;
	(*fd_array)[CH_EVT] = fd;
	(*fd_array)[CH_ACL_OUT] = fd;
	(*fd_array)[CH_ACL_IN] = fd;

	bt_vendor_fd = fd;

	ALOGI("%s returning %d", __func__, bt_vendor_fd);

	return 1;
}

static int bt_vendor_close(void *param)
{
	(void)(param);

	ALOGI("%s", __func__);

	if (bt_vendor_fd != -1) {
		close(bt_vendor_fd);
		bt_vendor_fd = -1;
	}

	return 0;
}

static int bt_vendor_rfkill(int block)
{
	struct rfkill_event event;
	int fd, len;

	ALOGI("%s", __func__);

	fd = open("/dev/rfkill", O_WRONLY);
	if (fd < 0) {
		ALOGE("Unable to open /dev/rfkill");
		return -1;
	}

	memset(&event, 0, sizeof(struct rfkill_event));
	event.op = RFKILL_OP_CHANGE_ALL;
	event.type = RFKILL_TYPE_BLUETOOTH;
	event.hard = block;
	event.soft = block;

	len = write(fd, &event, sizeof(event));
	if (len < 0) {
		ALOGE("Failed to change rfkill state");
		close(fd);
		return 1;
	}

	close(fd);
	return 0;
}

/* TODO: fw config should thread the device waiting and return immedialty */
static void bt_vendor_fw_cfg(void)
{
	struct sockaddr_hci addr;
	int fd = bt_vendor_fd;

	ALOGI("%s", __func__);

	if (fd == -1) {
		ALOGE("bt_vendor_fd: %s", strerror(EBADF));
		goto failure;
	}

	memset(&addr, 0, sizeof(addr));
        addr.hci_family = AF_BLUETOOTH;
        addr.hci_dev = hci_interface;
        addr.hci_channel = HCI_CHANNEL_USER;

        if (bt_vendor_wait_hcidev()) {
                ALOGE("HCI interface (%d) not found", hci_interface);
		goto failure;
	}

        /* Force interface down to use HCI user channel */
        if (ioctl(fd, IOCTL_HCIDEVDOWN, hci_interface)) {
                ALOGE("HCIDEVDOWN ioctl error: %s", strerror(errno));
		goto failure;
	}

        if (bind(fd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
                ALOGE("socket bind error %s", strerror(errno));
                goto failure;
        }

	ALOGI("HCI device ready");

	bt_vendor_callbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);

	return;

failure:
	ALOGE("Hardware Config Error");
	bt_vendor_callbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
}

static HC_BT_HDR *build_cmd_buf(uint16_t cmd, uint8_t pl_len, uint8_t *payload)
{
	HC_BT_HDR *p_buf = NULL;
	uint16_t cmd_len = HCI_CMD_PREAMBLE_SIZE + pl_len;
	uint8_t *p;

	assert(bt_vendor_callbacks && payload);

	p_buf = (HC_BT_HDR *) bt_vendor_callbacks->alloc(BT_HC_HDR_SIZE + cmd_len);

	if (!p_buf)
		return NULL;

	p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
	p_buf->offset = 0;
	p_buf->layer_specific = 0;
	p_buf->len = cmd_len;

	p = (uint8_t *) (p_buf + 1);

	/* opcode */
	UINT16_TO_STREAM(p, cmd);

	/* length of payload */
	*p = pl_len;
	++p;

	/* payload */
	memcpy(p, payload, pl_len);

	return p_buf;
}

static char *cmd_to_str(uint16_t cmd)
{
	switch (cmd) {
	case HCI_CMD_MARVELL_WRITE_PCM_SETTINGS:
		return "write_pcm_settings";
	case HCI_CMD_MARVELL_WRITE_PCM_SYNC_SETTINGS:
		return "write_pcm_sync_settings";
	case HCI_CMD_MARVELL_WRITE_PCM_LINK_SETTINGS:
		return "write_pcm_link_settings";
	case HCI_CMD_MARVELL_SET_SCO_DATA_PATH:
		return "set_sco_data_path";
	case HCI_CMD_MARVELL_WRITE_BD_ADDRESS:
		return "write_bd_address";
        case HCI_CMD_MARVELL_WRITE_WBS_SETTINGS:
		return "write_wbs_settings";
	default:
		break;
	}

	return "unknown command";
}

static void hw_mrvl_sco_config(void)
{
	HC_BT_HDR *p_buf;
	uint16_t cmd;

	assert(bt_vendor_callbacks);

	ALOGI("Start SCO config ...");
		/* Send HCI_CMD_MARVELL_SET_SCO_DATA_PATH */
	cmd = HCI_CMD_MARVELL_SET_SCO_DATA_PATH;
	p_buf = build_cmd_buf(cmd,
			SET_SCO_DATA_PATH_SIZE,
			set_sco_data_path);

	if (p_buf) {
		ALOGI("Sending hci command 0x%04hX (%s)", cmd, cmd_to_str(cmd));
		if (bt_vendor_callbacks->xmit_cb(cmd, p_buf, hw_mrvl_sco_config_cb))
			return;
		else
			bt_vendor_callbacks->dealloc(p_buf);
	}

	ALOGE("Vendor lib scocfg aborted");
	sco_cfg_status = BT_VND_OP_RESULT_FAIL;
	sem_post(&sem);
}
static void wide_band_config_cb(void *buf) {
	if (buf)
		bt_vendor_callbacks->dealloc(buf);
}

static void send_wide_band_switch(int codec) {
	uint8_t enable_wide_band;
	HC_BT_HDR *p_buf;
	uint16_t cmd;
	ALOGI("Configuration codec = %d", codec);
	if (codec == NARROW_BAND_CODEC) {
		enable_wide_band = 0;
	} else {
		enable_wide_band = 1;
	}
	cmd = HCI_CMD_MARVELL_WRITE_WBS_SETTINGS;
	p_buf = build_cmd_buf(cmd, 1, &enable_wide_band);
	if (p_buf) {
		ALOGI("Sending hci command 0x%04hX (%s)", cmd, cmd_to_str(cmd));
		if (bt_vendor_callbacks->xmit_cb(cmd, p_buf, wide_band_config_cb))
			return;
		else
			bt_vendor_callbacks->dealloc(p_buf);
	}
}



static void parse_evt_buf(HC_BT_HDR *p_evt_buf,
		struct bt_evt_param_t *evt_params)
{
	uint8_t *p = (uint8_t *) (p_evt_buf + 1) + HCI_EVT_CMD_CMPL_OPCODE;

	assert(p_evt_buf && evt_params);

	/* opcode */
	STREAM_TO_UINT16(evt_params->cmd, p);

	/* command return parameter */
	evt_params->cmd_ret_param = *p;
}

static void hw_mrvl_sco_config_cb(void *p_mem)
{
	HC_BT_HDR *p_evt_buf = (HC_BT_HDR *) p_mem;
	struct bt_evt_param_t evt_params = {0, 0};
	uint16_t cmd;
	HC_BT_HDR *p_buf;

	assert(bt_vendor_callbacks && p_mem);

	parse_evt_buf(p_evt_buf, &evt_params);

	/* free the buffer */
	bt_vendor_callbacks->dealloc(p_evt_buf);
	switch (evt_params.cmd) {
	case HCI_CMD_MARVELL_SET_SCO_DATA_PATH:
		/* Send HCI_CMD_MARVELL_WRITE_PCM_SYNC_SETTINGS */
		cmd = HCI_CMD_MARVELL_WRITE_PCM_SYNC_SETTINGS;
		p_buf = build_cmd_buf(cmd,
				WRITE_PCM_SYNC_SETTINGS_SIZE,
				write_pcm_sync_settings);
		break;

	case HCI_CMD_MARVELL_WRITE_PCM_SYNC_SETTINGS:
		/* Send HCI_CMD_MARVELL_WRITE_PCM_LINK_SETTINGS */
		cmd = HCI_CMD_MARVELL_WRITE_PCM_LINK_SETTINGS;
		p_buf = build_cmd_buf(cmd,
				WRITE_PCM_LINK_SETTINGS_SIZE,
				write_pcm_link_settings);
		break;

	case HCI_CMD_MARVELL_WRITE_PCM_LINK_SETTINGS:
		/* Start with HCI_CMD_MARVELL_WRITE_PCM_SETTINGS */
		cmd   = HCI_CMD_MARVELL_WRITE_PCM_SETTINGS;
		p_buf = build_cmd_buf(cmd,
				WRITE_PCM_SETTINGS_SIZE,
				write_pcm_settings);
		break;

	case HCI_CMD_MARVELL_WRITE_PCM_SETTINGS:
		/* sco config succeeds */
		ALOGI("SCO PCM config succeeds!");
		cmd   = HCI_CMD_MARVELL_WRITE_WBS_SETTINGS;
		p_buf = build_cmd_buf(cmd,
				WRITE_WBS_SETTINGS_SIZE,
				write_wbs_settings);
		 break;
	case HCI_CMD_MARVELL_WRITE_WBS_SETTINGS:
		ALOGI("WBS switch config succeeds!");
		sco_cfg_status = BT_VND_OP_RESULT_SUCCESS;
		sem_post(&sem);
		return;

	default:
		ALOGE("Received event for unexpected cmd (0x%04hX). Fail.",
			evt_params.cmd);
		p_buf = NULL;
		break;
	} /* switch (evt_params.cmd) */

	if (p_buf) {
		ALOGI("Sending hci command 0x%04hX (%s)", cmd, cmd_to_str(cmd));
		if (bt_vendor_callbacks->xmit_cb(cmd, p_buf, hw_mrvl_sco_config_cb))
			return;
		else
			bt_vendor_callbacks->dealloc(p_buf);
	}

	ALOGE("Vendor lib scocfg aborted");
	sco_cfg_status = BT_VND_OP_RESULT_FAIL;
	sem_post(&sem);
}

static int bt_vendor_op(bt_vendor_opcode_t opcode, void *param)
{
	int retval = 0;
	struct timespec abs_timeout;

	ALOGI("%s op %d", __func__, opcode);

	switch (opcode) {
	case BT_VND_OP_POWER_CTRL:
		ALOGI("%s power param %d", __func__, *((int*)param) );

		/* Block/unblock bluetooth rfkill node if rfkill is supported.
		 * Perform hardware config if supported.
		 */
		if ((!bt_hwcfg_en && !rfkill_en) || !param)
			break;

		if (*((int*)param) == BT_VND_PWR_ON && rfkill_en)
			retval = bt_vendor_rfkill(0);

		if (*((int*)param) == BT_VND_PWR_ON && bt_hwcfg_en)
			retval = bt_vendor_hw_cfg(0);

		if (*((int*)param) == BT_VND_PWR_OFF && bt_hwcfg_en)
			retval = bt_vendor_hw_cfg(1);

		if (*((int*)param) == BT_VND_PWR_OFF && rfkill_en)
			retval = bt_vendor_rfkill(1);

		break;

	case BT_VND_OP_FW_CFG:
		bt_vendor_fw_cfg();
		break;

	case BT_VND_OP_SCO_CFG:
		sco_cfg_status = BT_VND_OP_RESULT_FAIL;
		sem_init(&sem, 0, 0);
		if (clock_gettime(CLOCK_REALTIME, &abs_timeout) == -1) {
			ALOGE("clock_gettime() FAIL errno = %d", errno);
		}
		abs_timeout.tv_nsec += 500000000; /* timeout of 500 msec */
		abs_timeout.tv_sec += abs_timeout.tv_nsec / 1000000000;
		abs_timeout.tv_nsec %= 1000000000;
		hw_mrvl_sco_config();
		sem_timedwait(&sem, &abs_timeout);
		bt_vendor_callbacks->scocfg_cb(sco_cfg_status);
		sem_destroy(&sem);
		break;

	case BT_VND_OP_USERIAL_OPEN:
		retval = bt_vendor_open(param);
		break;

	case BT_VND_OP_USERIAL_CLOSE:
		retval = bt_vendor_close(param);
		break;

        case BT_VND_OP_GET_LPM_IDLE_TIMEOUT:
		*((uint32_t *)param) = 3000;
		retval = 0;
		break;

	case BT_VND_OP_LPM_SET_MODE:
		bt_vendor_callbacks->lpm_cb(BT_VND_OP_RESULT_SUCCESS);
		break;

	case BT_VND_OP_LPM_WAKE_SET_STATE:
		break;

	case BT_VND_OP_SET_AUDIO_STATE:
                if (param == NULL) {
			bt_vendor_callbacks->audio_state_cb(BT_VND_OP_RESULT_FAIL);
			retval = -1;
			break;
		}
		send_wide_band_switch(*((int *)param));
		bt_vendor_callbacks->audio_state_cb(BT_VND_OP_RESULT_SUCCESS);
		break;

	case BT_VND_OP_EPILOG:
		bt_vendor_callbacks->epilog_cb(BT_VND_OP_RESULT_SUCCESS);
		break;

        case BT_VND_OP_A2DP_OFFLOAD_START:
                break;

        case BT_VND_OP_A2DP_OFFLOAD_STOP:
                break;
	}

	ALOGI("%s op %d retval %d", __func__, opcode, retval);

	return retval;
}

static void bt_vendor_cleanup( void )
{
	ALOGI("%s", __func__);

	bt_vendor_callbacks = NULL;

#ifdef USE_CELLULAR_COEX
	hci_bind_client_cleanup();
#endif
}

const bt_vendor_interface_t BLUETOOTH_VENDOR_LIB_INTERFACE = {
	sizeof(bt_vendor_interface_t),
	bt_vendor_init,
	bt_vendor_op,
	bt_vendor_cleanup,
};
