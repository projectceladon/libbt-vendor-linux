LOCAL_PATH := $(call my-dir)

ifeq ($(BOARD_HAVE_BLUETOOTH_LINUX_PRI), true)

include $(CLEAR_VARS)

BDROID_DIR := $(TOP_DIR)system/bt

LOCAL_SRC_FILES := \
        bt_vendor_linux.c

LOCAL_C_INCLUDES += \
        $(BDROID_DIR)/hci/include

LOCAL_SHARED_LIBRARIES := \
        libcutils \
        liblog

LOCAL_MODULE := libbt-vendor
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE_TAGS := optional
LOCAL_HEADER_LIBRARIES += libutils_headers
include $(BUILD_SHARED_LIBRARY)

endif # BOARD_HAVE_BLUETOOTH_LINUX_PRI
