LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES := $(call all-java-files-under, src/com/example/usbfastcv)


LOCAL_PACKAGE_NAME := usbfastdemo
#LOCAL_CERTIFICATE := platform

include $(BUILD_PACKAGE)

include $(call all-named-subdir-makefiles, jni)
