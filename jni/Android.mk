LOCAL_PATH:= $(call my-dir)

#include $(CLEAR_VARS)
#LOCAL_MODULE := libfastcv
#LOCAL_SRC_FILES := libfastcv.a

#include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_PRELINK_MODULE:= false

# This variable determines the OpenGL ES API version to use:
# If set to true, OpenGL ES 1.1 is used, otherwise OpenGL ES 2.0.

USE_OPENGL_ES_1_1 := false

# Set OpenGL ES version-specific settings.

ifeq ($(USE_OPENGL_ES_1_1), true)
    OPENGLES_LIB  := -lGLESv1_CM
    OPENGLES_DEF  := -DUSE_OPENGL_ES_1_1
else
    OPENGLES_LIB  := -lGLESv2
    OPENGLES_DEF  := -DUSE_OPENGL_ES_2_0
endif

# Copy the header files to the out directory
LOCAL_COPY_HEADERS_TO           := fastcv
LOCAL_COPY_HEADERS              := fastcv.h fastcv.inl stdint_.h

# An optional set of compiler flags that will be passed when building
# C ***AND*** C++ source files.
#
# NOTE: flag "-Wno-write-strings" removes warning about deprecated conversion
#       from string constant to 'char*'

LOCAL_CFLAGS := -Wno-write-strings $(OPENGLES_DEF)

# The list of additional linker flags to be used when building your
# module. This is useful to pass the name of specific system libraries
# with the "-l" prefix.

LOCAL_LDLIBS := \
     -llog $(OPENGLES_LIB) -lfastcv

LOCAL_MODULE    := libusbfastdemo
LOCAL_CFLAGS    := -Werror
LOCAL_SRC_FILES := \
    capture.cpp \
    render.cpp
#    USBFastRender.cpp \
#    CameraRendererRGB565GL2.cpp \
#    CameraUtil.cpp
#    FastCVSample.cpp \
#    FPSCounter.cpp \

LOCAL_LDFLAGS += -ldl -ljnigraphics       # for dlsym and dlopen
LOCAL_STATIC_LIBRARIES := libfastcv 
LOCAL_SHARED_LIBRARIES := liblog libGLESv2 libandroid
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/fastcv/inc \
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/fastcv

LOCAL_MODULE_TAGS := optional

#LOCAL_MODULE_OWNER := qcom
#LOCAL_PROPRIETARY_MODULE := true

$(shell mkdir -p $(OUT)/obj/STATIC_LIBRARIES/libfastcv_intermediates)
$(shell touch $(OUT)/obj/STATIC_LIBRARIES/libfastcv_intermediates/export_includes)

#include $(BUILD_EXECUTABLE)
include $(BUILD_SHARED_LIBRARY)


#include $(CLEAR_VARS)

#LOCAL_STATIC_JAVA_LIBRARIES :=
#LOCAL_JNI_SHARED_LIBRARIES := libfastcvsample
#LOCAL_SRC_FILES := $(call all-subdir-java-files)
#LOCAL_PACKAGE_NAME := FastCVSample

#include $(BUILD_PACKAGE)

