LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

ROOT=/windows/D/softwares_Linux/pcl-for-android/build/

PCL_STATIC_LIB_DIR := $(ROOT)/pcl-android/lib
BOOST_STATIC_LIB_DIR := $(ROOT)/boost-android/lib
FLANN_STATIC_LIB_DIR := $(ROOT)/flann-android/lib
					
PCL_STATIC_LIBRARIES := pcl_common pcl_kdtree pcl_octree pcl_sample_consensus \
							pcl_surface pcl_features pcl_io pcl_keypoints pcl_recognition \
							pcl_search pcl_tracking pcl_filters pcl_io_ply \
							pcl_registration pcl_segmentation 
BOOST_STATIC_LIBRARIES := boost_date_time boost_iostreams boost_regex boost_system \
							boost_filesystem boost_program_options boost_signals \
							boost_thread
FLANN_STATIC_LIBRARIES := flann_cpp_s flann_s


define build_pcl_static
	include $(CLEAR_VARS)
	LOCAL_MODULE:=$1
	LOCAL_SRC_FILES:=$(PCL_STATIC_LIB_DIR)/lib$1.a
	include $(PREBUILT_STATIC_LIBRARY)
endef

define build_boost_static
	include $(CLEAR_VARS)
	LOCAL_MODULE:=$1
	LOCAL_SRC_FILES:=$(BOOST_STATIC_LIB_DIR)/lib$1.a
	include $(PREBUILT_STATIC_LIBRARY)
endef

define build_flann_static
	include $(CLEAR_VARS)
	LOCAL_MODULE:=$1
	LOCAL_SRC_FILES:=$(FLANN_STATIC_LIB_DIR)/lib$1.a
	include $(PREBUILT_STATIC_LIBRARY)
endef


$(foreach module,$(PCL_STATIC_LIBRARIES),$(eval $(call build_pcl_static,$(module))))
$(foreach module,$(BOOST_STATIC_LIBRARIES),$(eval $(call build_boost_static,$(module))))
$(foreach module,$(FLANN_STATIC_LIBRARIES),$(eval $(call build_flann_static,$(module))))

include $(CLEAR_VARS)

LOCAL_STATIC_LIBRARIES   += pcl_common pcl_kdtree pcl_octree pcl_sample_consensus \
							pcl_surface pcl_features pcl_io pcl_keypoints pcl_recognition \
							pcl_search pcl_tracking pcl_filters pcl_io_ply \
							pcl_registration pcl_segmentation 
							
LOCAL_STATIC_LIBRARIES   += boost_date_time boost_iostreams boost_regex boost_system \
							boost_filesystem boost_program_options boost_signals \
							boost_thread


LOCAL_SRC_FILES += src/jni_hellopcl.cpp

LOCAL_C_INCLUDES += \
    /usr/local/include/eigen3 \
    $(ROOT)/pcl-android/include/\
    $(ROOT)/flann-android/include/ \
    $(ROOT)/boost-android/include \
    $(LOCAL_PATH)/include \
    $(ROOT)/g2o/EXTERNAL/csparse \

LOCAL_CFLAGS += -std=c++11 -Wno-deprecated-declarations
LOCAL_CPPFLAGS += -std=c++11 -O3
LOCAL_MODULE := test
LOCAL_ARM_MODE := arm
LOCAL_LDLIBS += -landroid -lGLESv1_CM -lGLESv2 -llog -fopenmp
LOCAL_CFLAGS += -g

#LOCAL_CFLAGS += -fopenmp
#LOCAL_LDFLAGS += -fopenmp

include $(BUILD_SHARED_LIBRARY)
