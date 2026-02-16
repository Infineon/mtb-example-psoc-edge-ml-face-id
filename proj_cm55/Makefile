################################################################################
# \file Makefile
# \version 1.0
#
# \brief
# Makefile for configuring the Cortex-M55 core project
#
################################################################################
# \copyright
# (c) 2023-2026, Infineon Technologies AG, or an affiliate of Infineon
# Technologies AG.  SPDX-License-Identifier: Apache-2.0
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

include ../common.mk

################################################################################
# Basic Configuration
################################################################################

# Name of application (used to derive name of final linked file).
#
# If APPNAME is edited, ensure to update or regenerate launch
# configurations for your IDE.
APPNAME=proj_cm55

# If set to "true" or "1", display full command-lines when building.
VERBOSE=

COMPONENTS+=FREERTOS RTOS_AWARE USBH_BASE CMSIS_DSP U55

# Like COMPONENTS, but disable optional code that was enabled by default.
DISABLE_COMPONENTS+=

# Set to "true" to enable the profiling of the model, otherwise disabled
ENABLE_FACEID_MODEL_PROFILE=

# Set number of enrollment poses here - default 5 so we can have better pose variability
NUM_POSES=5

# Set to "true" to print face detection and face ID confidence scores
PRINT_SCORES=true

# Set to "true" to draw head pose axes
DRAW_HEAD_POSE_AXES=

# Set to "true" to draw facial landmarks
DRAW_FACIAL_LANDMARKS=

# Set to "true" to plot aligned faces for debug purposes
PLOT_ALIGNED_FACES=

# Set to "true" to run the demo on a saved image instead of the USB webcam buffer frames
TEST_SAVED_IMAGE=

# Set to "true" to use bigger font for bounding box labels
USE_BIGGER_FONT=true

# Set to "true" to show model inferencing time instead of FPS
SHOW_INFERENCING_TIME=true

################################################################################
# MTB ML / FaceID Specific Configuration
################################################################################

# Quantization type and TFLM memory selection
COMPONENTS+=ML_INT8x8 ML_TFLM
DEFINES+=TF_LITE_STATIC_MEMORY

# Add define to build code for CM55 and include additional files
DEFINES+=CY_ML_MODEL_MEM=.cy_socmem_data


# Use source files for application_code since it is not a lib we are releasing as stand-alone
INCLUDES+=../application_code/include
SEARCH+=../application_code/source
INCLUDES+=../images
INCLUDES+=$(SEARCH_cmsis)/COMPONENT_CMSIS_NN/Include

# FaceID Library Includes
INCLUDES+=../libraries/ifx_face_id/
LDLIBS+=../libraries/ifx_face_id/ifx_face_id.a

ifeq ($(PRINT_SCORES),true)
    DEFINES+=PRINT_SCORES
endif

ifeq ($(DRAW_HEAD_POSE_AXES),true)
    DEFINES+=DRAW_HEAD_POSE_AXES
endif

ifeq ($(DRAW_FACIAL_LANDMARKS),true)
    DEFINES+=DRAW_FACIAL_LANDMARKS
endif

ifeq ($(PLOT_ALIGNED_FACES),true)
    DEFINES+=PLOT_ALIGNED_FACES
endif

ifeq ($(TEST_SAVED_IMAGE),true)
    DEFINES+=TEST_SAVED_IMAGE
endif

ifeq ($(USE_BIGGER_FONT),true)
    DEFINES+=USE_BIGGER_FONT
endif

ifeq ($(SHOW_INFERENCING_TIME),true)
    DEFINES+=SHOW_INFERENCING_TIME
endif

################################################################################
# Application Specific Configuration
################################################################################

# Application Specific Defines
DEFINES+=CAMERA_WIDTH=320 CAMERA_HEIGHT=240 IMAGE_WIDTH=320 IMAGE_HEIGHT=240 NUM_POSES=$(NUM_POSES)

################################################################################
# Advanced Configuration
################################################################################

# Enable optional code that is ordinarily disabled by default.
#
# Available components depend on the specific targeted hardware and firmware
# in use. In general, if you have
#
#    COMPONENTS=foo bar
#
# ... then code in directories named COMPONENT_foo and COMPONENT_bar will be
# added to the build
#
COMPONENTS+=

# Like COMPONENTS, but disable optional code that was enabled by default.
DISABLE_COMPONENTS=

CORE=CM55
CORE_NAME=CM55_0
NPU=U55

# By default the build system automatically looks in the Makefile's directory
# tree for source code and builds it. The SOURCES variable can be used to
# manually add source code to the build process from a location not searched
# by default, or otherwise not found by the build system.
SOURCES+=

# Like SOURCES, but for include directories. Value should be paths to
# directories (without a leading -I).
INCLUDES+=


CY_ML_MIDDLEWARE_PATH=$(SEARCH_ml-middleware)



# Ignore few ml-middleware files and use the patched ones present in proj_cm55
CY_IGNORE+=${CY_ML_MIDDLEWARE_PATH}/include/mtb_ml_model.h
CY_IGNORE+=${CY_ML_MIDDLEWARE_PATH}/source/COMPONENT_ML_TFLM/mtb_ml_model.cpp

# Add additional defines related to ARM Helium and DSP extensions
DEFINES+=ARM_MATH_HELIUM ARM_MATH_DSP ARM_MATH_AUTOVECTORIZE CY_RETARGET_IO_CONVERT_LF_TO_CRLF _BAREMETAL=0 USE_BLOCKING_API ETHOSU55 USBH_DISABLE_STANDARD_OUTPUT

# Check which kit is being used
ifeq (APP_KIT_PSE84_AI, $(TARGET))
DEFINES+=USE_KIT_PSE84_AI
endif

# Select softfp or hardfp floating point. Default is softfp.
VFP_SELECT+=hardfp

# Additional / custom C compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
CFLAGS+=

# Additional / custom C++ compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
CXXFLAGS+=

# Additional / custom assembler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
ASFLAGS+=

# Additional / custom linker flags.
LDFLAGS+=

# Additional / custom libraries to link in to the application.
LDLIBS+=

# Path to the linker script to use (if empty, use the default linker script).
LINKER_SCRIPT=

# Custom pre-build commands to run.
PREBUILD=

# Custom post-build commands to run.
POSTBUILD=


################################################################################
# Paths
################################################################################

# Relative path to the project directory (default is the Makefile's directory).
#
# This controls where automatic source code discovery looks for code.
CY_APP_PATH=

# Relative path to the shared repo location.
#
# All .mtb files have the format, <URI>#<COMMIT>#<LOCATION>. If the <LOCATION>
# field begins with $$ASSET_REPO$$, then the repo is deposited in the path
# specified by the CY_GETLIBS_SHARED_PATH variable. The default location is one
# directory level above the current app directory. This is used with
# CY_GETLIBS_SHARED_NAME variable, which specifies the directory name.
CY_GETLIBS_SHARED_PATH=../..

# Directory name of the shared repo location.
#
CY_GETLIBS_SHARED_NAME=mtb_shared

include $(CY_TOOLS_DIR)/make/start.mk
