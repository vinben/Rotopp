#**************************************************************************
#* This file is a part of our work (Siggraph'16 paper, binary, code and dataset):
#*
#* Roto++: Accelerating Professional Rotoscoping using Shape Manifolds
#* Wenbin Li, Fabio Viola, Jonathan Starck, Gabriel J. Brostow and Neill D.F. Campbell
#*
#* w.li AT cs.ucl.ac.uk
#* http://visual.cs.ucl.ac.uk/pubs/rotopp
#*
#* Copyright (c) 2016, Wenbin Li
#* All rights reserved.
#*
#* Redistribution and use in source and binary forms, with or without
#* modification, are permitted provided that the following conditions are met:
#*
#* -- Redistributions of source code and data must retain the above
#*    copyright notice, this list of conditions and the following disclaimer.
#* -- Redistributions in binary form must reproduce the above copyright
#*    notice, this list of conditions and the following disclaimer in the
#*    documentation and/or other materials provided with the distribution.
#*
#* THIS WORK AND THE RELATED SOFTWARE, SOURCE CODE AND DATA IS PROVIDED BY
#* THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
#* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
#* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
#* NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
#* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
#* USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
#* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
#* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#**************************************************************************/

cmake_minimum_required(VERSION 2.8.11)

IF(POLICY CMP0042)
    CMAKE_POLICY(SET CMP0042 NEW)
ENDIF(POLICY CMP0042)

IF(POLICY CMP0043)
    CMAKE_POLICY(SET CMP0043 NEW)
ENDIF(POLICY CMP0043)

set(BUILD_SHARED_LIBS OFF)

PROJECT(Roto)
set (Roto_VERSION_MAJOR 1)
set (Roto_VERSION_MINOR 6)
set (Roto_VERSION_PATCH 0)

option(ENABLE_DEBUG "Output the debug information" ON)
option(ENABLE_SMART_BEZIER "Use smarter Bezier curve" ON)
option(ENABLE_SOLVER_MODE "Enable editing on system mode" ON)
option(ENABLE_INCREASING_SMOOTH "Using increasing ratio on smooth point function" ON)

IF(APPLE)
    set(CMAKE_CXX_FLAGS "-std=c++11 -stdlib=libc++")
    set (CMAKE_PREFIX_PATH "/usr/local/Cellar/qt5/5.5.1_2") # Point this to your qt library for mac
ELSEif(UNIX)
    #    set (CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu") # use the qt5 library from apt-get
    set (CMAKE_PREFIX_PATH "/home/wenbinli/Qt/5.5/gcc_64") # Point this to your qt library for Linux
ENDIF(APPLE)

set(CMAKE_BUILD_TYPE "Release")
if(NOT CMAKE_BUILD_TYPE)
    message("** CMAKE_BUILD_TYPE not passed. Assuming Debug")
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC OFF)

set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH} 
    "${CMAKE_SOURCE_DIR}/Cmake/modules/"
)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

IF(ENABLE_DEBUG)
    set(DEBUG_OUTPUT 1)
ELSE(ENABLE_DEBUG)
    set(DEBUG_OUTPUT 0)
ENDIF(ENABLE_DEBUG)

# Find libraries
find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

set(GXX_COMPILE_FLAGS
    "-g -Wall -Wextra -Wno-unused-variable -Wignored-qualifiers -Wno-reorder -Wno-sign-compare -std=c++11 -pthread -fPIC"
)

IF(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GXX_COMPILE_FLAGS}")
ENDIF()

qt5_add_resources(UI_RESOURCES resources.qrc)

set(HPP_SRC
    ./include/FrameFlow.hpp
    ./include/ImgIcon.hpp
    ./include/utils.hpp
    ./include/mainWindow.hpp
    ./include/RotoCore.hpp
    ./include/RotoIO.hpp
    ./include/RotoStatus.hpp
    ./include/AvdTracker.hpp
    ./include/RotoActiveContour.hpp
    ./include/ConfigIO.hpp
    ./include/designZone/DesignScene.hpp
    ./include/designZone/DesignView.hpp
    ./include/designZone/SniperView.hpp
    ./include/designZone/SpCurve.hpp
    ./include/designZone/SpHook.hpp
    ./include/designZone/SpNode.hpp
    ./include/designZone/SpNodeRotate.hpp
    ./include/designZone/SpPoly.hpp
    ./include/designZone/SpRect.hpp
    ./include/designZone/SpRectRotate.hpp
    ./include/designZone/SpRotator.hpp
    ./include/dialogs/CreateProjDialog.hpp
    ./include/dialogs/LoadProjDialog.hpp
    ./include/dialogs/LoadNukeProjDialog.hpp
    ./include/dialogs/SaveProjDialog.hpp
    ./include/dialogs/ExportProjDialog.hpp
    ./include/dialogs/ImageProcDialog.hpp
    ./include/rotoSolver/kernel.hpp
    ./include/rotoSolver/gplvm.hpp
    ./include/rotoSolver/baseDefs.hpp
    ./include/rotoSolver/transformations.hpp
    ./include/rotoSolver/eigenUtils.hpp
    ./include/rotoSolver/planarTrackerSolver.hpp
    ./include/rotoSolver/fileUtils.hpp
)

qt5_wrap_cpp(MOC_FILES
    ./include/FrameFlow.hpp
    ./include/ImgIcon.hpp
    ./include/mainWindow.hpp
    ./include/RotoCore.hpp
    ./include/RotoIO.hpp
    ./include/RotoStatus.hpp
    ./include/AvdTracker.hpp
    ./include/RotoActiveContour.hpp
    ./include/ConfigIO.hpp
    ./include/designZone/DesignScene.hpp
    ./include/designZone/DesignView.hpp
    ./include/designZone/SniperView.hpp
    ./include/dialogs/CreateProjDialog.hpp
    ./include/dialogs/LoadProjDialog.hpp
    ./include/dialogs/LoadNukeProjDialog.hpp
    ./include/dialogs/SaveProjDialog.hpp
    ./include/dialogs/ExportProjDialog.hpp
    ./include/dialogs/ImageProcDialog.hpp
)

set(CPP_SRC
    ./src/FrameFlow.cpp
    ./src/ImgIcon.cpp
    ./src/main.cpp
    ./src/mainWindow.cpp
    ./src/RotoCore.cpp
    ./src/RotoIO.cpp
    ./src/RotoStatus.cpp
    ./src/utils.cpp
    ./src/AvdTracker.cpp
    ./src/RotoActiveContour.cpp
    ./src/ConfigIO.cpp
    ./src/designZone/DesignScene.cpp
    ./src/designZone/DesignView.cpp
    ./src/designZone/SniperView.cpp
    ./src/designZone/SpCurve.cpp
    ./src/designZone/SpHook.cpp
    ./src/designZone/SpNode.cpp
    ./src/designZone/SpNodeRotate.cpp
    ./src/designZone/SpPoly.cpp
    ./src/designZone/SpRect.cpp
    ./src/designZone/SpRectRotate.cpp
    ./src/designZone/SpRotator.cpp
    ./src/dialogs/CreateProjDialog.cpp
    ./src/dialogs/LoadProjDialog.cpp
    ./src/dialogs/LoadNukeProjDialog.cpp
    ./src/dialogs/SaveProjDialog.cpp
    ./src/dialogs/ExportProjDialog.cpp
    ./src/dialogs/ImageProcDialog.cpp
    ./src/rotoSolver/kernel.cpp
    ./src/rotoSolver/baseDefs.cpp
    ./src/rotoSolver/transformations.cpp
    ./src/rotoSolver/planarTrackerSolver.cpp
    ./src/rotoSolver/fileUtils.cpp
)

configure_file (
    "${PROJECT_SOURCE_DIR}/include/RotoConfig.h.in"
    "${PROJECT_BINARY_DIR}/include/RotoConfig.h"
)

include_directories(
    ./thirdparty/eigen
    ./thirdparty/eigen/unsupported
    ./thirdparty/glog/src
    ./thirdparty/gflags
    ${CERES_INCLUDE_DIRS}
    ${CMAKE_BINARY_DIR}/config
)

add_executable(Roto
    ${HPP_SRC}
    ${CPP_SRC}
    ${MOC_FILES}
    ${UI_RESOURCES})

LINK_DIRECTORIES( ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} )

target_link_libraries(Roto
    ceres
    glog
    gflags
    opencv_core
    opencv_features2d
    opencv_legacy
    ${Qt5Core_QTMAIN_LIBRARIES}
)

#message("-- CERES_INCLUDE_DIRS: ${CERES_INCLUDE_DIRS}")
#message("-- CERES_LIBRARIES: ${CERES_LIBRARIES}")

qt5_use_modules(Roto
    Widgets
    Core
)

add_subdirectory(thirdparty)

