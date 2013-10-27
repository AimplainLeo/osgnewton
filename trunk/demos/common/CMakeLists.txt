# Copyright (c) <2009> <Newton Game Dynamics>
# 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
# 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely


set (projectName common)

include_directories (${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB_RECURSE headers *.h)
file(GLOB_RECURSE sources *.cpp)

add_library(${projectName} STATIC ${headers} ${sources})
add_dependencies (${projectName} osgNewton)

#add_definitions (-D_CRT_SECURE_NO_WARNINGS -D_NEWTON_STATIC_LIB -D_CNEWTON_STATIC_LIB -D_OSG_NEWTON_STATIC_LIB)

set_target_properties(${projectName} PROPERTIES DEBUG_OUTPUT_NAME ${projectName}_d)

set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/Zm400
															   /YuHelpersStdAfx.h
															   /I${CMAKE_CURRENT_SOURCE_DIR}/../../source
															   /I${osgSDK}/include 
															   /I${osgSDKBuild}/include 
															   /I${newtonPath}/packages/dMath
															   /I${newtonPath}/packages/dNewton
															   /I${newtonPath}/packages/dCustomJoints
															   /I${newtonPath}/coreLibrary_300/source/newton")

set_source_files_properties (HelpersStdAfx.cpp PROPERTIES COMPILE_FLAGS "/YcHelpersStdAfx.h")
