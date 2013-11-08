# Copyright (c) <2009> <Newton Game Dynamics>
# 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
# 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely

set (projectName demo04_basicJoints)

include_directories (${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB_RECURSE headers *.h)
file(GLOB_RECURSE sources *.cpp)

add_executable(${projectName} ${headers} ${sources})
add_dependencies (${projectName} common)
add_dependencies (${projectName} osgNewton)

#add_definitions (-D_CRT_SECURE_NO_WARNINGS -D_NEWTON_STATIC_LIB -D_CNEWTON_STATIC_LIB -D_OSG_NEWTON_STATIC_LIB)


set_target_properties(${projectName} PROPERTIES DEBUG_OUTPUT_NAME ${projectName}_d)

set_target_properties(${projectName} PROPERTIES
									 RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../bin/")

set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/I${CMAKE_CURRENT_SOURCE_DIR}/../common
															   /I${CMAKE_CURRENT_SOURCE_DIR}/../../source
															   /I${osgSDK}/include 
															   /I${osgSDKBuild}/include 
															   /I${newtonPath}/packages/dMath
															   /I${newtonPath}/packages/dNewton
															   /I${newtonPath}/packages/dCustomJoints
															   /I${newtonPath}/coreLibrary_300/source/newton")

target_link_libraries (${projectName} debug ../common/debug/common_d
									  debug ../../source/debug/osgNewton_d
									  debug opengl32.lib
									  debug ${osgSDKBuild}/lib/osgd.lib
									  debug ${osgSDKBuild}/lib/osgGAd.lib
									  debug ${osgSDKBuild}/lib/osgDBd.lib
									  debug ${osgSDKBuild}/lib/osgUtild.lib
									  debug ${osgSDKBuild}/lib/osgTextd.lib
									  debug ${osgSDKBuild}/lib/osgViewerd.lib
									  debug ${osgSDKBuild}/lib/OpenThreadsd.lib
									  debug ${newtonPath}/packages/projects/${packagesLibPath}/${PlatformName}/dNewton/debug/dNewton_d.lib)
									  

target_link_libraries (${projectName} optimized ../common/release/common
									  optimized ../../source/release/osgNewton
									  optimized opengl32.lib
									  optimized ${osgSDKBuild}/lib/osg.lib
									  optimized ${osgSDKBuild}/lib/osgGA.lib
									  optimized ${osgSDKBuild}/lib/osgDB.lib
									  optimized ${osgSDKBuild}/lib/osgUtil.lib
									  optimized ${osgSDKBuild}/lib/osgText.lib
									  optimized ${osgSDKBuild}/lib/osgViewer.lib
									  optimized ${osgSDKBuild}/lib/OpenThreads.lib
									  optimized ${newtonPath}/packages/projects/${packagesLibPath}/${PlatformName}/dNewton/release/dNewton.lib)


