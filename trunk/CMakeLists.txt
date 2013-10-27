# Copyright (c) <2009> <Newton Game Dynamics>
# 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
# 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely

cmake_minimum_required (VERSION 2.8.11)
project (osgNewton)

set(CMAKE_USE_RELATIVE_PATHS ON)

option (buildStaticLibrary "build static library" false)   

set (newtonPath "$(NEWTON_DYNAMICS)")
set (osgSDK "$(OSG_SDK_PATH)")
set (osgSDKBuild "$(OSG_SDK_BUILD_PATH)")
set (PlatformName "$(PlatformName)")


if (buildStaticLibrary)
#	_NEWTON_STATIC_LIB;_CNEWTON_STATIC_LIB
	
else (buildStaticLibrary)
	if (MSVC_VERSION EQUAL 1500)
		set (newtonLibPath project_vs2008_dll)
		set (packagesLibPath visualStudio_2008_dll)
	endif (MSVC_VERSION EQUAL 1500)
	
	if (MSVC_VERSION EQUAL 1600)
		set (newtonLibPath project_vs2010_dll)
		set (packagesLibPath visualStudio_2010_dll)
	endif (MSVC_VERSION EQUAL 1600)
		
	if (MSVC_VERSION EQUAL 1700)
		set (newtonLibPath project_vs2012_dll)
		set (packagesLibPath visualStudio_2012_dll)
	endif (MSVC_VERSION EQUAL 1700)
endif (buildStaticLibrary)	


add_subdirectory (source)
add_subdirectory (demos)





