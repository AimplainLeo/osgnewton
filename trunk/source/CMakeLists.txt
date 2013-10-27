# Copyright (c) <2009> <Newton Game Dynamics>
# 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
# 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely


if (buildStaticLibrary)
	set (libraryType STATIC)
#	_NEWTON_STATIC_LIB;_CNEWTON_STATIC_LIB
else (buildStaticLibrary)
	set (libraryType SHARED)
	set (newtonBuildLibraryType "_OSG_NEWTON_BUILD_DLL")
endif (buildStaticLibrary)	

set (projectName osgNewton)

include_directories (${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB_RECURSE headers *.h)
file(GLOB_RECURSE sources *.cpp)

add_library(${projectName} ${libraryType} ${headers} ${sources})

add_definitions (-D_WIN_32_VER -D_CRT_SECURE_NO_WARNINGS -D${newtonBuildLibraryType})

if (MSVC)

	if (NOT buildStaticLibrary)
		target_link_libraries (${projectName} debug ${newtonPath}/packages/projects/${packagesLibPath}/${PlatformName}/dMath/debug/dMath_d.lib
											  debug	${newtonPath}/packages/projects/${packagesLibPath}/${PlatformName}/dNewton/debug/dNewton_d.lib
											  debug ${newtonPath}/coreLibrary_300/projects/windows/${newtonLibPath}/${PlatformName}/newton/Debug/newton_d.lib
											  debug ${newtonPath}/packages/projects/${packagesLibPath}/${PlatformName}/dJointLibrary/debug/dJointLibrary_d.lib
											  debug ${osgSDKBuild}/lib/osgd.lib
											  debug ${osgSDKBuild}/lib/osgDBd.lib
											  debug ${osgSDKBuild}/lib/osgUtild.lib
											  debug ${osgSDKBuild}/lib/osgViewerd.lib
											  debug ${osgSDKBuild}/lib/OpenThreadsd.lib
											  debug opengl32.lib)

		target_link_libraries (${projectName} optimized ${newtonPath}/packages/projects/${packagesLibPath}/${PlatformName}/dMath/release/dMath.lib
											  optimized ${newtonPath}/packages/projects/${packagesLibPath}/${PlatformName}/dNewton/release/dNewton.lib
											  optimized ${newtonPath}/coreLibrary_300/projects/windows/${newtonLibPath}/${PlatformName}/newton/Release/newton.lib
											  optimized ${newtonPath}/packages/projects/${packagesLibPath}/${PlatformName}/dJointLibrary/release/dJointLibrary.lib
											  optimized ${osgSDKBuild}/lib/osg.lib
											  optimized ${osgSDKBuild}/lib/osgDB.lib
											  optimized ${osgSDKBuild}/lib/osgUtil.lib
											  optimized ${osgSDKBuild}/lib/osgViewer.lib
											  optimized ${osgSDKBuild}/lib/OpenThreads.lib
											  optimized opengl32.lib)
											  
		add_custom_command (TARGET ${projectName} POST_BUILD 
							COMMAND xcopy  "\"\$(OutDir)\\*.dll\"" "\"${PROJECT_SOURCE_DIR}\\bin\\debug\\*.*\""  /F /Y /R		
							COMMAND xcopy  "\"\$(OutDir)\\*.pdb\"" "\"${PROJECT_SOURCE_DIR}\\bin\\debug\\*.*\""  /F /Y /R		
							COMMAND xcopy  "\"${newtonPath}\\packages\\projects\\${packagesLibPath}\\$(PlatformName)\\dNewton\\debug\\*.dll\"" "\"${PROJECT_SOURCE_DIR}\\bin\\debug\\*.*\""  /F /Y /R		
							COMMAND xcopy  "\"${newtonPath}\\packages\\projects\\${packagesLibPath}\\$(PlatformName)\\dNewton\\debug\\*.pdb\"" "\"${PROJECT_SOURCE_DIR}\\bin\\debug\\*.*\""  /F /Y /R		
							COMMAND xcopy  "\"${newtonPath}\\packages\\projects\\${packagesLibPath}\\$(PlatformName)\\dJointLibrary\\debug\\*.dll\"" "\"${PROJECT_SOURCE_DIR}\\bin\\debug\\*.*\""  /F /Y /R
							COMMAND xcopy  "\"${newtonPath}\\packages\\projects\\${packagesLibPath}\\$(PlatformName)\\dJointLibrary\\debug\\*.pdb\"" "\"${PROJECT_SOURCE_DIR}\\bin\\debug\\*.*\""  /F /Y /R
							COMMAND xcopy  "\"${newtonPath}\\coreLibrary_300\\projects\\windows\\${newtonLibPath}\\${PlatformName}\\newton\\debug\\*.dll\"" "\"${PROJECT_SOURCE_DIR}\\bin\\debug\\*.*\""  /F /Y /R
							COMMAND xcopy  "\"${newtonPath}\\coreLibrary_300\\projects\\windows\\${newtonLibPath}\\${PlatformName}\\newton\\debug\\*.pdb\"" "\"${PROJECT_SOURCE_DIR}\\bin\\debug\\*.*\""  /F /Y /R
							COMMAND xcopy  "\"${newtonPath}\\packages\\projects\\${packagesLibPath}\\$(PlatformName)\\dNewton\\release\\*.dll\"" "\"${PROJECT_SOURCE_DIR}\\bin\\debug\\*.*\""  /F /Y /R		
							COMMAND xcopy  "\"${newtonPath}\\packages\\projects\\${packagesLibPath}\\$(PlatformName)\\dNewton\\release\\*.pdb\"" "\"${PROJECT_SOURCE_DIR}\\bin\\debug\\*.*\""  /F /Y /R		
							COMMAND xcopy  "\"${newtonPath}\\packages\\projects\\${packagesLibPath}\\$(PlatformName)\\dJointLibrary\\release\\*.dll\"" "\"${PROJECT_SOURCE_DIR}\\bin\\debug\\*.*\""  /F /Y /R		
							COMMAND xcopy  "\"${newtonPath}\\packages\\projects\\${packagesLibPath}\\$(PlatformName)\\dJointLibrary\\release\\*.pdb\"" "\"${PROJECT_SOURCE_DIR}\\bin\\debug\\*.*\""  /F /Y /R		
							COMMAND xcopy  "\"${newtonPath}\\coreLibrary_300\\projects\\windows\\${newtonLibPath}\\${PlatformName}\\newton\\release\\*.dll\"" "\"${PROJECT_SOURCE_DIR}\\bin\\debug\\*.*\""  /F /Y /R		
							COMMAND xcopy  "\"${newtonPath}\\coreLibrary_300\\projects\\windows\\${newtonLibPath}\\${PlatformName}\\newton\\release\\*.pdb\"" "\"${PROJECT_SOURCE_DIR}\\bin\\debug\\*.*\""  /F /Y /R
							
							COMMAND xcopy  "\"\$(OutDir)\\*.dll\"" "\"${PROJECT_SOURCE_DIR}\\bin\\release\\*.*\""  /F /Y /R		
							COMMAND xcopy  "\"\$(OutDir)\\*.pdb\"" "\"${PROJECT_SOURCE_DIR}\\bin\\release\\*.*\""  /F /Y /R		
							COMMAND xcopy  "\"${newtonPath}\\packages\\projects\\${packagesLibPath}\\$(PlatformName)\\dNewton\\debug\\*.dll\"" "\"${PROJECT_SOURCE_DIR}\\bin\\release\\*.*\""  /F /Y /R		
							COMMAND xcopy  "\"${newtonPath}\\packages\\projects\\${packagesLibPath}\\$(PlatformName)\\dNewton\\debug\\*.pdb\"" "\"${PROJECT_SOURCE_DIR}\\bin\\release\\*.*\""  /F /Y /R		
							COMMAND xcopy  "\"${newtonPath}\\packages\\projects\\${packagesLibPath}\\$(PlatformName)\\dJointLibrary\\debug\\*.dll\"" "\"${PROJECT_SOURCE_DIR}\\bin\\release\\*.*\""  /F /Y /R
							COMMAND xcopy  "\"${newtonPath}\\packages\\projects\\${packagesLibPath}\\$(PlatformName)\\dJointLibrary\\debug\\*.pdb\"" "\"${PROJECT_SOURCE_DIR}\\bin\\release\\*.*\""  /F /Y /R
							COMMAND xcopy  "\"${newtonPath}\\coreLibrary_300\\projects\\windows\\${newtonLibPath}\\${PlatformName}\\newton\\debug\\*.dll\"" "\"${PROJECT_SOURCE_DIR}\\bin\\release\\*.*\""  /F /Y /R
							COMMAND xcopy  "\"${newtonPath}\\coreLibrary_300\\projects\\windows\\${newtonLibPath}\\${PlatformName}\\newton\\debug\\*.pdb\"" "\"${PROJECT_SOURCE_DIR}\\bin\\release\\*.*\""  /F /Y /R
							COMMAND xcopy  "\"${newtonPath}\\packages\\projects\\${packagesLibPath}\\$(PlatformName)\\dNewton\\release\\*.dll\"" "\"${PROJECT_SOURCE_DIR}\\bin\\release\\*.*\""  /F /Y /R		
							COMMAND xcopy  "\"${newtonPath}\\packages\\projects\\${packagesLibPath}\\$(PlatformName)\\dNewton\\release\\*.pdb\"" "\"${PROJECT_SOURCE_DIR}\\bin\\release\\*.*\""  /F /Y /R		
							COMMAND xcopy  "\"${newtonPath}\\packages\\projects\\${packagesLibPath}\\$(PlatformName)\\dJointLibrary\\release\\*.dll\"" "\"${PROJECT_SOURCE_DIR}\\bin\\release\\*.*\""  /F /Y /R		
							COMMAND xcopy  "\"${newtonPath}\\packages\\projects\\${packagesLibPath}\\$(PlatformName)\\dJointLibrary\\release\\*.pdb\"" "\"${PROJECT_SOURCE_DIR}\\bin\\release\\*.*\""  /F /Y /R		
							COMMAND xcopy  "\"${newtonPath}\\coreLibrary_300\\projects\\windows\\${newtonLibPath}\\${PlatformName}\\newton\\release\\*.dll\"" "\"${PROJECT_SOURCE_DIR}\\bin\\release\\*.*\""  /F /Y /R		
							COMMAND xcopy  "\"${newtonPath}\\coreLibrary_300\\projects\\windows\\${newtonLibPath}\\${PlatformName}\\newton\\release\\*.pdb\"" "\"${PROJECT_SOURCE_DIR}\\bin\\release\\*.*\""  /F /Y /R)
				  
										  
	endif (NOT buildStaticLibrary)


	set_target_properties(${projectName} PROPERTIES DEBUG_OUTPUT_NAME ${projectName}_d)

	set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/Zm400
																   /YuosgNewtonStdAfx.h
																   /I${osgSDK}/include 
																   /I${osgSDKBuild}/include 
																   /I${newtonPath}/packages/dMath
																   /I${newtonPath}/packages/dNewton
																   /I${newtonPath}/packages/dCustomJoints
																   /I${newtonPath}/coreLibrary_300/source/newton")

	set_source_files_properties(osgNewtonStdAfx.cpp PROPERTIES COMPILE_FLAGS "/YcosgNewtonStdAfx.h")

						
endif(MSVC)



