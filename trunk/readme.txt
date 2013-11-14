Testing the wrapper binary examples:
1- Go to folder: C:\develoment\osgnewton\bin\release
2-You will find a series of compiled demos. Try some or all them before you try anything else.

Building the library:
1-DownLoad Newton SDK 3.13 (minimum)
2-Open visual studio 2010 solution in folder: ..\newton-dynamics\packages\projects\visualStudio_2010\build.sln

3-You will need to set few environment variables: 
Say your open scenegraph SDK is installed in folder: C:\development\OpenSceneGraph-3.0.1\projects_vs2010
and the newton SDK is installed in forder: C:\development\newton-dynamics

these environment  variables should point to:
OSG_SDK_PATH=C:\Development\OpenSceneGraph-3.0.1
OSG_SDK_BUILD_PATH=C:\Development\OpenSceneGraph-3.0.1\vs2010_sln
NEWTON_DYNAMICS=C:\development\newton-dynamics

4-Now with visual studio 2010, open solution:
C:\development\newton-dynamics\packages\projects\visualStudio_2010_dll\ogreNewton.sln

buidl the debug and release targets.

5-in CMake drag file C:\develoment\osgnewton\CMakeLists.txt to the Cmake GUI

6-select visual studio 2010 compiler and click config and then generate solution

7-now you can open the solution with Visual studio 2010 and build the wrapper and all demos.






 



