
/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/


#include "HelpersStdAfx.h"
#include "Utils.h"



osg::Node* FindNodeByName (const std::string& name, osg::Node* const rootNode)
{

	class TraverseNode: public osg::NodeVisitor 
	{
		public:
		TraverseNode(const std::string& name)
			:osg::NodeVisitor (NodeVisitor::TRAVERSE_ALL_CHILDREN)
			,m_found(NULL)
			,m_name(name)
		{
		}


		virtual void apply(osg::Node& node)
		{
			if (m_name == node.getName()) {
				m_found = & node;
				return;
			}
			traverse (node);
		}

		osg::Node* m_found;
		const std::string m_name;
	};

	TraverseNode traverse (name);
	rootNode->accept(traverse);
	return traverse.m_found;
}

class DemosFindFileCallback: public osgDB::FindFileCallback
{
	public:

	virtual std::string findDataFile(const std::string& filename, const osgDB::Options* options, osgDB::CaseSensitivity caseSensitivity)
	{
		#if defined (_MSC_VER)
				char appPath [256];
				GetModuleFileNameA(NULL, appPath, sizeof (appPath));
				strlwr (appPath);
				char* const end = strstr (appPath, "bin\\");
				end [0] = 0;
				if (filename.find(appPath) != -1) {
					return filename;
				}
				return std::string (appPath) + std::string ("bin\\media\\") + filename;
		#elif defined(_MACOSX_VER)
				char tmp[2048];
				CFURLRef appURL (CFBundleCopyBundleURL(CFBundleGetMainBundle()));
				CFStringRef filePath (CFURLCopyFileSystemPath (appURL, kCFURLPOSIXPathStyle));
				CFStringGetCString (filePath, tmp, PATH_MAX, kCFStringEncodingUTF8);
				//char* const ptr = strstr (tmp, "applications");
				//ptr [0] = 0;
				//sprintf (outPathName, "%sapplications/media/%s", tmp, name);
				sprintf (outPathName, "%s/Contents/Resources/%s", tmp, name);

				// Clean up 
				CFRelease( appURL ); 
				CFRelease( filePath );

		#elif (defined (_POSIX_VER) || defined (_POSIX_VER_64))

				char id[2048];
				char appPath[2048];

				sprintf(id, "/proc/%d/exe", getpid());
				memset (appPath, 0, sizeof (appPath));
				readlink(id, appPath, sizeof (appPath));
				char* const end = strstr (appPath, "applications");
				*end = 0;
				sprintf (outPathName, "%sapplications/media/%s", appPath, name);

		#else
		#error  "error: need to implement \"GetWorkingFileName\" here for this platform"
		#endif

	}

	virtual std::string findLibraryFile(const std::string& filename, const osgDB::Options* options, osgDB::CaseSensitivity caseSensitivity)
	{
		return filename;
	}
};


dFloat Rand (dFloat base)
{
	return 2.0f * base * (dFloat(rand()) / RAND_MAX - 0.5f); 
}


void InitWindowsSystem (osgViewer::Viewer& viewer, const char* const title, int x, int y, int width, int height)
{
	viewer.setUpViewInWindow(x, y, width, height);

	//Get the traits of the current window
	osg::ref_ptr< osg::GraphicsContext::Traits > traits = new osg::GraphicsContext::Traits( *viewer.getCamera()->getGraphicsContext()->getTraits());

	//Set the title
	traits->windowName = title;

	// disable vertical sync
	traits->vsync = false;

	//Create a new graphics context with the modified traits
	osg::ref_ptr< osg::GraphicsContext > gc = osg::GraphicsContext::createGraphicsContext( traits.get() );
	gc->realize();
	gc->makeCurrent();

	// set the vertical black off by default
	osgViewer::Viewer::Windows windows;
	viewer.getWindows(windows);
	for(osgViewer::Viewer::Windows::iterator itr = windows.begin(); itr != windows.end(); ++itr) {
		osgViewer::GraphicsWindow* const graphicsWindow = (*itr);
		graphicsWindow->setSyncToVBlank (false);
	}

	//Create the new camera which is a copy of the current camera in the viewer
	osg::ref_ptr<osg::Camera> cam = new osg::Camera( *viewer.getCamera() ); 

	//Set the cameras graphics context to the gc we made above
	cam->setGraphicsContext( gc );

	//assign the viewer the new camera
	viewer.setCamera( cam.get() );

	// int the camera default perspective matrix
	SetCameraProjectionMatrix (viewer, 60.0f, dFloat (width) / height, 0.01, 1000.0f);


	// set the file find callback
	osg::ref_ptr<DemosFindFileCallback> filecallback = new DemosFindFileCallback;
	osgDB::Registry::instance()->setFindFileCallback (filecallback.get());
}


void SetCameraProjectionMatrix (osgViewer::Viewer& viewer, dFloat viewAngleInDegress, dFloat aspectRatio, dFloat nearPlane, dFloat farPlane)
{
	osg::Camera* const camera = viewer.getCamera();
	camera->setProjectionMatrixAsPerspective (viewAngleInDegress, aspectRatio, nearPlane, farPlane);

//	camera->setViewMatrixAsLookAt (osg::Vec3 (-3.0f, -5.0f, 2.0f), osg::Vec3 (0.0f, 0.0f, 2.0f), osg::Vec3 (0.0f, 0.0f, 1.0f));
	camera->setViewMatrixAsLookAt (osg::Vec3 (-0.0f, -5.0f, 2.0f), osg::Vec3 (0.0f, 0.0f, 2.0f), osg::Vec3 (0.0f, 0.0f, 1.0f));
}



