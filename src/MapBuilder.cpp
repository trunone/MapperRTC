#include "MapBuilder.h"

MapBuilder *MapBuilder::unique_instance = new MapBuilder();

MapBuilder::MapBuilder() {

	// -----------------------------------------------------------------------------------
	//            MRPT
	// -----------------------------------------------------------------------------------
	CConfigFile				iniFile("icp-slam.ini");

	// ------------------------------------------
	//			Load config from file:
	// ------------------------------------------
	rawlog_offset		 = iniFile.read_int("MappingApplication","rawlog_offset",0,  /*Force existence:*/ true);
	OUT_DIR_STD			 = iniFile.read_string("MappingApplication","logOutput_dir","log_out",  /*Force existence:*/ true);
	LOG_FREQUENCY		 = iniFile.read_int("MappingApplication","LOG_FREQUENCY",5,  /*Force existence:*/ true);
	SAVE_POSE_LOG		 = iniFile.read_bool("MappingApplication","SAVE_POSE_LOG", false,  /*Force existence:*/ true);
	SAVE_3D_SCENE        = iniFile.read_bool("MappingApplication","SAVE_3D_SCENE", false,  /*Force existence:*/ true);
	CAMERA_3DSCENE_FOLLOWS_ROBOT = iniFile.read_bool("MappingApplication","CAMERA_3DSCENE_FOLLOWS_ROBOT", true,  /*Force existence:*/ true);

	SHOW_PROGRESS_3D_REAL_TIME = false;
	SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS = 0;

	MRPT_LOAD_CONFIG_VAR( SHOW_PROGRESS_3D_REAL_TIME, bool,  iniFile, "MappingApplication");
	MRPT_LOAD_CONFIG_VAR( SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS, int, iniFile, "MappingApplication");

	OUT_DIR = OUT_DIR_STD.c_str();

	// ------------------------------------
	//		Constructor of ICP-SLAM object
	// ------------------------------------
	map_builder_icp.ICP_options.loadFromConfigFile( iniFile, "MappingApplication");
	map_builder_icp.ICP_params.loadFromConfigFile ( iniFile, "ICP");

	// Construct the maps with the loaded configuration.
	map_builder_icp.initialize();


	// ---------------------------------
	//   CMetricMapBuilder::TOptions
	// ---------------------------------
	map_builder_icp.options.verbose = true;
	map_builder_icp.options.alwaysInsertByClass.fromString( iniFile.read_string("MappingApplication","alwaysInsertByClass","") );


	// Print params:
//	printf("Running with the following parameters:\n");
//	printf(" Output directory:\t\t\t'%s'\n",OUT_DIR);
//	printf(" matchAgainstTheGrid:\t\t\t%c\n", map_builder_icp.ICP_options.matchAgainstTheGrid ? 'Y':'N');
//	printf(" Log record freq:\t\t\t%u\n",LOG_FREQUENCY);
//	printf("  SAVE_3D_SCENE:\t\t\t%c\n", SAVE_3D_SCENE ? 'Y':'N');
//	printf("  SAVE_POSE_LOG:\t\t\t%c\n", SAVE_POSE_LOG ? 'Y':'N');
//	printf("  CAMERA_3DSCENE_FOLLOWS_ROBOT:\t%c\n",CAMERA_3DSCENE_FOLLOWS_ROBOT ? 'Y':'N');
//
//	printf("\n");

	map_builder_icp.ICP_params.dumpToConsole();
	map_builder_icp.ICP_options.dumpToConsole();

	// Prepare output directory:
	// --------------------------------
	deleteFilesInDirectory(OUT_DIR);
	createDirectory(OUT_DIR);

	// Checks:
	step = 0;

	// Open log files:
	// ----------------------------------
	f_log.open(format("%s/log_times.txt",OUT_DIR));
	f_path.open(format("%s/log_estimated_path.txt",OUT_DIR));
	f_pathOdo.open(format("%s/log_odometry_path.txt",OUT_DIR));

	// Create 3D window if requested:
#if MRPT_HAS_WXWIDGETS
	if (SHOW_PROGRESS_3D_REAL_TIME)
	{
		win3D = CDisplayWindow3D::Create("ICP-SLAM @ MRPT C++ Library", 600, 500);
		win3D->setCameraZoom(20);
		win3D->setCameraAzimuthDeg(-45);
	}
#endif
}

bool MapBuilder::StopMapping() {
    outputFile.close();
	f_log.close();
	f_path.close();
	f_pathOdo.close();

	// Save map:
	map_builder_icp.getCurrentlyBuiltMap(finalMap);

    COccupancyGridMap2D picture;
    picture.loadFromSimpleMap(finalMap);
    picture.saveAsBitmapFile("test.bmp");

	str = format("%s/_finalmap_.simplemap",OUT_DIR);
	printf("Dumping final map in binary format to: %s\n", str.c_str() );
	map_builder_icp.saveCurrentMapToFile(str);

	CMultiMetricMap  *finalPointsMap = map_builder_icp.getCurrentlyBuiltMetricMap();
	str = format("%s/_finalmaps_.txt",OUT_DIR);
	printf("Dumping final metric maps to %s_XXX\n", str.c_str() );
	finalPointsMap->saveMetricMapRepresentationToFile( str );

	map_builder_icp.clear();

    return false;
}

bool MapBuilder::InitMapping() {
	// Rawlog
	outputFile.open(format("%s/dataset.rawlog",OUT_DIR));

    return false;
}

bool MapBuilder::StartMapping(
        CActionCollection action_collection,
        CSensoryFrame sensory_frame
) {
    // ----------------------------------------------------------
    //						Map Building
    // ----------------------------------------------------------
	CPose2D	odoPose(0,0,0);

	// Execute:
	// ----------------------------------------
	map_builder_icp.processActionObservation( action_collection, sensory_frame );

	// Info log:
	// -----------
	f_log.printf("%f %i\n",1000.0f*t_exec,map_builder_icp.getCurrentlyBuiltMapSize() );

	const CMultiMetricMap* mostLikMap =  map_builder_icp.getCurrentlyBuiltMetricMap();

	// Save a 3D scene view of the mapping process:
	if (0==(step % LOG_FREQUENCY) || (SAVE_3D_SCENE || win3D.present()))
	{
		CPose3D robotPose;
		map_builder_icp.getCurrentPoseEstimation()->getMean(robotPose);

		COpenGLScenePtr		scene = COpenGLScene::Create();

		COpenGLViewportPtr view=scene->getViewport("main");
		ASSERT_(view);

		COpenGLViewportPtr view_map = scene->createViewport("mini-map");
		view_map->setBorderSize(2);
		view_map->setViewportPosition(0.01,0.01,0.35,0.35);
		view_map->setTransparent(false);

		{
			mrpt::opengl::CCamera &cam = view_map->getCamera();
			cam.setAzimuthDegrees(-90);
			cam.setElevationDegrees(90);
			cam.setPointingAt(robotPose);
			cam.setZoomDistance(20);
			cam.setOrthogonal();
		}

		// The ground:
		mrpt::opengl::CGridPlaneXYPtr groundPlane = mrpt::opengl::CGridPlaneXY::Create(-200,200,-200,200,0,5);
		groundPlane->setColor(0.4,0.4,0.4);
		view->insert( groundPlane );
		view_map->insert( CRenderizablePtr( groundPlane) ); // A copy

		// The camera pointing to the current robot pose:
		if (CAMERA_3DSCENE_FOLLOWS_ROBOT)
		{
			scene->enableFollowCamera(true);

			mrpt::opengl::CCamera &cam = view_map->getCamera();
			cam.setAzimuthDegrees(-45);
			cam.setElevationDegrees(45);
			cam.setPointingAt(robotPose);
		}

		// The maps:
		{
			opengl::CSetOfObjectsPtr obj = opengl::CSetOfObjects::Create();
			mostLikMap->getAs3DObject( obj );
			view->insert(obj);

			// Only the point map:
			opengl::CSetOfObjectsPtr ptsMap = opengl::CSetOfObjects::Create();
			if (mostLikMap->m_pointsMaps.size())
			{
				mostLikMap->m_pointsMaps[0]->getAs3DObject(ptsMap);
				view_map->insert( ptsMap );
			}
		}

		// Draw the robot path:
		CPose3DPDFPtr posePDF =  map_builder_icp.getCurrentPoseEstimation();
		CPose3D  curRobotPose;
		posePDF->getMean(curRobotPose);
		{
			opengl::CSetOfObjectsPtr obj = opengl::stock_objects::RobotPioneer();
			obj->setPose( curRobotPose );
			view->insert(obj);
		}
		{
			opengl::CSetOfObjectsPtr obj = opengl::stock_objects::RobotPioneer();
			obj->setPose( curRobotPose );
			view_map->insert( obj );
		}

		// Show 3D?
		if (win3D)
		{
			opengl::COpenGLScenePtr &ptrScene = win3D->get3DSceneAndLock();
			ptrScene = scene;

			win3D->unlockAccess3DScene();

			// Move camera:
			win3D->setCameraPointingToPoint( curRobotPose.x(),curRobotPose.y(),curRobotPose.z() );

			// Update:
			win3D->forceRepaint();

			sleep( SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS );
		}
	}

    est_x = map_builder_icp.getCurrentPoseEstimation()->getMeanVal().x();
    est_y = map_builder_icp.getCurrentPoseEstimation()->getMeanVal().y();
    est_th = map_builder_icp.getCurrentPoseEstimation()->getMeanVal().yaw();

	step++;

    return false;
}

bool MapBuilder::get_map(CSimpleMap& map) {
    //map_builder_icp.getCurrentlyBuiltMap(map);

    map_builder_icp.saveCurrentMapToFile("/tmp/icp_tmp.simplemap", false);

    return false;
}

