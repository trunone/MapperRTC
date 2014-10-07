#include "MapBuilder.h"

#define SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS 5
#define SHOW_PROGRESS_3D_REAL_TIME true
#define CAMERA_3DSCENE_FOLLOWS_ROBOT true
#define SAVE_3D_SCENE false
#define LOG_FREQUENCY 5

MapBuilder *MapBuilder::unique_instance_ = new MapBuilder();

MapBuilder::MapBuilder() {
}

bool MapBuilder::StopMapping() {

	map_builder_icp_.clear();

    return false;
}

bool MapBuilder::InitMapping() {
    // ICP algorithm params
    map_builder_icp_.ICP_params.maxIterations                   = 80;
    map_builder_icp_.ICP_params.minAbsStep_trans                = 1e-6;
    map_builder_icp_.ICP_params.minAbsStep_rot                  = 1e-6;
    map_builder_icp_.ICP_params.thresholdDist                   = 0.3;
    map_builder_icp_.ICP_params.thresholdAng                    = DEG2RAD(5.0);
    map_builder_icp_.ICP_params.ALFA                            = 0.8;
    map_builder_icp_.ICP_params.smallestThresholdDist           = 0.05;
    map_builder_icp_.ICP_params.onlyClosestCorrespondences      = true;
    map_builder_icp_.ICP_params.ICP_algorithm                   = icpClassic;
    map_builder_icp_.ICP_params.corresponding_points_decimation = 5;

    // ICP application options
    map_builder_icp_.ICP_options.localizationLinDistance = 0.2;
    map_builder_icp_.ICP_options.localizationAngDistance = 5;
    map_builder_icp_.ICP_options.insertionLinDistance    = 1.2;
    map_builder_icp_.ICP_options.insertionAngDistance    = 45.0;
    map_builder_icp_.ICP_options.minICPgoodnessToAccept  = 0.40;
    map_builder_icp_.ICP_options.matchAgainstTheGrid     = 0;

    //map_builder_icp_.ICP_options.mapInitializers.options.occupancyGrid_count=0;
    //map_builder_icp_.ICP_options.mapInitializers.options.gasGrid_count=0;
    //map_builder_icp_.ICP_options.mapInitializers.options.landmarksMap_count=0;
    //map_builder_icp_.ICP_options.mapInitializers.options.beaconMap_count=0;
    //map_builder_icp_.ICP_options.mapInitializers.options.pointsMap_count=1;

    map_builder_icp_.ICP_options.mapInitializers.options.likelihoodMapSelection =
            CMultiMetricMap::TOptions::mapFuseAll;

    TMetricMapInitializer map_init;

    map_init.metricMapClassType = CLASS_ID(CSimplePointsMap);
    map_init.pointsMapOptions_options.insertionOpts.minDistBetweenLaserPoints = 0.05;
    map_init.pointsMapOptions_options.insertionOpts.fuseWithExisting = false;
    map_init.pointsMapOptions_options.insertionOpts.isPlanarMap = 1;

    map_builder_icp_.ICP_options.mapInitializers.push_back(map_init);

	// Construct the maps with the loaded configuration.
	map_builder_icp_.initialize();

    // Debug Info.
	map_builder_icp_.options.verbose = true;
	map_builder_icp_.options.alwaysInsertByClass.fromString("");

	map_builder_icp_.ICP_params.dumpToConsole();
	map_builder_icp_.ICP_options.dumpToConsole();

	// Checks:
	step_ = 0;

	// Create 3D window if requested:
#if MRPT_HAS_WXWIDGETS
	if (SHOW_PROGRESS_3D_REAL_TIME)
	{
		win3d_ptr_ = CDisplayWindow3D::Create("ICP-SLAM @ MRPT C++ Library", 600, 500);
		win3d_ptr_->setCameraZoom(20);
		win3d_ptr_->setCameraAzimuthDeg(-45);
	}
#endif

    return false;
}

bool MapBuilder::StartMapping(
        CActionCollection action_collection,
        CSensoryFrame sensory_frame
) {
    // Building map
	map_builder_icp_.processActionObservation( action_collection, sensory_frame );

	// Save a 3D scene view of the mapping process:
	if (0==(step_ % LOG_FREQUENCY) || (SAVE_3D_SCENE || win3d_ptr_.present()))
	{
		CPose3D robotPose;
		map_builder_icp_.getCurrentPoseEstimation()->getMean(robotPose);

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
			map_builder_icp_.getCurrentlyBuiltMetricMap()->getAs3DObject( obj );
			view->insert(obj);

			// Only the point map:
			opengl::CSetOfObjectsPtr ptsMap = opengl::CSetOfObjects::Create();
			if (map_builder_icp_.getCurrentlyBuiltMetricMap()->m_pointsMaps.size())
			{
				map_builder_icp_.getCurrentlyBuiltMetricMap()->m_pointsMaps[0]->getAs3DObject(ptsMap);
				view_map->insert( ptsMap );
			}
		}

		// Draw the robot path:
		CPose3DPDFPtr posePDF =  map_builder_icp_.getCurrentPoseEstimation();
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
		if (win3d_ptr_)
		{
			opengl::COpenGLScenePtr &ptrScene = win3d_ptr_->get3DSceneAndLock();
			ptrScene = scene;

			win3d_ptr_->unlockAccess3DScene();

			// Move camera:
			win3d_ptr_->setCameraPointingToPoint( curRobotPose.x(),curRobotPose.y(),curRobotPose.z() );

			// Update:
			win3d_ptr_->forceRepaint();

			sleep( SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS );
		}
	}

    est_x_ = map_builder_icp_.getCurrentPoseEstimation()->getMeanVal().x();
    est_y_ = map_builder_icp_.getCurrentPoseEstimation()->getMeanVal().y();
    est_th_ = map_builder_icp_.getCurrentPoseEstimation()->getMeanVal().yaw();

	step_++;

    return false;
}

bool MapBuilder::get_map(CSimpleMap& map) {

    map_builder_icp_.getCurrentlyBuiltMap(map);

    return false;
}

