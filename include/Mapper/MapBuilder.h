#include <mrpt/base.h>
#include <mrpt/utils.h>
#include <mrpt/obs.h>
#include <mrpt/slam.h>
#include <mrpt/gui.h>

using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::system;
using namespace mrpt::math;

class MapBuilder
{
public:
    MapBuilder();
    static MapBuilder* get_instance() { return unique_instance; }
    ~MapBuilder();

    bool InitMapping();
    bool StopMapping();
    bool StartMapping(CActionCollection, CSensoryFrame);

    double get_est_x() { return est_x; }
    double get_est_y() { return est_y; }
    double get_est_th() { return est_th; }

    bool get_map(CSimpleMap&);

private:
    static MapBuilder* unique_instance;

    // Output rawlog:
	CFileGZOutputStream  outputFile;

	// Building map:
	const char* OUT_DIR;

	// config file
	// ----------------------------------
	unsigned int rawlog_offset;
	string OUT_DIR_STD;
	int LOG_FREQUENCY;
	bool  SAVE_POSE_LOG;
	bool  SAVE_3D_SCENE;
	bool  CAMERA_3DSCENE_FOLLOWS_ROBOT;

	bool 	SHOW_PROGRESS_3D_REAL_TIME;
	int		SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS;

	// log files:
	// ----------------------------------
	CFileOutputStream  f_log;
	CFileOutputStream  f_path;
	CFileOutputStream  f_pathOdo;

	// Checks:
	int	step;
	string								str;
	CSimpleMap							finalMap;
	float								t_exec;
	COccupancyGridMap2D::TEntropyInfo	entropy;

	// ICP-SLAM object:
	CMetricMapBuilderICP map_builder_icp;

	// Create 3D window if requested:
	CDisplayWindow3DPtr	win3D;

    double est_x, est_y, est_th;

};
