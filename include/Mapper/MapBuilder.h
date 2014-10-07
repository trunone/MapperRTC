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
    static MapBuilder* get_instance() { return unique_instance_; }
    ~MapBuilder();

    bool InitMapping();
    bool StopMapping();
    bool StartMapping(CActionCollection, CSensoryFrame);

    double get_est_x() { return est_x_; }
    double get_est_y() { return est_y_; }
    double get_est_th() { return est_th_; }

    bool get_map(CSimpleMap&);

private:
    static MapBuilder* unique_instance_;

	int step_;

	// ICP-SLAM object:
	CMetricMapBuilderICP map_builder_icp_;

	// Create 3D window if requested:
	CDisplayWindow3DPtr win3d_ptr_;

    double est_x_, est_y_, est_th_;

};
