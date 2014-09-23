// -*- C++ -*-
/*!
 * @file  Mapper.cpp
 * @brief ModuleDescription
 * @date $Date$
 *
 * $Id$
 */

#include "Mapper.h"

// Module specification
// <rtc-template block="module_spec">
static const char* mapper_spec[] =
  {
    "implementation_id", "Mapper",
    "type_name",         "Mapper",
    "description",       "ModuleDescription",
    "version",           "1.0.0",
    "vendor",            "Intelligent Systems Lab.",
    "category",          "SLAM",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.Direction", "1",
    "conf.default.Aperture", "240.0",
    // Widget
    "conf.__widget__.Direction", "text",
    "conf.__widget__.Aperture", "text",
    // Constraints
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
Mapper::Mapper(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_OdometryIn("odometry", m_Odometry),
    m_RangeDataIn("rangeData", m_RangeData),
    m_EstPoseOut("estPose", m_EstPose),
    m_ProviderPortPort("ProviderPort")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
Mapper::~Mapper()
{
}



RTC::ReturnCode_t Mapper::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("odometry", m_OdometryIn);
  addInPort("rangeData", m_RangeDataIn);
  
  // Set OutPort buffer
  addOutPort("estPose", m_EstPoseOut);
  
  // Set service provider to Ports
  m_ProviderPortPort.registerProvider("simpleMap", "Map::SimpleMap", m_simpleMap);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_ProviderPortPort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("Direction", m_Direction, "1");
  bindParameter("Aperture", m_Aperture, "240.0");
  // </rtc-template>

    now_x = 0.0; now_y = 0.0; now_theta = 0.0;
	past_x = 0.0; past_y = 0.0; past_theta = 0.0;

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Mapper::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Mapper::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Mapper::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t Mapper::onActivated(RTC::UniqueId ec_id)
{
    MapBuilder::get_instance()->InitMapping();
	firstloop = true;

  return RTC::RTC_OK;
}


RTC::ReturnCode_t Mapper::onDeactivated(RTC::UniqueId ec_id)
{
    MapBuilder::get_instance()->StopMapping();

  return RTC::RTC_OK;
}


RTC::ReturnCode_t Mapper::onExecute(RTC::UniqueId ec_id)
{
	//Sleep(300);

	// ------------------------------------------------------------------------
	//                           Rawlog
	// ------------------------------------------------------------------------
	CActionCollection action_collection;
	CSensoryFrame  sensory_frame;

	// Odometry
	if(m_OdometryIn.isNew()){
		m_OdometryIn.read();
		now_x = m_Odometry.data.position.x;
		now_y = m_Odometry.data.position.y;
		now_theta = m_Odometry.data.heading;

		if(firstloop == true){
			past_x = now_x; past_y = now_y; past_theta = now_theta;
			firstloop = false;
		}

		double x = (now_x-past_x)*cos(-past_theta)-(now_y-past_y)*sin(-past_theta);
		double y = (now_x-past_x)*sin(-past_theta)+(now_y-past_y)*cos(-past_theta);
		double theta = now_theta-past_theta;

		CPose2D pos(x, y, theta);
		CActionRobotMovement2D movement2d;
		CActionRobotMovement2D::TMotionModelOptions	options;
		movement2d.computeFromOdometry(pos, options);
		mrpt::system::TTimeStamp AtD0 = mrpt::system::getCurrentTime();
		movement2d.timestamp = AtD0;
		action_collection.insert(movement2d);
	}

	// LRF
	if(m_RangeDataIn.isNew()){
		m_RangeDataIn.read();
		CObservation2DRangeScanPtr obs2d = CObservation2DRangeScan::Create();
		int SCANS_SIZE = m_RangeData.ranges.length();
		obs2d->aperture = m_Aperture * M_PIf / 180.0;
		if(m_Direction == 1)
			obs2d->rightToLeft = true;
		else
			obs2d->rightToLeft = false;
		//obs2d->maxRange = 40;
		obs2d->validRange.resize(SCANS_SIZE);
		obs2d->scan.resize(SCANS_SIZE);
		mrpt::system::TTimeStamp AtD1	=  mrpt::system::getCurrentTime();
		obs2d->timestamp = AtD1;
		obs2d->scan.resize(SCANS_SIZE);
		for(int i=0; i<SCANS_SIZE; i++) {
			obs2d->scan[i] = (float)m_RangeData.ranges[i];
			obs2d->validRange[i] = 1;
		}
		sensory_frame.insert(obs2d);  // memory of "obs2d" will be automatically freed.
	}

	past_x = now_x;
	past_y = now_y;
	past_theta = now_theta;

    MapBuilder::get_instance()->StartMapping(action_collection, sensory_frame);
  
    m_EstPose.data.position.x = MapBuilder::get_instance()->get_est_x();
    m_EstPose.data.position.y = MapBuilder::get_instance()->get_est_y();
    m_EstPose.data.heading    = MapBuilder::get_instance()->get_est_th();
    //m_debug ? printf("Current Position: (x, y, th) = (%f, %f, %f)",
    //               x, y, th) : true;
    m_EstPoseOut.write();

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Mapper::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Mapper::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Mapper::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Mapper::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Mapper::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void MapperInit(RTC::Manager* manager)
  {
    coil::Properties profile(mapper_spec);
    manager->registerFactory(profile,
                             RTC::Create<Mapper>,
                             RTC::Delete<Mapper>);
  }
  
};


