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
    "conf.default.Aperture", "180.0",
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
    m_mapPort("map")

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
  m_mapPort.registerProvider("simpleMap", "Map::SimpleMap", m_simpleMap);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_mapPort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("Direction", m_Direction, "1");
  bindParameter("Aperture", m_Aperture, "240.0");
  // </rtc-template>
    
    past_pose_.position.x = 0;
    past_pose_.position.y = 0;;
    past_pose_.heading = 0;

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
	first_run_ = true;

  return RTC::RTC_OK;
}


RTC::ReturnCode_t Mapper::onDeactivated(RTC::UniqueId ec_id)
{
    MapBuilder::get_instance()->StopMapping();

  return RTC::RTC_OK;
}


RTC::ReturnCode_t Mapper::onExecute(RTC::UniqueId ec_id)
{
	CActionCollection action_collection;
	CSensoryFrame  sensory_frame;

	// Odometry
	if(m_OdometryIn.isNew()){
		m_OdometryIn.read();

		if(first_run_ == true){
            past_pose_ = m_Odometry.data;
			first_run_ = false;
		}

		double x = (m_Odometry.data.position.x-past_pose_.position.x)*cos(-past_pose_.heading)
            -(m_Odometry.data.position.y-past_pose_.position.y)*sin(-past_pose_.heading);
		double y = (m_Odometry.data.position.x-past_pose_.position.x)*sin(-past_pose_.heading)
            +(m_Odometry.data.position.y-past_pose_.position.y)*cos(-past_pose_.heading);
		double theta = m_Odometry.data.heading-past_pose_.heading;

		CPose2D pos(x, y, theta);
		CActionRobotMovement2D movement2d;
		CActionRobotMovement2D::TMotionModelOptions options;
		movement2d.computeFromOdometry(pos, options);
        movement2d.timestamp = mrpt::system::getCurrentTime();
		action_collection.insert(movement2d);
	}

	// LRF
	if(m_RangeDataIn.isNew()){
		m_RangeDataIn.read();
		CObservation2DRangeScanPtr obs2d = CObservation2DRangeScan::Create();
		obs2d->aperture = m_Aperture * M_PIf / 180.0;
		if(m_Direction == 1)
			obs2d->rightToLeft = true;
		else
			obs2d->rightToLeft = false;
		//obs2d->maxRange = 40;
		obs2d->validRange.resize(m_RangeData.ranges.length());
		obs2d->scan.resize(m_RangeData.ranges.length());
		obs2d->timestamp = mrpt::system::getCurrentTime();
		obs2d->scan.resize(m_RangeData.ranges.length());
		for(unsigned int i=0; i<m_RangeData.ranges.length(); i++) {
			obs2d->scan[i] = (float)m_RangeData.ranges[i];
			obs2d->validRange[i] = 1;
		}
		sensory_frame.insert(obs2d);
	}

    past_pose_ = m_Odometry.data;

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


