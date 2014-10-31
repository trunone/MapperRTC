MapperRTC
=================

Layout
----

![MapperRTC](https://farm4.staticflickr.com/3935/15478441318_f66a904f63_o.png)

Features
----
  * Receive range data and Odometer to build the map of the indoor environment
  * Estimate the position of the robot
  * Output the map in string format

Requirements
----
  * OS
   * Linux Distributions
   * Windows
  * Softwares
   * OpenRTM-aist C++ 1.1 or later
   * CMake 2.8

Port
----

| Name     | Type          | Data Type   | Purpose |
| -------- | ------------- | ----------- | ------- |
| RangeData   | In       | RTC::RangeData | The range data from Laser Range Finder|
| Odemetry  | Out      | RTC::TimedPose2D  | The pose data from odemetry |
| estPose     | Out      | RTC::TimedPose2D   | The estimate pose |
| map      | Service | MapService | Send current map |

  * MapService.idl

| Function | Variable | Type | Data Type   |
| -------- | -------- | ---- | ----------- |
| getMap   | arg      | out  | string      |

Usage
----

Download and install [Mobile Robot Programming Toolkit](http://www.mrpt.org/)

License
----

Licensed under the [Apache License, Version 2.0][Apache]  
Distributed under the [MIT License][mit].  
Dual licensed under the [MIT license][MIT] and [Apache License, Version 2.0][Apache].  
 
[Apache]: http://www.apache.org/licenses/LICENSE-2.0
[MIT]: http://www.opensource.org/licenses/mit-license.php
