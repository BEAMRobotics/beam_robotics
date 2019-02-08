# novatel_span_driver

[![Build Status](http://jenkins.ros.org/buildStatus/icon?job=devel-indigo-novatel_span_driver)](http://jenkins.ros.org/job/devel-indigo-novatel_span_driver/)

This ROS package connects via Ethernet to a [NovAtel](http://www.novatel.com/) receiver running
[SPAN](http://www.novatel.com/span).

Please see the ROS Wiki for details: http://wiki.ros.org/novatel_span_driver

As of November 20th, 2017 this version of the driver differs from the one linking on the wiki in that:
- The NavSatFix message information is populated from INSPVAX instead of BESTPOS
- A conversion from GPS time to UTC done to populate the timestamps in the ROS messages instead of using ros time now
