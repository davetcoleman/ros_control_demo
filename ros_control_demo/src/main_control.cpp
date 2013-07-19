/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 *  \author Dave Coleman
 *  \desc   Main for testing ros_control
 */

// ROS
//#include <ros/ros.h>
//#include <control_toolbox/pid.h>
//#include <realtime_tools/realtime_buffer.h>
#include <transmission_interface/transmission_info.h>

namespace ros_control_demo
{

} // namespace

/**
 * \brief Executable main
 * \return error code
 */
int main(int argc, char **argv)
{
  // Initialize node
  //ros::init(argc, argv, "ros_control_demo", ros::init_options::AnonymousName);
  //ros::AsyncSpinner spinner(1);
  //spinner.start();

  // Test Transmission Info dependencies -----------------------------------------------
  transmission_interface::ActuatorInfo tran;
  tran.xml_element_;

  /*
  // Initialize PID copying ------------------------------------------------------------
  control_toolbox::Pid pid1(100,0.1,10,0,0);
  control_toolbox::Pid pid2 = pid1;  
  
  pid1.printValues();
  //  pid2.printValues();
  */

  /*
  // Test the realtime buffer const stuff ------------------------------------------------
  realtime_tools::RealtimeBuffer<double> buffer; 
  buffer.writeFromNonRT(4.0);

  const realtime_tools::RealtimeBuffer<double> *buffer_const;
  buffer_const = &buffer; 

  buffer_const->readFromRT();
  */

  return 0;
}
