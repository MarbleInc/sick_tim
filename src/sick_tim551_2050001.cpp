/*
 * Copyright (C) 2013, Osnabrück University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Osnabrück University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: 14.11.2013
 *
 *      Author:
 *         Martin Günther <mguenthe@uos.de>
 *
 */

#include <sick_tim/sick_tim_common_usb.h>
#include <sick_tim/sick_tim_common_tcp.h>
#include <sick_tim/sick_tim_common_mockup.h>
#include <sick_tim/sick_tim551_2050001_parser.h>
#include <mbot_parameters/parameter_source_ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sick_tim551_2050001");
  sick_tim::SickTim5512050001Parameters params;
  ros::NodeHandle nh("~");
  marble::parameters::load(std::make_shared<marble::parameters::ParameterSourceRos>(nh),&params);

  sick_tim::SickTim5512050001Parser* parser = new sick_tim::SickTim5512050001Parser();

  if (params.range_min>0)
  {
    parser->set_range_min(params.range_min);
  }
  if (params.range_max>0)
  {
    parser->set_range_max(params.range_max);
  }
  if (params.time_increment>0)
  {
    parser->set_time_increment(params.time_increment);
  }

  sick_tim::SickTimCommon* s = NULL;

  int result = sick_tim::ExitError;
  while (ros::ok())
  {
    // Atempt to connect/reconnect
    if (params.subscribe_datagram)
      s = new sick_tim::SickTimCommonMockup(parser, params.publish_dependency);
    else if (params.useTCP)
      s = new sick_tim::SickTimCommonTcp(params.hostname, params.port,
        params.timelimit, parser, params.publish_dependency);
    else
      s = new sick_tim::SickTimCommonUsb(parser, params.device_number,
        params.publish_dependency);
    result = s->init();

    while(ros::ok() && (result == sick_tim::ExitSuccess)){
      ros::spinOnce();
      result = s->loopOnce();
    }

    delete s;

    if (result == sick_tim::ExitFatal)
      return result;

    if (ros::ok() && !params.subscribe_datagram && !params.useTCP)
      ros::Duration(1.0).sleep(); // Only attempt USB connections once per second
  }

  delete parser;
  return result;
}
