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
 *      Author: Martin Günther <mguenthe@uos.de>
 *
 */

#ifndef SICK_TIM551_2050001_PARSER_H_
#define SICK_TIM551_2050001_PARSER_H_

#include "abstract_parser.h"
#include <parameters/parameter_declaration.h>
#include <mbot_diagnostics/output_diagnostic.h>

namespace sick_tim
{
struct SickTim5512050001Parameters {
  SickTim5512050001Parameters() {
    MBOT_PARAMETER_REQUIRED(SickTim5512050001Parameters, publish_dependency);
    MBOT_PARAMETER_OPTIONAL(SickTim5512050001Parameters, useTCP);
    MBOT_PARAMETER_OPTIONAL(SickTim5512050001Parameters, subscribe_datagram);
    MBOT_PARAMETER_OPTIONAL(SickTim5512050001Parameters, device_number);
    MBOT_PARAMETER_OPTIONAL(SickTim5512050001Parameters, hostname);
    MBOT_PARAMETER_OPTIONAL(SickTim5512050001Parameters, timelimit);
    MBOT_PARAMETER_OPTIONAL(SickTim5512050001Parameters, port);
    MBOT_PARAMETER_OPTIONAL(SickTim5512050001Parameters, range_max);
    MBOT_PARAMETER_OPTIONAL(SickTim5512050001Parameters, range_min);
    MBOT_PARAMETER_OPTIONAL(SickTim5512050001Parameters, time_increment);
  }

  // Output diagnostic params for the sick scan
  marble::OutputDiagnosticParams publish_dependency;

  // Use TCP flag (Needs to be true when hostname set)
  bool useTCP = false;

  // Subscribe datagram flag
  bool subscribe_datagram = false;

  // Device numbers
  int device_number = 0;

  // Host name and port
  std::string hostname = "hostname";
  std::string port = "2112";

  // Time limit
  int timelimit = 5;

  //Ranges
  double range_max = 0.0;
  double range_min = 0.0;
  double time_increment = 0.0;
};

class SickTim5512050001Parser : public AbstractParser
{
public:
  SickTim5512050001Parser();
  virtual ~SickTim5512050001Parser();

  virtual int parse_datagram(char* datagram, size_t datagram_length, SickTimConfig &config,
                             sensor_msgs::LaserScan &msg);

  void set_range_min(float min);
  void set_range_max(float max);
  void set_time_increment(float time);

private:
  float override_range_min_, override_range_max_;
  float override_time_increment_;
};

} /* namespace sick_tim */
#endif /* SICK_TIM551_2050001_PARSER_H_ */
