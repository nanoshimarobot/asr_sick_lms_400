/*
 * Copyright (c) 2009 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
 * Code based on the LGPL Player SICK LMS400 driver by Nico Blodow and Radu
 * Bogdan Rusu
 *
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
 * $Id$
 *
 */

#include <netdb.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <sys/fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#define BUF_SIZE 1024

namespace asr_sick_lms_400
{
////////////////////////////////////////////////////////////////////////////////
typedef struct
{
  unsigned char * string;
  int length;
} MeasurementQueueElement_t;

////////////////////////////////////////////////////////////////////////////////
typedef struct
{
  uint16_t Format;
  uint16_t DistanceScaling;
  int32_t StartingAngle;
  uint16_t AngularStepWidth;
  uint16_t NumberMeasuredValues;
  uint16_t ScanningFrequency;
  uint16_t RemissionScaling;
  uint16_t RemissionStartValue;
  uint16_t RemissionEndValue;
} MeasurementHeader_t;

////////////////////////////////////////////////////////////////////////////////
class asr_sick_lms_400
{
public:
  asr_sick_lms_400() {}
  asr_sick_lms_400(rclcpp::Node * node, const char * host, int port, int debug_mode);

  // Creates socket, connects
  int Connect();
  int Disconnect();

  // Configuration parameters
  int SetAngularResolution(
    const char * password, float ang_res, float angle_start, float angle_range);
  int SetScanningFrequency(const char * password, float freq, float angle_start, float angle_range);
  int SetResolutionAndFrequency(float freq, float ang_res, float angle_start, float angle_range);

  int StartMeasurement(bool intensity = true);
  sensor_msgs::msg::LaserScan ReadMeasurement();
  int StopMeasurement();

  int SetUserLevel(int8_t userlevel, const char * password);
  int GetMACAddress(char ** macadress);

  int SetIP(char * ip);
  int SetGateway(char * gw);
  int SetNetmask(char * mask);
  int SetPort(uint16_t port);

  int ResetDevice();
  int TerminateConfiguration();

  int SendCommand(const char * cmd);
  int ReadResult();
  // for "Variables", Commands that only reply with one Answer message
  int ReadAnswer();
  // for "Procedures", Commands that reply with a Confirmation message and an
  // Answer message
  int ReadConfirmationAndAnswer();

  int EnableRIS(int onoff);
  int SetMeanFilterParameters(int num_scans);
  int SetRangeFilterParameters(float range_min, float range_max);
  int EnableFilters(int filter_mask);

  // turns a string holding an ip address into long
  unsigned char * ParseIP(char * ip);

private:
  // node ptr for log
  const rclcpp::Node * node_;

  // assembles STX's, length field, message, checksum ready to be sent. Cool.
  int AssembleCommand(unsigned char * command, int len);

  const char * hostname_;
  int sockfd_, portno_, n_;
  struct sockaddr_in serv_addr_;
#if HAVE_GETADDRINFO
  struct addrinfo * addr_ptr_;
#else
  struct hostent * server_;
#endif

  // Internal Parameters:
  int verbose_;
  int ExtendedRIS_;
  int MeanFilterNumScans_;
  float RangeFilterTopLimit_;
  float RangeFilterBottomLimit_;
  int FilterMask_;

  long int scanning_frequency_, resolution_;

  // for reading:
  unsigned char buffer_[4096];
  unsigned int bufferlength_;

  // for sending:
  unsigned char command_[BUF_SIZE];
  int commandlength_;
  std::vector<MeasurementQueueElement_t> * MeasurementQueue_;
};
}  // namespace asr_sick_lms_400
