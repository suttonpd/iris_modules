/**
 * \file components/gpp/phy/MultiOfdmRx/MultiOfdmRxComponent.h
 * \version 1.0
 *
 * \section COPYRIGHT
 *
 * Copyright 2012-2013 The Iris Project Developers. See the
 * COPYRIGHT file at the top-level directory of this distribution
 * and at http://www.softwareradiosystems.com/iris/copyright.html.
 *
 * \section LICENSE
 *
 * This file is part of the Iris Project.
 *
 * Iris is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * 
 * Iris is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * A copy of the GNU Lesser General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 * \section DESCRIPTION
 *
 * A multichannel OFDM receiver component.
 */

#ifndef PHY_MULTIOFDMRXCOMPONENT_H_
#define PHY_MULTIOFDMRXCOMPONENT_H_

#include "irisapi/PhyComponent.h"
#include "multichannelrx.h"

namespace iris
{
namespace phy
{

/** A multichannel OFDM receiver component.
 *
 * This component uses the multichannelrx class from liquid-usrp.
 */
class MultiOfdmRxComponent
  : public PhyComponent
{
 public:
  MultiOfdmRxComponent(std::string name);
  ~MultiOfdmRxComponent();
  virtual void calculateOutputTypes(
      std::map<std::string, int>& inputTypes,
      std::map<std::string, int>& outputTypes);
  virtual void registerPorts();
  virtual void initialize();
  virtual void process();
  void callback( unsigned char *  _header,
                 int              _header_valid,
                 unsigned char *  _payload,
                 unsigned int     _payload_len,
                 int              _payload_valid,
                 framesyncstats_s _stats,
                 void *           _userdata);

 private:
  uint16_t localAddress_x;   ///< Address of this client (used to drop own frames)
  int nChans_x;
  int nCarriers_x;
  int cpLen_x;
  int taperLen_x;

  double curTimestamp;
  double curSampleRate;

  multichannelrx* mcrx_;
  std::vector<unsigned int> channelIds_;
  struct ChannelStat{
    long nRx;
    double aveEvm;
    double devEvm;
    ChannelStat():nRx(0),aveEvm(0),devEvm(0){}
  };
  std::vector<ChannelStat> channelStats_;
};

namespace multiofdmrxdetail
{
static MultiOfdmRxComponent* gPtr;

int gCallback( unsigned char *  _header,
               int              _header_valid,
               unsigned char *  _payload,
               unsigned int     _payload_len,
               int              _payload_valid,
               framesyncstats_s _stats,
               void *           _userdata);
} // namespace multiofdmrxdetail

} // namespace phy
} // namespace iris

#endif // PHY_MULTIOFDMRXCOMPONENT_H_
