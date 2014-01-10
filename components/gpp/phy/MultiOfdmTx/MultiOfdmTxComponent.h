/**
 * \file components/gpp/phy/MultiOfdmTx/MultiOfdmTxComponent.h
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
 * A multichannel OFDM transmitter component
 */

#ifndef PHY_MULTIOFDMTXCOMPONENT_H_
#define PHY_MULTIOFDMTXCOMPONENT_H_

#include "irisapi/PhyComponent.h"
#include "multichanneltx.h"

namespace iris
{
namespace phy
{

/** A multichannel OFDM transmitter component.
 *
 * This component uses the multichanneltx class from liquid-usrp
 */
class MultiOfdmTxComponent
  : public PhyComponent
{
 public:
  MultiOfdmTxComponent(std::string name);
  ~MultiOfdmTxComponent();
  virtual void calculateOutputTypes(
      std::map<std::string, int>& inputTypes,
      std::map<std::string, int>& outputTypes);
  virtual void registerPorts();
  virtual void initialize();
  virtual void process();

 private:
  bool addFrame(int ch, int size, uint8_t* payload, double ts);

  uint16_t localAddress_x;   ///< Address of this client (used to drop own frames)
  int nChans_x;
  int nCarriers_x;
  int cpLen_x;
  int taperLen_x;
  bool guardChannels_x;
  uint32_t mask_x;
  std::string modulation_x;

  modulation_scheme mod_;
  multichanneltx* mctx_;
  uint8_t header_[8];
  int curChan_;
  int startChan_;
  int stopChan_;
};

} // namespace phy
} // namespace iris

#endif // PHY_MULTIOFDMTXCOMPONENT_H_
