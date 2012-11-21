/**
 * \file UsrpUhdTxComponent.h
 * \version 1.0
 *
 * \section COPYRIGHT
 *
 * Copyright 2012 The Iris Project Developers. See the
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
 * A sink component which writes to a USRP transmitter using the
 * Universal Hardware Driver (UHD).
 * This component supports streaming data by default. For bursty data,
 * set "streaming" to false. For bursty data, packets must be contained
 * in a single DataSet. If a timestamp is specified in a DataSet, the
 * packet will be transmitted at that time, if supported by the USRP.
 */

#ifndef PN_USRPUHDTXCOMPONENT_H_
#define PN_USRPUHDTXCOMPONENT_H_

#include "irisapi/PNComponent.h"
#include <uhd/usrp/multi_usrp.hpp>

namespace iris
{

class UsrpUhdTxComponent: public PNComponent
{
public:
  UsrpUhdTxComponent(std::string name);
  virtual ~UsrpUhdTxComponent();
  virtual std::map<std::string, int> calculateOutputTypes(std::map<std::string, int> inputTypes);
  virtual void registerPorts();
  virtual void initialize();
  virtual void process();
  virtual void parameterHasChanged(std::string name);

private:
  ReadBuffer< std::complex<float> >* inBuf_;

  //Exposed parameters
  std::string args_x;   //!< See http://www.ettus.com/uhd_docs/manual/html/identification.html
  double rate_x;        //!< Rate of outgoing samples
  double frequency_x;   //!< Tx frequency
  double fixLoOffset_x; //!< Fix the local oscillator offset (defaults to 2*rate)
  float gain_x;         //!< Overall tx gain
  std::string antenna_x;//!< Daughterboard antenna selection
  std::string subDev_x; //!< Daughterboard subdevice specification
  double bw_x;          //!< Daughterboard IF filter bandwidth (Hz)
  std::string ref_x;    //!< Reference waveform (internal, external, mimo)
  bool streaming_x;     //!< Streaming or bursty traffic?
  std::string fmt_x;    //!< Data format (fc64, fc32 or sc16)

  //The device
  uhd::usrp::multi_usrp::sptr usrp_;
  uhd::tx_streamer::sptr txStream_;
};

} // namespace iris

#endif // PN_USRPUHDTXCOMPONENT_H_
