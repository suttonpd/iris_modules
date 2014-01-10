/**
 * \file components/gpp/phy/MultiOfdmRx/MultiOfdmRxComponent.cpp
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
 * Implementation of a multichannel OFDM receiver component.
 */

#include "MultiOfdmRxComponent.h"

#include "irisapi/LibraryDefs.h"
#include "irisapi/Version.h"

using namespace std;

namespace iris
{
namespace phy
{

namespace multiofdmrxdetail
{
int gCallback( unsigned char *  _header,
               int              _header_valid,
               unsigned char *  _payload,
               unsigned int     _payload_len,
               int              _payload_valid,
               framesyncstats_s _stats,
               void *           _userdata)
{
  gPtr->callback(_header,
                 _header_valid,
                 _payload,
                 _payload_len,
                 _payload_valid,
                 _stats,
                 _userdata);
}
} // namespace multiofdmrxdetail


// export library symbols
IRIS_COMPONENT_EXPORTS(PhyComponent, MultiOfdmRxComponent);

MultiOfdmRxComponent::MultiOfdmRxComponent(std::string name)
  : PhyComponent(name,
                "MultiOfdmRx",
                "An example phy component",
                "Paul Sutton",
                "0.1")
  ,mcrx_(NULL)
{
  multiofdmrxdetail::gPtr = this;
  registerParameter("localaddress", "Address of this client", "65535",
      false, localAddress_x);
  registerParameter("nchans", "Number of channels", "1",
      false, nChans_x, Interval<int>(1,64));
  registerParameter("ncarriers", "Number of subcarriers", "64",
      false, nCarriers_x);
  registerParameter("cplen", "Cyclic prefix length", "16",
      false, cpLen_x);
  registerParameter("taperLen", "Shaping taper length", "4",
      false, taperLen_x);
}

MultiOfdmRxComponent::~MultiOfdmRxComponent()
{
  if(mcrx_)
    delete mcrx_;
  for(int i=0;i<channelStats_.size();i++)
  {
    LOG(LINFO) << "Channel " << i
               << " nRx: " << channelStats_[i].nRx
               << " aveEvm: " << channelStats_[i].aveEvm;
  }
}

void MultiOfdmRxComponent::registerPorts()
{
  registerInputPort("input1", TypeInfo< complex<float> >::identifier);
  registerOutputPort("output1", TypeInfo< uint8_t >::identifier);
}

void MultiOfdmRxComponent::calculateOutputTypes(
    std::map<std::string,int>& inputTypes,
    std::map<std::string,int>& outputTypes)
{
  outputTypes["output1"] = TypeInfo< uint8_t >::identifier;
}

void MultiOfdmRxComponent::initialize()
{
  channelIds_.resize(nChans_x);
  channelStats_.resize(nChans_x);
  void* userdata[nChans_x];
  framesync_callback callbacks[nChans_x];
  for(unsigned int i=0; i<nChans_x; i++) {
    channelIds_[i] = i;
    userdata[i] = &channelIds_[i];
    callbacks[i] = multiofdmrxdetail::gCallback;
  }
  unsigned char * p = NULL;   // default subcarrier allocation
  mcrx_ = new multichannelrx(nChans_x, nCarriers_x, cpLen_x, taperLen_x,
                             p, userdata, callbacks);
}

void MultiOfdmRxComponent::process()
{
  //Get a DataSet from the input DataBuffer
  DataSet< complex<float> >* readDataSet = NULL;
  getInputDataSet("input1", readDataSet);
  std::size_t size = readDataSet->data.size();
  complex<float>* data = &readDataSet->data[0];

  //Copy the timestamp and sample rate for the DataSets
  curTimestamp  = readDataSet->timeStamp;
  curSampleRate = readDataSet->sampleRate;

  mcrx_->Execute(data, size);

  //Release the DataSets
  releaseInputDataSet("input1", readDataSet);

}

void MultiOfdmRxComponent::callback( unsigned char *  _header,
               int              _header_valid,
               unsigned char *  _payload,
               unsigned int     _payload_len,
               int              _payload_valid,
               framesyncstats_s _stats,
               void *           _userdata)
{
  unsigned int len = (3+_stats.num_framesyms)*(nCarriers_x+cpLen_x+taperLen_x);
  double rateFactor = nChans_x;
  double c = mcrx_->GetSampleCount()/rateFactor - len;

  if(_header_valid)
  {
    uint16_t address = (_header[0] << 8 | _header[1]);
    if(address == localAddress_x)
    {
      LOG(LDEBUG) << "Dropping own transmission";
      return;
    }
    else
    {
      unsigned int channel = *(unsigned int*)_userdata;
      LOG(LDEBUG) << "Frame received on channel " << channel
                  << ". RSSI: " << _stats.rssi
                  << ". EVM: " << _stats.evm << ".";
      if(channel < nChans_x)
      {
        double e = channelStats_[channel].aveEvm;
        e*=channelStats_[channel].nRx++;
        e/=channelStats_[channel].nRx;
        e+=_stats.evm / channelStats_[channel].nRx;
        channelStats_[channel].aveEvm = e;
      }
      //unsigned int channel = _header[2];
      if(_payload_valid)
      {
        DataSet<uint8_t>* w = NULL;
        getOutputDataSet("output1", w, _payload_len);
        w->sampleRate = curSampleRate/rateFactor;
        w->timeStamp = curTimestamp+(c*(1/w->sampleRate));
        copy(_payload, &_payload[_payload_len], w->data.begin());
        releaseOutputDataSet("output1", w);
      } else {
        LOG(LWARNING) << "Payload invalid";
      }
    }
  } else {
    LOG(LWARNING) << "Header invalid";
  }
}

} // namesapce phy
} // namespace iris
