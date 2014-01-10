/**
 * \file components/gpp/phy/MultiOfdmTx/MultiOfdmTxComponent.cpp
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
 * Implementation of a multichannel OFDM transmitter component.
 */

#include "MultiOfdmTxComponent.h"

#include "irisapi/LibraryDefs.h"
#include "irisapi/Version.h"

#include "utility/RawFileUtility.h"

using namespace std;

namespace iris
{
namespace phy
{

// export library symbols
IRIS_COMPONENT_EXPORTS(PhyComponent, MultiOfdmTxComponent);

MultiOfdmTxComponent::MultiOfdmTxComponent(std::string name)
  : PhyComponent(name,                              // component name
                "multiofdmtx",                      // component type
                "A multichannel OFDM transmitter",  // description
                "Paul Sutton",                      // author
                "0.1")                              // version
  ,mctx_(NULL)
  ,curChan_(0)
{
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
  registerParameter("guardchannels", "Null outside channels?", "false",
      true, guardChannels_x);
  registerParameter("mask", "Active channel mask", "4294967295",
      true, mask_x);
  registerParameter("modulation", "Modulation order (qpsk, qam16)", "qpsk",
      true, modulation_x);
}

MultiOfdmTxComponent::~MultiOfdmTxComponent()
{
  if(mctx_)
    delete mctx_;
}

void MultiOfdmTxComponent::registerPorts()
{
  registerInputPort("input1", TypeInfo< uint8_t >::identifier);
  registerOutputPort("output1", TypeInfo< complex<float> >::identifier);
}

void MultiOfdmTxComponent::calculateOutputTypes(
    std::map<std::string,int>& inputTypes,
    std::map<std::string,int>& outputTypes)
{
  outputTypes["output1"] = TypeInfo< complex<float> >::identifier;
}

void MultiOfdmTxComponent::initialize()
{
  unsigned char * p = NULL; //Default subcarrier allocation
  mctx_ = new multichanneltx(nChans_x, nCarriers_x, cpLen_x, taperLen_x, p);
  if(guardChannels_x) curChan_ = 1;
  if(modulation_x == "qpsk"){mod_ = LIQUID_MODEM_QPSK;}
  if( modulation_x == "qam16"){mod_ = LIQUID_MODEM_QAM16;}
}

void MultiOfdmTxComponent::process()
{
  //Get a DataSet from the input DataBuffer
  DataSet<uint8_t>* readDataSet = NULL;
  getInputDataSet("input1", readDataSet);
  std::size_t size = readDataSet->data.size();
  uint8_t* payload = &readDataSet->data[0];

  startChan_ = 0;
  stopChan_ = nChans_x;
  if(guardChannels_x)
  {
    startChan_ = 1;
    stopChan_ = nChans_x - 1;
  }

  if(!readDataSet->isControl) // only send on 1 active channel
  {
    bool sent = false;
    do{
      sent = addFrame(curChan_++, size, payload, readDataSet->timeStamp);
      if(curChan_ == stopChan_)
        curChan_ = startChan_;
    }
    while(!sent);
  }
  else // send on all active channels
  {
    for(int i=startChan_;i<stopChan_;i++)
    {
      addFrame(i, size, payload, readDataSet->timeStamp);
    }
  }

  releaseInputDataSet("input1", readDataSet);
}

bool MultiOfdmTxComponent::addFrame(int ch, int size, uint8_t* payload, double ts)
{
  if(!((mask_x >> ch) & 0x1))
    return false;
  if(mctx_->IsChannelReadyForData(ch))
  {
    // write header (first two bytes local address, then channel id, then random)
    header_[0] = (localAddress_x >> 8) & 0xff;
    header_[1] = (localAddress_x     ) & 0xff;
    header_[2] = ch & 0xff;
    for(int i=3; i<8; i++)
      header_[i] = rand() & 0xff;

    mctx_->UpdateData(ch, header_, payload, size,
                     mod_, LIQUID_FEC_NONE, LIQUID_FEC_HAMMING128);

    if(++ch == stopChan_)
    {
      unsigned int len = mctx_->GetFrameSize();

      //Get a DataSet from the output DataBuffer
      DataSet< complex<float> >* writeDataSet = NULL;
      getOutputDataSet("output1", writeDataSet, len);
      complex<float>* buf = &writeDataSet->data[0];

      LOG(LINFO) << "Writing frame of length " << len;

      for(int i=0;i<len;i+=nChans_x)
        mctx_->GenerateSamples(&buf[i]);

      //Set timestamp to that of the last packet added
      writeDataSet->timeStamp = ts;

      //RawFileUtility::write(buf, &buf[len], "multiofdmframe");
      releaseOutputDataSet("output1", writeDataSet);
    }
  }
  return true;
}

} // namesapce phy
} // namespace iris
