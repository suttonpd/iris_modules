/**
 * \file components/gpp/stack/PacketGenerator/PacketGeneratorComponent.cpp
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
 * Implementation of a simple packet generator with a graphical interface.
 */

#include "PacketGeneratorComponent.h"
#include "irisapi/LibraryDefs.h"
#include "irisapi/Version.h"
#include "PacketGeneratorGui/PacketGeneratorGui.h"

using namespace std;

namespace iris
{
namespace stack
{

//! Export library symbols
IRIS_COMPONENT_EXPORTS(StackComponent, PacketGeneratorComponent);

PacketGeneratorComponent::PacketGeneratorComponent(std::string name)
  : StackComponent(name,                          // Component name
                   "packetgenerator",             // Component type
                   "A simple packet generator",   // Description
                   "Paul Sutton",                 // Author
                   "0.1")                         // Version
{
}

void PacketGeneratorComponent::initialize()
{
}

void PacketGeneratorComponent::start()
{
  gui_ = new PacketGeneratorGui;
  thread_.reset(new boost::thread(
                  boost::bind( &PacketGeneratorComponent::threadFunction, this))
                );
}

void PacketGeneratorComponent::stop()
{
  if(gui_)
    delete gui_;
}

void PacketGeneratorComponent::threadFunction()
{
  int length = 0;
  boost::this_thread::sleep(boost::posix_time::seconds(1));
  try
  {
    while(true)
    {
      boost::this_thread::interruption_point();
      gui_->waitForPacket(length);
      sendPacket(length);
    }
  }
  catch(IrisException& ex)
  {
    LOG(LFATAL) << "Error in PacketGenerator component: " << ex.what() << " - thread exiting.";
  }
  catch(boost::thread_interrupted)
  {
    LOG(LINFO) << "Thread " << boost::this_thread::get_id() << " in PacketGenerator interrupted.";
  }
}

void PacketGeneratorComponent::sendPacket(int length)
{
  boost::shared_ptr<StackDataSet> packet(new StackDataSet);
  uint8_t count = 0;
  for(int i=0;i<length;i++)
    packet->data.push_back(count++);
  sendDownwards(packet);
}

void PacketGeneratorComponent::processMessageFromAbove(boost::shared_ptr<StackDataSet> set)
{
  //Shouldn't get messages from above
}

void PacketGeneratorComponent::processMessageFromBelow(boost::shared_ptr<StackDataSet> set)
{
  //Check data
  bool valid = true;
  uint8_t count = 0;
  for(int i=0;i<set->data.size();i++)
    if(set->data[i] != count++)
      valid = false;

  if(valid)
    LOG(LINFO) << "Received valid packet";
  else
    LOG(LINFO) << "Received invalid packet";
}

} // namespace stack
} // namespace iris
