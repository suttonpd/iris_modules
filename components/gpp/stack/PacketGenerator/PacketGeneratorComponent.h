/**
 * \file components/gpp/stack/PacketGenerator/PacketGeneratorComponent.h
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
 * A simple packet generator with a graphical interface.
 */

#ifndef STACK_PACKETGENERATORCOMPONENT_H_
#define STACK_PACKETGENERATORCOMPONENT_H_

#include "irisapi/StackComponent.h"

class PacketGeneratorGui;

namespace iris
{
namespace stack
{

/** A simple packet generator with a graphical interface.
 *
 * Provides an interface to generate individual packets or sequences of
 * packets of a particular length.
 */
class PacketGeneratorComponent
  : public StackComponent
{
public:
  PacketGeneratorComponent(std::string name);
  virtual void initialize();
  virtual void start();
  virtual void stop();
  virtual void processMessageFromAbove(boost::shared_ptr<StackDataSet> set);
	virtual void processMessageFromBelow(boost::shared_ptr<StackDataSet> set);

private:
  PacketGeneratorGui* gui_;
  boost::scoped_ptr< boost::thread > thread_;

  void threadFunction();
  void sendPacket(int length);
};

} // namespace stack
} // namespace iris

#endif // STACK_PACKETGENERATORCOMPONENT_H_
