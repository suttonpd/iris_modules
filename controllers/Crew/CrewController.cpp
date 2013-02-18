/**
 * \file controllers/Crew/CrewController.cpp
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
 * Implementation of the simple Crew controller.
 */

#include <sstream>

#include "irisapi/LibraryDefs.h"
#include "irisapi/Version.h"
#include "CrewController.h"
#include <boost/bind.hpp>

using namespace std;

namespace iris
{

//! Export library functions
IRIS_CONTROLLER_EXPORTS(CrewController);

CrewController::CrewController()
  : Controller("Crew", "A simple demonstration controller", "Paul Sutton", "0.1")
{
}

void CrewController::subscribeToEvents()
{
  // Format: subscribeToEvent(Event name, Component name);
}

void CrewController::initialize()
{
  crewThread_.reset( new boost::thread( boost::bind( &CrewController::threadLoop, this ) ) );
}

void CrewController::threadLoop()
{
  bool flip = false;
  int dataCarriers = 192;
  int guardCarriers = 55;

  try{
    while(true)
    {
      boost::this_thread::interruption_point();

      boost::this_thread::sleep(boost::posix_time::milliseconds(3000));

      if(flip)
      {
        dataCarriers = 92;
        guardCarriers = 155;
      }
      else
      {
        dataCarriers = 192;
        guardCarriers = 55;
      }
      flip = !flip;


      //Reconfigure the radio by creating a reconfiguration set
      ReconfigSet r;

      ParametricReconfig p1,p2;
      p1.engineName = "phyengine1";
      p1.componentName = "ofdmmod1";
      p1.parameterName = "numdatacarriers";
      stringstream str1;
      str1 << dataCarriers;
      p1.parameterValue = str1.str();

      r.paramReconfigs.push_back(p1);

      p2.engineName = "phyengine1";
      p2.componentName = "ofdmmod1";
      p2.parameterName = "numguardcarriers";
      stringstream str2;
      str2.clear();
      str2 << guardCarriers;
      p2.parameterValue = str2.str();

      r.paramReconfigs.push_back(p2);

      reconfigureRadio(r);
    }
  }
  catch(boost::thread_interrupted&)
  {}
}

void CrewController::processEvent(Event &e)
{}

void CrewController::destroy()
{
  crewThread_->interrupt();
  crewThread_->join();
}

} // namespace iris
