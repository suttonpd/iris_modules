/**
 * \file controllers/Crew/CrewController.h
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
 * An example controller to be used when creating new controllers.
 */

#ifndef CONTROLLERS_EXAMPLECONTROLLER_H_
#define CONTROLLERS_EXAMPLECONTROLLER_H_

#include "irisapi/Controller.h"
#include <boost/thread/thread.hpp>
#include <boost/scoped_ptr.hpp>

namespace iris
{

/** A simple controller to be used for CREW demonstrations
 *
 * Periodically reconfigures an ofdm modulation chain.
 */
class CrewController
  : public Controller
{
public:
  CrewController();
	virtual void subscribeToEvents();
	virtual void initialize();
  virtual void processEvent(Event &e);
	virtual void destroy();

private:
	void threadLoop();
	boost::scoped_ptr< boost::thread > crewThread_;
};

} // namespace iris

#endif // CONTROLLERS_EXAMPLECONTROLLER_H_
