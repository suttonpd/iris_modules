/**
 * \file controllers/PacketGenerator/PacketGeneratorGui/test/PacketGeneratorGui_test.cpp
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
 * Main test file for PacketGeneratorGui class.
 */

#define BOOST_TEST_MODULE PacketGeneratorGui_Test

#include "PacketGeneratorGui.h"

#include <boost/scoped_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/progress.hpp>
#include <boost/bind.hpp>
#include <qapplication.h>
#include <cstdlib>
#include <vector>
#include <boost/test/unit_test.hpp>
#include <iostream>

using namespace std;

void threadMain1()
{
  PacketGeneratorGui gui;
  boost::this_thread::sleep(boost::posix_time::seconds(2));

}

BOOST_AUTO_TEST_SUITE (PacketGeneratorGui_Test)

BOOST_AUTO_TEST_CASE(PacketGeneratorGui_Init_Test)
{
  int argc = 1;
  char* argv[] = { const_cast<char *>("PacketGeneratorGui_Init_Test"), NULL };
  QApplication a(argc, argv);

  boost::scoped_ptr< boost::thread > thread1_;
  thread1_.reset( new boost::thread( &threadMain1 ) );
  qApp->exec();
  thread1_->join();
}

BOOST_AUTO_TEST_SUITE_END()
