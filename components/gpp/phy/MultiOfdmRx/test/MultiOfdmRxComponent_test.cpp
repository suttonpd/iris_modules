/**
 * \file components/gpp/phy/OfdmModulator/OfdmModulatorComponent_test.cpp
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
 * Main test file for OfdmModulator component.
 */

#define BOOST_TEST_MODULE OfdmModulatorComponent_Test

#include <boost/test/unit_test.hpp>

#include "../MultiOfdmRxComponent.h"
#include "utility/DataBufferTrivial.h"
#include "utility/RawFileUtility.h"

using namespace std;
using namespace iris;
using namespace iris::phy;

BOOST_AUTO_TEST_SUITE (MultiOfdmRxComponent_Test)

BOOST_AUTO_TEST_CASE(MultiOfdmRxComponent_Basic_Test)
{
  BOOST_REQUIRE_NO_THROW(MultiOfdmRxComponent mod("test"));
}

BOOST_AUTO_TEST_CASE(MultiOfdmRxComponent_Parm_Test)
{
  MultiOfdmRxComponent mod("test");
  BOOST_CHECK(mod.getParameterDefaultValue("nchans") == "1");
  BOOST_CHECK(mod.getParameterDefaultValue("ncarriers") == "64");
  BOOST_CHECK(mod.getParameterDefaultValue("cplen") == "16");
  BOOST_CHECK(mod.getParameterDefaultValue("taperlen") == "4");

  BOOST_CHECK(mod.getValue("nchans") == "1");
  BOOST_CHECK(mod.getValue("ncarriers") == "64");
  BOOST_CHECK(mod.getValue("cplen") == "16");
  BOOST_CHECK(mod.getValue("taperlen") == "4");
}

BOOST_AUTO_TEST_CASE(MultiOfdmRxComponent_Ports_Test)
{
  MultiOfdmRxComponent mod("test");
  BOOST_REQUIRE_NO_THROW(mod.registerPorts());

  vector<Port> iPorts = mod.getInputPorts();
  BOOST_REQUIRE(iPorts.size() == 1);
  BOOST_REQUIRE(iPorts.front().portName == "input1");
  BOOST_REQUIRE(iPorts.front().supportedTypes.front() ==
      TypeInfo< complex<float> >::identifier);

  vector<Port> oPorts = mod.getOutputPorts();
  BOOST_REQUIRE(oPorts.size() == 1);
  BOOST_REQUIRE(oPorts.front().portName == "output1");
  BOOST_REQUIRE(oPorts.front().supportedTypes.front() ==
      TypeInfo< uint8_t >::identifier);

  map<string, int> iTypes,oTypes;
  iTypes["input1"] = TypeInfo< complex<float> >::identifier;
  mod.calculateOutputTypes(iTypes,oTypes);
  BOOST_REQUIRE(oTypes["output1"] == TypeInfo< uint8_t >::identifier);
}

BOOST_AUTO_TEST_CASE(MultiOfdmRxComponent_Init_Test)
{
  MultiOfdmRxComponent mod("test");
  mod.registerPorts();

  map<string, int> iTypes,oTypes;
  iTypes["input1"] = TypeInfo< complex<float> >::identifier;
  mod.calculateOutputTypes(iTypes,oTypes);

  BOOST_REQUIRE_NO_THROW(mod.initialize());
}

BOOST_AUTO_TEST_CASE(MultiOfdmRxComponent_Process_Test)
{
  MultiOfdmRxComponent mod("test");
  mod.setValue("nchans", "2");
  mod.registerPorts();

  map<string, int> iTypes,oTypes;
  iTypes["input1"] = TypeInfo< complex<float> >::identifier;
  mod.calculateOutputTypes(iTypes,oTypes);

  DataBufferTrivial< complex<float> > in;
  DataBufferTrivial< uint8_t > out;

  DataSet< complex<float> >* iSet = NULL;
  int size = RawFileUtility::getNumElements< complex<float> >("multiofdmframe");
  in.getWriteData(iSet, size);
  RawFileUtility::read(iSet->data.begin(), iSet->data.end(), "multiofdmframe");
  in.releaseWriteData(iSet);

  mod.setBuffers(&in,&out);
  mod.initialize();
  BOOST_REQUIRE_NO_THROW(mod.process());

  BOOST_REQUIRE(out.hasData());
}

BOOST_AUTO_TEST_SUITE_END()
