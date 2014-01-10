#
# Copyright 2012-2013 The Iris Project Developers. See the
# COPYRIGHT file at the top-level directory of this distribution
# and at http://www.softwareradiosystems.com/iris/copyright.html.
#
# This file is part of the Iris Project.
#
# Iris is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as
# published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# Iris is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# A copy of the GNU Lesser General Public License can be found in
# the LICENSE file in the top-level directory of this distribution
# and at http://www.gnu.org/licenses/.
#

MESSAGE(STATUS "  Processing tutofdmmodulator.")

########################################################################
# Add includes and dependencies
########################################################################
FIND_PACKAGE( FFTW3F )

########################################################################
# Build the library from source files
########################################################################
SET(sources
	TutOfdmModulatorComponent.cpp
)

IF(FFTW3F_FOUND AND IRIS_HAVE_COMPLEXPLOT)
    INCLUDE_DIRECTORIES(${FFTW3F_INCLUDE_DIRS})
    
    # Static library to be used in tests
    ADD_LIBRARY(comp_gpp_phy_tutofdmmodulator_static STATIC ${sources})
    
    # Shared library to be used in radios
    ADD_LIBRARY(comp_gpp_phy_tutofdmmodulator SHARED ${sources})
    TARGET_LINK_LIBRARIES(comp_gpp_phy_tutofdmmodulator ${FFTW3F_LIBRARIES} complexplot)
    SET_TARGET_PROPERTIES(comp_gpp_phy_tutofdmmodulator PROPERTIES OUTPUT_NAME "tutofdmmodulator")
    IRIS_INSTALL(comp_gpp_phy_tutofdmmodulator)
    IRIS_APPEND_INSTALL_LIST(tutofdmmodulator)
    
    # Add the test and benchmark directories
    ADD_SUBDIRECTORY(test)
    ADD_SUBDIRECTORY(benchmark)
ELSE(FFTW3F_FOUND AND IRIS_HAVE_COMPLEXPLOT)
    IRIS_APPEND_NOINSTALL_LIST(tutofdmmodulator)
ENDIF(FFTW3F_FOUND AND IRIS_HAVE_COMPLEXPLOT)
