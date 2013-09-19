MESSAGE(STATUS "  Processing liquidofdmmod.")

########################################################################
# Add includes and dependencies
########################################################################

FIND_PACKAGE(LIQUIDDSP REQUIRED)
FIND_PACKAGE(FFTW3F)

########################################################################
# Build the library from source files
########################################################################
SET(sources
  LiquidOfdmModComponent.cpp
)

IF (LIQUIDDSP_FOUND AND FFTW3F_FOUND)
  ADD_LIBRARY(comp_gpp_pn_liquidofdmmod SHARED ${sources})
  TARGET_LINK_LIBRARIES(comp_gpp_pn_liquidofdmmod ${LIQUIDDSP_LIBRARIES} ${FFTW3F_LIBRARY})
  SET_TARGET_PROPERTIES(comp_gpp_pn_liquidofdmmod PROPERTIES OUTPUT_NAME "liquidofdmmod")
  IRIS_INSTALL(comp_gpp_pn_liquidofdmmod)
  IRIS_APPEND_INSTALL_LIST(liquidofdmmod)
ELSE (LIQUIDDSP_FOUND AND FFTW3F_FOUND)
  IRIS_APPEND_NOINSTALL_LIST(liquidofdmmod)
ENDIF (LIQUIDDSP_FOUND AND FFTW3F_FOUND)
