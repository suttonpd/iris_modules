MESSAGE(STATUS "  Processing liquidofdmdemod.")

########################################################################
# Add includes and dependencies
########################################################################

FIND_PACKAGE(LIQUIDDSP REQUIRED)
FIND_PACKAGE(FFTW3F)

########################################################################
# Build the library from source files
########################################################################
SET(sources
    LiquidOfdmDemodComponent.cpp
)

IF (LIQUIDDSP_FOUND AND FFTW3F_FOUND)
  ADD_LIBRARY(comp_gpp_pn_liquidofdmdemod SHARED ${sources})
  TARGET_LINK_LIBRARIES(comp_gpp_pn_liquidofdmdemod ${LIQUIDDSP_LIBRARIES} ${FFTW3F_LIBRARY})
  SET_TARGET_PROPERTIES(comp_gpp_pn_liquidofdmdemod PROPERTIES OUTPUT_NAME "liquidofdmdemod")
  ADD_DEPENDENCIES(comp_gpp_pn_liquidofdmdemod liquid-dsp)
  IRIS_INSTALL(comp_gpp_pn_liquidofdmdemod)
  IRIS_APPEND_INSTALL_LIST(liquidofdmdemod)
ELSE (LIQUIDDSP_FOUND AND FFTW3F_FOUND)
  IRIS_APPEND_NOINSTALL_LIST(liquidofdmdemod)
ENDIF (LIQUIDDSP_FOUND AND FFTW3F_FOUND)
