########################################################################
# Setup Qt and Qwt
########################################################################
FIND_PACKAGE(Qt4 REQUIRED)
IF(QT4_FOUND)
    INCLUDE(${QT_USE_FILE})
ENDIF(QT4_FOUND) 

FIND_PACKAGE(Qwt REQUIRED)
IF(QT4_FOUND AND QWT_FOUND)
    INCLUDE_DIRECTORIES(${QWT_INCLUDE_DIRS})
ENDIF(QT4_FOUND AND QWT_FOUND)
 
########################################################################
# Build our library
########################################################################
IF(QT4_FOUND AND QWT_FOUND)
    SET(sources
      PacketGeneratorGuiWidget.cpp
      PacketGeneratorGuiWidget.h
      PacketGeneratorGuiWrapper.cpp
      PacketGeneratorGuiWrapper.h
      PacketGeneratorGui.cpp
      PGGuiCallback.h
    )
    QT4_WRAP_CPP(MOCSrcs PacketGeneratorGuiWidget.h PacketGeneratorGuiWrapper.h)
    ADD_LIBRARY(packetgeneratorgui ${sources} ${MOCSrcs})
    TARGET_LINK_LIBRARIES(packetgeneratorgui ${QT_LIBRARIES} ${QWT_LIBRARIES})
    IRIS_SET_PIC(packetgeneratorgui)
    SET(IRIS_HAVE_PACKETGENERATORGUI ON CACHE INTERNAL "Has PacketGeneratorGui been built?")
    ADD_SUBDIRECTORY(test)
ELSE(QT4_FOUND AND QWT_FOUND)
    SET(IRIS_HAVE_PACKETGENERATORGUI OFF CACHE INTERNAL "Has PacketGeneratorGui been built?")
ENDIF(QT4_FOUND AND QWT_FOUND)
