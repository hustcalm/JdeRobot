
ADD_SUBDIRECTORY( BOWMapping )

IF(TARGET rtabmap_gui)
   ADD_SUBDIRECTORY( RGBDMapping )
   ADD_SUBDIRECTORY( WifiMapping )
ELSE()
   MESSAGE(STATUS "RTAB-Map GUI lib is not built, the RGBDMapping and WifiMapping examples will not be built...")
ENDIF()


