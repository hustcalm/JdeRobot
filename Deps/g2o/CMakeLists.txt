FIND_PACKAGE(G2O)

IF(G2O_FOUND)
MESSAGE(STATUS "  With g2o  = YES")
ELSE()
MESSAGE(STATUS "  With g2o  = NO (g2o not found)")
ENDIF()

