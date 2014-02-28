SET(PROJECT_NAME PCLTools)
PROJECT( ${PROJECT_NAME} )
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

#### !! HOST specific ####
INCLUDE( "../local_dependencies.cmake" )

#### DEPENDENCIES ####
###### PCL ######
FIND_PACKAGE( PCL REQUIRED )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )
###### VTK ######
FIND_PACKAGE( VTK REQUIRED )
INCLUDE_DIRECTORIES( ${VTK_INCLUDE_DIRS} )

#### SOURCE ####
SET( HPP_LIST
    include/pcltools/primitives.hpp
    include/pcltools/util.hpp
)
SET( H_LIST
    include/pcltools/primitives.h
)

SET( SRC_LIST
    src/pcltools/primitives.cpp
)

#### EXECUTABLE ####
INCLUDE_DIRECTORIES( include )
ADD_LIBRARY( ${PROJECT_NAME} SHARED ${SRC_LIST} ${H_LIST} ${HPP_LIST} )
SET_TARGET_PROPERTIES( ${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "d")

#### LINKING ####
TARGET_LINK_LIBRARIES( ${PROJECT_NAME}
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
)

## TESTer executable ##
SET( TESTER_TARGET_NAME test${PROJECT_NAME} )

get_property( lib_location TARGET ${PROJECT_NAME} PROPERTY LOCATION )
message (STATUS "lib_location == ${lib_location}")

#### SOURCE ####
SET( TEST_SRC_LIST
    test/main.cpp
)

ADD_EXECUTABLE( ${TESTER_TARGET_NAME} ${TEST_SRC_LIST} )
TARGET_LINK_LIBRARIES( ${TESTER_TARGET_NAME}
    ${lib_location}
)




