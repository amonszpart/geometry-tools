SET(PROJECT_NAME PCLTools)
PROJECT( ${PROJECT_NAME} )
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

#### !! HOST specific ####
INCLUDE( "../local_dependencies.cmake" ) # contains SET( PCL_DIR ".../pcl-trunk/install/share/pcl-1.7/")

SET( dir ${CMAKE_CURRENT_SOURCE_DIR}/build/${CMAKE_BUILD_TYPE}/)
SET( EXECUTABLE_OUTPUT_PATH ${dir} CACHE PATH "Build directory" FORCE)
SET( LIBRARY_OUTPUT_PATH    ${dir} CACHE PATH "Build directory" FORCE)
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x" )
SET( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -g" )

#### DEPENDENCIES ####
###### PCL ######
#FIND_PACKAGE( PCL REQUIRED COMPONENTS common io search features segmentation visualization sample_consensus)
SET( PCL_INCLUDE_DIRS "${PCL_DIR}/../../include")
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} "/usr/include/eigen3" )
###### VTK ######
#FIND_PACKAGE( VTK REQUIRED )
#INCLUDE_DIRECTORIES( ${VTK_INCLUDE_DIRS} )

#### SOURCE ####
SET( HPP_LIST
    include/pcltools/primitives.hpp
    include/pcltools/util.hpp
)
SET( H_LIST
    include/pcltools/primitives.h
    include/smartgeometry/util.h
)

#SET( SRC_LIST
    #src/pcltools/primitives.cpp
    #src/geometry/geometry.cpp
    #src/util.cpp
#)

#### EXECUTABLE ####
INCLUDE_DIRECTORIES( include )
ADD_LIBRARY( ${PROJECT_NAME} SHARED ${SRC_LIST} ${H_LIST} ${HPP_LIST} )
SET_TARGET_PROPERTIES( ${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "d")
set_target_properties( ${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

#### LINKING ####

## TESTer executable ##
#SET( TESTER_TARGET_NAME test${PROJECT_NAME} )

#get_property( lib_location TARGET ${PROJECT_NAME} PROPERTY LOCATION )
#message (STATUS "lib_location == ${lib_location}")
#get_property( debug_postfix TARGET ${PROJECT_NAME} PROPERTY DEBUG_POSTFIX)
#message (STATUS "debug_postfix== ${debug_postfix}")
#message( STATUS "ARCHIVE_OUTPUT_DIRECTORY: " ${ARCHIVE_OUTPUT_DIRECTORY} )
#message( STATUS "RUNTIME_OUTPUT_DIRECTORY: " ${RUNTIME_OUTPUT_DIRECTORY} )


#### SOURCE ####
#SET( TEST_SRC_LIST
#    test/main.cpp
#)

#TARGET_LINK_LIBRARIES( ${TESTER_TARGET_NAME}
    #${PCL_LIBRARIES}
    #${VTK_LIBRARIES}
#)

#ADD_EXECUTABLE( ${TESTER_TARGET_NAME} ${TEST_SRC_LIST} )
#TARGET_LINK_LIBRARIES( ${TESTER_TARGET_NAME}
#    ${lib_location}
#)




