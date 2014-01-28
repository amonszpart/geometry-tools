SET(PROJECT_NAME PCLSuite)
PROJECT( ${PROJECT_NAME} )
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

#### !! HOST specific ####
SET(PCL_DIR "/home/bontius/workspace/3rdparty/pcl-trunk/install/share/pcl-1.7")

#### DEPENDENCIES ####
FIND_PACKAGE( PCL REQUIRED )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )

#### SOURCE ####
SET( H_LIST
)

SET( SRC_LIST
    src/main.cpp
)

#### EXECUTABLE
#ADD_LIBRARY( ${PROJECT_NAME} SHARED ${SRC_LIST} ${H_LIST} )
ADD_EXECUTABLE( ${PROJECT_NAME} ${SRC_LIST} ${H_LIST} )

# LINKING
TARGET_LINK_LIBRARIES( ${PROJECT_NAME}
    ${PCL_LIBRARIES}
    vtkCommon
    vtkFiltering
    vtkRendering
)




