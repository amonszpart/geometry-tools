project(matlabWrapper)
cmake_minimum_required(VERSION 2.8)

# !!! CHANGE THIS     !!!
SET( MATLAB_PATH "/media/Data2/Programs/MATLAB/R2013b" )
# !!! CHANGE THIS END !!!

#creates "build/Debug" and "build/Release" destinations
SET( dir ${CMAKE_CURRENT_SOURCE_DIR}/build/${CMAKE_BUILD_TYPE})
SET( EXECUTABLE_OUTPUT_PATH ${dir} CACHE PATH "Build directory" FORCE)
SET( LIBRARY_OUTPUT_PATH    ${dir} CACHE PATH "Build directory" FORCE)

# debug symbols, if needed
SET( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -g" )

# defines
#add_definitions( -DMATLAB_APP_PATH="\\"${MATLAB_PATH}/bin/matlab\\"" )
add_definitions( -DMATLAB_APP_PATH="\\"/home/bontius/matlab\\"" )

# might not be needed...
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -static-libstdc++" )

# includes
INCLUDE_DIRECTORIES(
    include
    ${MATLAB_PATH}/extern/include
)

#sources
SET( H_LIST
    include/matlab_wrapper/Matlab.h
)

SET( SRC_LIST
    src/Matlab.cpp
)

# libraries
SET( LIB_LIST
    ${MATLAB_PATH}/bin/glnxa64/libeng.so
    ${MATLAB_PATH}/bin/glnxa64/libmex.so
    ${MATLAB_PATH}/bin/glnxa64/libmx.so
) # might need more libs

# link lib
add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${H_LIST})
set_target_properties( ${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "d")
target_link_libraries( ${PROJECT_NAME}
    ${LIB_LIST}
)

# build tester executable
add_executable( test${PROJECT_NAME} src/main.cpp ${H_LIST})
target_link_libraries( test${PROJECT_NAME}
    ${PROJECT_NAME}
)

