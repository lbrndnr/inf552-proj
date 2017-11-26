CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

FIND_PACKAGE(OpenCV REQUIRED)

PROJECT(pano_stitching)

ADD_EXECUTABLE(pano src/main.cpp src/pano.cpp src/pano.h src/RANSAC.h src/RANSAC.tpp)
TARGET_LINK_LIBRARIES(pano ${OpenCV_LIBS})

ADD_EXECUTABLE(test test/RANSAC_test.cpp src/pano.cpp src/pano.h src/RANSAC.h src/RANSAC.tpp)
TARGET_LINK_LIBRARIES(test ${OpenCV_LIBS})
