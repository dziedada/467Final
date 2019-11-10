# find required packages
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(LibRealSense2 REQUIRED)
find_package(ZCM REQUIRED)
find_package(libusb-1.0 REQUIRED)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(
    ${OPENCV_INCLUDE_DIR}
    ${YAMLCPP_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${LIBREALSENSE2_INCLUDE_DIRS}
    ${ZCM_INCLUDE+DIRS}
)

include_directories(SYSTEM
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(RealsenseSettings SHARED RealsenseSettings.cpp)

add_library(CameraInterface SHARED
    RealsenseInterface.cpp
)

add_library(VisionUtils SHARED utilities.cpp)

target_link_libraries(CameraInterface
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${LIBREALSENSE2_LIBRARIES}
    ${LIBUSB_1_LIBRARIES}
    RealsenseSettings
    m
)

target_link_libraries(CameraInterface realsense2)

target_link_libraries(VisionUtils
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${Octomap_LIBRARIES}
)