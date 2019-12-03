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
    ${ZCM_INCLUDE_DIRS}
)

include_directories(SYSTEM
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(vision-driver VisionDriver.cpp)

target_link_libraries(vision-driver
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${ZCM_LIBRARIES}
    ${YAMLCPP_LIBRARY}
    CameraInterface
    VisionUtils
    BallDetector
    Calibrator
    common
    m
)

add_executable(ball-detector-testing BallDetectorTesting.cpp)

target_link_libraries(ball-detector-testing
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${ZCM_LIBRARIES}
    ${YAMLCPP_LIBRARY}
    CameraInterface
    VisionUtils
    BallDetector
    Calibrator
    CloudTransformer
    common
    m
)

add_executable(calibration Calibration.cpp)

target_link_libraries(calibration
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${ZCM_LIBRARIES}
    ${YAMLCPP_LIBRARY}
    CameraInterface
    VisionUtils
    BallDetector
    Calibrator
    common
    m
)

if (BUILD_HOUGH_CIRCLE)
    add_executable(hough-circle HoughCircleTest.cpp)

    target_link_libraries(hough-circle
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${ZCM_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        CameraInterface
        VisionUtils
        BallDetector
        Calibrator
        common
        m
    )
endif()
