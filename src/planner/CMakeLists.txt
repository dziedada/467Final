if(NOT BUILD_PLANNER)
    return()
endif()

# find required packages
find_package(libusb-1.0 REQUIRED)
find_package(LCM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    ${YAMLCPP_INCLUDE_DIRS}
    ${LCM_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${OPENCV_INCLUDE_DIR}
)

add_executable(ArmPlanner planner.cpp)

target_link_libraries( ArmPlanner LINK_PUBLIC ${LCM_LIBRARY} ${OpenCV_LIBS})
