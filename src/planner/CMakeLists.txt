if(NOT BUILD_PLANNER)
    return()
endif()

# find required packages
find_package(libusb-1.0 REQUIRED)
find_package(LCM REQUIRED)

include_directories(
    ${YAMLCPP_INCLUDE_DIRS}
    ${LCM_INCLUDE_DIR}
)

add_executable(ArmPlanner planner.cpp)

target_link_libraries( ArmPlanner LINK_PUBLIC ${LCM_LIBRARY} )
