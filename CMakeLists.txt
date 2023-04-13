cmake_minimum_required(VERSION 3.8)
project(tara_ros2)

CMAKE_MINIMUM_REQUIRED(VERSION 3.14)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    #  add_compile_options(-Wall -Wextra)
endif ()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(GLIB REQUIRED glib-2.0)

find_package(Tara REQUIRED)

find_package(OpenCV REQUIRED)
message("-- FOUND OpenCV Libs at\n\"${OpenCV_LIBS}\"")

set(CMAKE_CXX_FLAGS
        "${SSE_FLAGS} -O2 -g -std=c++17 -march=native -fno-omit-frame-pointer"
        )

add_executable(tara_node src/tara_node.cpp)
target_link_libraries(tara_node ${OpenCV_LIBS} ${GLIB_LDFLAGS} ${Tara_LIBS})
ament_target_dependencies(tara_node rclcpp sensor_msgs cv_bridge)

include_directories(
        ${PROJECT_SOURCE_DIR}/src
        ${OpenCV_INCLUDE_DIRS}
        ${GLIB_INCLUDE_DIRS}
        ${Tara_INCLUDE_DIRS}
)

install(TARGETS
        tara_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch config
        DESTINATION share/${PROJECT_NAME}/
        )

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
