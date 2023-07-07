cmake_minimum_required(VERSION 3.25)

#set(CMAKE_C_COMPILER "/usr/bin/gcc-9")
#set(CMAKE_CXX_COMPILER "/usr/bin/g++-9")


set(PROJECT_NAME ros2_dashboard)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 20)

find_package(Qt6 COMPONENTS Core Gui Quick Qml WebEngineCore WebEngineQuick QuickControls2 REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(asio CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(QuickQanava REQUIRED)
find_package(Catch2 CONFIG REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
qt_standard_project_setup()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/thirdy/dynamic_message_introspection/dynmsg)

install(
        DIRECTORY config launch maps meshes models params rviz src worlds
        DESTINATION share/${PROJECT_NAME}
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


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS $<$<CONFIG:Debug>:QT_QML_DEBUG>)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

FILE(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_SOURCE_DIR} "src/*.cpp" "src/*.hpp")

add_executable(${PROJECT_NAME} ${SOURCES} res.qrc)
target_link_libraries(${PROJECT_NAME} PUBLIC QuickQanava Qt6::Core Qt6::Gui Qt6::Quick Qt6::Qml)
target_link_libraries(${PROJECT_NAME} PUBLIC Qt6::WebEngineCore Qt6::WebEngineQuick)
target_link_libraries(${PROJECT_NAME} PUBLIC spdlog::spdlog)
target_link_libraries(${PROJECT_NAME} PUBLIC asio::asio systemd)
target_link_libraries(${PROJECT_NAME} PUBLIC nlohmann_json::nlohmann_json)
target_link_libraries(${PROJECT_NAME} PUBLIC dynmsg)


set(QML_CPP_OBJECTS ${CMAKE_CURRENT_SOURCE_DIR}/src/qml/visualization/GenericTextViz.hpp ${CMAKE_CURRENT_SOURCE_DIR}/src/qml/visualization/GenericTextViz.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/qml/visualization/RasterViz.hpp ${CMAKE_CURRENT_SOURCE_DIR}/src/qml/visualization/RasterViz.cpp)
message(${QML_CPP_OBJECTS})


#target_link_libraries(${PROJECT_NAME} PRIVATE WebEngineWidgets)

set(QuickQanavaDir ${CMAKE_CURRENT_SOURCE_DIR}/thirdy/QuickQanava/)
target_include_directories(${PROJECT_NAME} PRIVATE ${QuickQanavaDir}/src ${QuickQanavaDir}/QuickContainers/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/src/qml/visualization)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/src/qml/models)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/thirdy/dynamic_message_introspection/dynmsg/include)

ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp sensor_msgs)
ament_package()


qt_add_qml_module(${PROJECT_NAME}
        URI com.viz.types
        VERSION 1.0
        SOURCES
        ${QML_CPP_OBJECTS}
        )
