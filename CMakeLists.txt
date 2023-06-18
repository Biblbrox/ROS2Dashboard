cmake_minimum_required(VERSION 3.25)

set(PROJECT_NAME ROS2Dashboard)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 20)

find_package(Qt6 COMPONENTS Core Gui Quick Qml WebEngineCore WebEngineQuick QuickControls2 REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(asio CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
qt_standard_project_setup()
find_package(QuickQanava REQUIRED)
find_package(Catch2 CONFIG REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS $<$<CONFIG:Debug>:QT_QML_DEBUG>)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SOURCES src/main.cpp res.qrc src/ros2_entities/Ros2Node.hpp src/ros2_entities/Ros2Topic.hpp src/ros2_entities/Ros2Entity.hpp src/daemon_client/DaemonClient.hpp src/ros2_entities/Ros2State.hpp src/ros2_entities/Ros2State.cpp src/daemon_client/DaemonClient.cpp src/core/Logger.hpp src/core/Logger.cpp src/core/Application.hpp src/core/Application.cpp src/qml/models/Ros2EntityListModel.hpp src/qml/models/Ros2EntityListModel.cpp src/qml/models/Ros2NodeListModel.hpp src/qml/models/Ros2NodeListModel.cpp src/qml/models/Ros2PackageListModel.hpp src/qml/models/Ros2PackageListModel.cpp src/ros2_entities/Ros2Node.cpp src/ros2_entities/Ros2Topic.cpp src/ros2_entities/Ros2Package.cpp src/network/Bytes.hpp src/network/Bytes.cpp src/core/Platform.hpp src/qml/models/Ros2TopicListModel.hpp src/qml/models/Ros2TopicListModel.cpp src/ros2_entities/Ros2Connection.hpp src/ros2_entities/Ros2Subscriber.hpp src/ros2_entities/Ros2Subscriber.cpp src/ros2_entities/Ros2Publisher.hpp src/ros2_entities/Ros2Publisher.cpp src/qml/models/Ros2ConnectionListModel.hpp src/qml/models/Ros2ConnectionListModel.cpp)
add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE QuickQanava Qt6::Core Qt6::Gui Qt6::Quick Qt6::Qml)
target_link_libraries(${PROJECT_NAME} PRIVATE Qt6::WebEngineCore Qt6::WebEngineQuick)
target_link_libraries(${PROJECT_NAME} PRIVATE spdlog::spdlog)
target_link_libraries(${PROJECT_NAME} PRIVATE asio::asio systemd)
target_link_libraries(${PROJECT_NAME} PRIVATE nlohmann_json::nlohmann_json)
target_link_libraries(${PROJECT_NAME} PRIVATE Catch2::Catch2 Catch2::Catch2WithMain)
#target_link_libraries(${PROJECT_NAME} PRIVATE WebEngineWidgets)

set(QuickQanavaDir ${CMAKE_CURRENT_SOURCE_DIR}/thirdy/QuickQanava/)
target_include_directories(${PROJECT_NAME} PRIVATE ${QuickQanavaDir}/src ${QuickQanavaDir}/QuickContainers/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
