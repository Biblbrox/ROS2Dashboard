#include <QtConcurrent/QtConcurrent>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <utility>
#include <yaml-cpp/node/node.h>

#include "GenericTextViz.hpp"
#include "GeometryViz.hpp"
#include "RasterViz.hpp"
#include "VisualizerModel.hpp"
#include "core/Logger.hpp"
#include "thirdy/dynamic_message_introspection/dynmsg/include/dynmsg/message_reading.hpp"
#include "thirdy/dynamic_message_introspection/dynmsg/include/dynmsg/typesupport.hpp"
#include "thirdy/dynamic_message_introspection/dynmsg/include/dynmsg/yaml_utils.hpp"

using rclcpp::Node;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud;
using std::make_shared;
using std::make_unique;
using std::unique_ptr;
using std_msgs::msg::String;

namespace ros2monitor {

VisualizerModel::VisualizerModel(int argc, char **argv, std::shared_ptr<DaemonClient> daemon_client, QObject *parent) : QAbstractListModel(parent), m_argc(argc), m_argv(argv)
{
    m_daemon_client = std::move(daemon_client);

    /**
     * All ros2 messages:
     * TODO: classify all of these message types to groups
    cartographer_ros_msgs/msg/HistogramBucket,
    cartographer_ros_msgs/msg/LandmarkEntry,
    cartographer_ros_msgs/msg/LandmarkList,
    cartographer_ros_msgs/msg/Metric,
    cartographer_ros_msgs/msg/MetricFamily,
    cartographer_ros_msgs/msg/MetricLabel,
    cartographer_ros_msgs/msg/StatusCode,
    cartographer_ros_msgs/msg/StatusResponse,
    cartographer_ros_msgs/msg/SubmapEntry,
    cartographer_ros_msgs/msg/SubmapList,
    cartographer_ros_msgs/msg/SubmapTexture,
    cartographer_ros_msgs/msg/TrajectoryStates,
    cascade_lifecycle_msgs/msg/Activation,
    cascade_lifecycle_msgs/msg/State,
    control_msgs/msg/AdmittanceControllerState,
    control_msgs/msg/DynamicJointState,
    control_msgs/msg/GripperCommand,
    control_msgs/msg/InterfaceValue,
    control_msgs/msg/JointComponentTolerance,
    control_msgs/msg/JointControllerState,
    control_msgs/msg/JointJog,
    control_msgs/msg/JointTolerance,
    control_msgs/msg/JointTrajectoryControllerState,
    control_msgs/msg/MecanumDriveControllerState,
    control_msgs/msg/PidState,
    control_msgs/msg/SteeringControllerStatus,
    controller_manager_msgs/msg/ChainConnection,
    controller_manager_msgs/msg/ControllerState,
    controller_manager_msgs/msg/HardwareComponentState,
    controller_manager_msgs/msg/HardwareInterface,
    diagnostic_msgs/msg/DiagnosticArray,
    diagnostic_msgs/msg/DiagnosticStatus,
    diagnostic_msgs/msg/KeyValue,
    dwb_msgs/msg/CriticScore,
    dwb_msgs/msg/LocalPlanEvaluation,
    dwb_msgs/msg/Trajectory2D,
    dwb_msgs/msg/TrajectoryScore,

    gazebo_msgs/msg/ContactState,
    gazebo_msgs/msg/ContactsState,
    gazebo_msgs/msg/EntityState,
    gazebo_msgs/msg/LinkState,
    gazebo_msgs/msg/LinkStates,
    gazebo_msgs/msg/ModelState,
    gazebo_msgs/msg/ModelStates,
    gazebo_msgs/msg/ODEJointProperties,
    gazebo_msgs/msg/ODEPhysics,
    gazebo_msgs/msg/PerformanceMetrics,
    gazebo_msgs/msg/SensorPerformanceMetric,
    gazebo_msgs/msg/WheelSlip,
    gazebo_msgs/msg/WorldState,
    geographic_msgs/msg/BoundingBox,
    geographic_msgs/msg/GeoPath,
    geographic_msgs/msg/GeoPoint,
    geographic_msgs/msg/GeoPointStamped,
    geographic_msgs/msg/GeoPose,
    geographic_msgs/msg/GeoPoseStamped,
    geographic_msgs/msg/GeoPoseWithCovariance,
    geographic_msgs/msg/GeoPoseWithCovarianceStamped,
    geographic_msgs/msg/GeographicMap,
    geographic_msgs/msg/GeographicMapChanges,
    geographic_msgs/msg/KeyValue,
    geographic_msgs/msg/MapFeature,
    geographic_msgs/msg/RouteNetwork,
    geographic_msgs/msg/RoutePath,
    geographic_msgs/msg/RouteSegment,
    geographic_msgs/msg/WayPoint,
    geometry_msgs/msg/Accel,
    geometry_msgs/msg/AccelStamped,
    geometry_msgs/msg/AccelWithCovariance,
    geometry_msgs/msg/AccelWithCovarianceStamped,
    geometry_msgs/msg/Inertia,
    geometry_msgs/msg/InertiaStamped,
    geometry_msgs/msg/Point,
    geometry_msgs/msg/Point32,
    geometry_msgs/msg/PointStamped,
    geometry_msgs/msg/Polygon,
    geometry_msgs/msg/PolygonStamped,
    geometry_msgs/msg/Pose,
    geometry_msgs/msg/Pose2D,
    geometry_msgs/msg/PoseArray,
    geometry_msgs/msg/PoseStamped,
    geometry_msgs/msg/PoseWithCovariance,
    geometry_msgs/msg/PoseWithCovarianceStamped,
    geometry_msgs/msg/Quaternion,
    geometry_msgs/msg/QuaternionStamped,
    geometry_msgs/msg/Transform,
    geometry_msgs/msg/TransformStamped,
    geometry_msgs/msg/Twist,
    geometry_msgs/msg/TwistStamped,
    geometry_msgs/msg/TwistWithCovariance,
    geometry_msgs/msg/TwistWithCovarianceStamped,
    geometry_msgs/msg/Vector3,
    geometry_msgs/msg/Vector3Stamped,
    geometry_msgs/msg/Wrench,
    geometry_msgs/msg/WrenchStamped,
    lifecycle_msgs/msg/State,
    lifecycle_msgs/msg/Transition,
    lifecycle_msgs/msg/TransitionDescription,
    lifecycle_msgs/msg/TransitionEvent,
    map_msgs/msg/OccupancyGridUpdate,
    map_msgs/msg/PointCloud2Update,
    map_msgs/msg/ProjectedMap,
    map_msgs/msg/ProjectedMapInfo,
    moveit_msgs/msg/AllowedCollisionEntry,
    moveit_msgs/msg/AllowedCollisionMatrix,
    moveit_msgs/msg/AttachedCollisionObject,
    moveit_msgs/msg/BoundingVolume,
    moveit_msgs/msg/CartesianPoint,
    moveit_msgs/msg/CartesianTrajectory,
    moveit_msgs/msg/CartesianTrajectoryPoint,
    moveit_msgs/msg/CollisionObject,
    moveit_msgs/msg/ConstraintEvalResult,
    moveit_msgs/msg/Constraints,
    moveit_msgs/msg/ContactInformation,
    moveit_msgs/msg/CostSource,
    moveit_msgs/msg/DisplayRobotState,
    moveit_msgs/msg/DisplayTrajectory,
    moveit_msgs/msg/GenericTrajectory,
    moveit_msgs/msg/Grasp,
    moveit_msgs/msg/GripperTranslation,
    moveit_msgs/msg/JointConstraint,
    moveit_msgs/msg/JointLimits,
    moveit_msgs/msg/KinematicSolverInfo,
    moveit_msgs/msg/LinkPadding,
    moveit_msgs/msg/LinkScale,
    moveit_msgs/msg/MotionPlanDetailedResponse,
    moveit_msgs/msg/MotionPlanRequest,
    moveit_msgs/msg/MotionPlanResponse,
    moveit_msgs/msg/MotionSequenceItem,
    moveit_msgs/msg/MotionSequenceRequest,
    moveit_msgs/msg/MotionSequenceResponse,
    moveit_msgs/msg/MoveItErrorCodes,
    moveit_msgs/msg/ObjectColor,
    moveit_msgs/msg/OrientationConstraint,
    moveit_msgs/msg/OrientedBoundingBox,
    moveit_msgs/msg/PlaceLocation,
    moveit_msgs/msg/PlannerInterfaceDescription,
    moveit_msgs/msg/PlannerParams,
    moveit_msgs/msg/PlanningOptions,
    moveit_msgs/msg/PlanningScene,
    moveit_msgs/msg/PlanningSceneComponents,
    moveit_msgs/msg/PlanningSceneWorld,
    moveit_msgs/msg/PositionConstraint,
    moveit_msgs/msg/PositionIKRequest,
    moveit_msgs/msg/RobotState,
    moveit_msgs/msg/RobotTrajectory,
    moveit_msgs/msg/TrajectoryConstraints,
    moveit_msgs/msg/VisibilityConstraint,
    moveit_msgs/msg/WorkspaceParameters,
    nav2_msgs/msg/BehaviorTreeLog,
    nav2_msgs/msg/BehaviorTreeStatusChange,
    nav2_msgs/msg/Costmap,
    nav2_msgs/msg/CostmapFilterInfo,
    nav2_msgs/msg/CostmapMetaData,
    nav2_msgs/msg/Particle,
    nav2_msgs/msg/ParticleCloud,
    nav2_msgs/msg/SpeedLimit,
    nav2_msgs/msg/VoxelGrid,
    nav_2d_msgs/msg/Path2D,
    nav_2d_msgs/msg/Pose2D32,
    nav_2d_msgs/msg/Pose2DStamped,
    nav_2d_msgs/msg/Twist2D,
    nav_2d_msgs/msg/Twist2D32,
    nav_2d_msgs/msg/Twist2DStamped,
    nav_msgs/msg/GridCells,
    nav_msgs/msg/MapMetaData,
    nav_msgs/msg/OccupancyGrid,
    nav_msgs/msg/Odometry,
    nav_msgs/msg/Path,
    object_recognition_msgs/msg/ObjectInformation,
    object_recognition_msgs/msg/ObjectType,
    object_recognition_msgs/msg/RecognizedObject,
    object_recognition_msgs/msg/RecognizedObjectArray,
    object_recognition_msgs/msg/Table,
    object_recognition_msgs/msg/TableArray,
    octomap_msgs/msg/Octomap,
    octomap_msgs/msg/OctomapWithPose,
    pcl_msgs/msg/ModelCoefficients,
    pcl_msgs/msg/PointIndices,
    pcl_msgs/msg/PolygonMesh,
    pcl_msgs/msg/Vertices,
    rclpy_message_converter_msgs/msg/NestedUint8ArrayTestMessage,
    rclpy_message_converter_msgs/msg/TestArray,
    rclpy_message_converter_msgs/msg/Uint8Array3TestMessage,
    rclpy_message_converter_msgs/msg/Uint8ArrayTestMessage,
    rmw_dds_common/msg/Gid,
    rmw_dds_common/msg/NodeEntitiesInfo,,
    rmw_dds_common/msg/ParticipantEntitiesInfo,
    robot_calibration_msgs/msg/CalibrationData,
    robot_calibration_msgs/msg/CameraParameter,
    robot_calibration_msgs/msg/CaptureConfig,
    robot_calibration_msgs/msg/ExtendedCameraInfo,
    robot_calibration_msgs/msg/Observation,
    robot_controllers_msgs/msg/ControllerState,
    robot_controllers_msgs/msg/DiffDriveLimiterParams,
    rosbag2_interfaces/msg/ReadSplitEvent,
    rosbag2_interfaces/msg/WriteSplitEvent,
    rosgraph_msgs/msg/Clock,
    sensor_msgs/msg/BatteryState,
    sensor_msgs/msg/CameraInfo,
    sensor_msgs/msg/ChannelFloat32,
    sensor_msgs/msg/CompressedImage,
    sensor_msgs/msg/FluidPressure,
    sensor_msgs/msg/Illuminance,

    sensor_msgs/msg/Imu,
    sensor_msgs/msg/JointState,
    sensor_msgs/msg/Joy,
    sensor_msgs/msg/JoyFeedback,
    sensor_msgs/msg/JoyFeedbackArray,
    sensor_msgs/msg/LaserEcho,
    sensor_msgs/msg/MagneticField,
    sensor_msgs/msg/MultiDOFJointState,
    sensor_msgs/msg/MultiEchoLaserScan,
    sensor_msgs/msg/NavSatFix,
    sensor_msgs/msg/NavSatStatus,

    sensor_msgs/msg/PointField,
    sensor_msgs/msg/Range,
    sensor_msgs/msg/RegionOfInterest,
    sensor_msgs/msg/RelativeHumidity,

    shape_msgs/msg/Mesh,
    shape_msgs/msg/MeshTriangle,
    shape_msgs/msg/Plane,
    shape_msgs/msg/SolidPrimitive,
    statistics_msgs/msg/MetricsMessage,
    statistics_msgs/msg/StatisticDataPoint,
    statistics_msgs/msg/StatisticDataType,

    stereo_msgs/msg/DisparityImage,
    system_modes_msgs/msg/Mode,
    system_modes_msgs/msg/ModeEvent,
    tf2_msgs/msg/TF2Error,
    tf2_msgs/msg/TFMessage,
    theora_image_transport/msg/Packet,
    trajectory_msgs/msg/JointTrajectory,
    trajectory_msgs/msg/JointTrajectoryPoint,
    trajectory_msgs/msg/MultiDOFJointTrajectory,
    trajectory_msgs/msg/MultiDOFJointTrajectoryPoint,
    turtlebot3_msgs/msg/SensorState,
    turtlebot3_msgs/msg/Sound,
    turtlebot3_msgs/msg/VersionInfo,
    turtlesim/msg/Color,
    turtlesim/msg/Pose,
    unique_identifier_msgs/msg/UUID,
    visualization_msgs/msg/ImageMarker,
    visualization_msgs/msg/InteractiveMarker,
    visualization_msgs/msg/InteractiveMarkerControl,
    visualization_msgs/msg/InteractiveMarkerFeedback,
    visualization_msgs/msg/InteractiveMarkerInit,
    visualization_msgs/msg/InteractiveMarkerPose,
    visualization_msgs/msg/InteractiveMarkerUpdate,
    visualization_msgs/msg/Marker,
    visualization_msgs/msg/MarkerArray,
    visualization_msgs/msg/MenuEntry,
    visualization_msgs/msg/MeshFile,
    visualization_msgs/msg/UVCoordinate,
     */

    // Initialize groups of ros2 messages
    m_geometryGroup = {
        "sensor_msgs/msg/LaserScan",
        "sensor_msgs/msg/PointCloud",
        "sensor_msgs/msg/PointCloud2",
    };

    m_rasterGroup = {
        "sensor_msgs/msg/Image",
    };

    m_textGroup = {
        "std_msgs/msg/Bool",
        "std_msgs/msg/Byte",
        "std_msgs/msg/ByteMultiArray",
        "std_msgs/msg/Char",
        "std_msgs/msg/ColorRGBA",
        "std_msgs/msg/Empty",
        "std_msgs/msg/Float32",
        "std_msgs/msg/Float32MultiArray",
        "std_msgs/msg/Float64",
        "std_msgs/msg/Float64MultiArray",
        "std_msgs/msg/Header",
        "std_msgs/msg/Int16",
        "std_msgs/msg/Int16MultiArray",
        "std_msgs/msg/Int32",
        "std_msgs/msg/Int32MultiArray",
        "std_msgs/msg/Int64",
        "std_msgs/msg/Int64MultiArray",
        "std_msgs/msg/Int8",
        "std_msgs/msg/Int8MultiArray",
        "std_msgs/msg/MultiArrayDimension",
        "std_msgs/msg/MultiArrayLayout",
        "std_msgs/msg/String",
        "std_msgs/msg/UInt16",
        "std_msgs/msg/UInt16MultiArray",
        "std_msgs/msg/UInt32",
        "std_msgs/msg/UInt32MultiArray",
        "std_msgs/msg/UInt64",
        "std_msgs/msg/UInt64MultiArray",
        "std_msgs/msg/UInt8",
        "std_msgs/msg/UInt8MultiArray",
        "action_msgs/msg/GoalInfo",
        "action_msgs/msg/GoalStatus",
        "action_msgs/msg/GoalStatusArray",
        "actionlib_msgs/msg/GoalID",
        "actionlib_msgs/msg/GoalStatus",
        "actionlib_msgs/msg/GoalStatusArray",
        "bond/msg/Constants",
        "bond/msg/Status",
        "builtin_interfaces/msg/Duration",
        "builtin_interfaces/msg/Time",
        "cartographer_ros_msgs/msg/BagfileProgress",
        "sensor_msgs/msg/Temperature",
        "sensor_msgs/msg/TimeReference",
        "example_interfaces/msg/Bool",
        "example_interfaces/msg/Byte",
        "example_interfaces/msg/ByteMultiArray",
        "example_interfaces/msg/Char",
        "example_interfaces/msg/Empty",
        "example_interfaces/msg/Float32",
        "example_interfaces/msg/Float32MultiArray",
        "example_interfaces/msg/Float64",
        "example_interfaces/msg/Float64MultiArray",
        "example_interfaces/msg/Int16",
        "example_interfaces/msg/Int16MultiArray",
        "example_interfaces/msg/Int32",
        "example_interfaces/msg/Int32MultiArray",
        "example_interfaces/msg/Int64",
        "example_interfaces/msg/Int64MultiArray",
        "example_interfaces/msg/Int8",
        "example_interfaces/msg/Int8MultiArray",
        "example_interfaces/msg/MultiArrayDimension",
        "example_interfaces/msg/MultiArrayLayout",
        "example_interfaces/msg/String",
        "example_interfaces/msg/UInt16",
        "example_interfaces/msg/UInt16MultiArray",
        "example_interfaces/msg/UInt32",
        "example_interfaces/msg/UInt32MultiArray",
        "example_interfaces/msg/UInt64",
        "example_interfaces/msg/UInt64MultiArray",
        "example_interfaces/msg/UInt8",
        "example_interfaces/msg/UInt8MultiArray",
        "example_interfaces/msg/WString",
        "rcl_interfaces/msg/FloatingPointRange",
        "rcl_interfaces/msg/IntegerRange",
        "rcl_interfaces/msg/ListParametersResult",
        "rcl_interfaces/msg/Log",
        "rcl_interfaces/msg/Parameter",
        "rcl_interfaces/msg/ParameterDescriptor",
        "rcl_interfaces/msg/ParameterEvent",
        "rcl_interfaces/msg/ParameterEventDescriptors",
        "rcl_interfaces/msg/ParameterType",
        "rcl_interfaces/msg/ParameterValue",
        "rcl_interfaces/msg/SetParametersResult",
    };
}

void VisualizerModel::update(std::vector<Ros2Connection> connections)
{
    m_connections = std::move(connections);
}

void VisualizerModel::addTopicViz(VisualizationType type, const std::string &topic_type, const std::string &topic_name, QQuickItem *item)
{
    if (m_components.contains(topic_name))
        return;

    if (m_visualizerNode == nullptr) {
        rclcpp::init(m_argc, m_argv);
        const std::string reserved_name = "visualization_node";
        m_visualizerNode = make_shared<Node>(reserved_name);

        m_subscribers["reserver_dummy_topic"] = m_visualizerNode->create_generic_subscription("reserver_dummy_topic", "std_msgs/msg/String", 10, [](auto _) {});
    }

    if (type == VisualizationType::raster) {
        m_components[topic_name] = item;
        m_visualizer_states[topic_name] = VisualizerState::running;
        m_subscribers[topic_name] = m_visualizerNode->create_generic_subscription(topic_name, topic_type, 10, [this, topic_name, topic_type](std::shared_ptr<rclcpp::SerializedMessage> image_serialized) {
            if (m_visualizer_states[topic_name] == VisualizerState::paused)
                return;
            using MessageT = Image;
            MessageT serialized;
            auto serializer = rclcpp::Serialization<MessageT>();
            serializer.deserialize_message(image_serialized.get(), &serialized);
            // Finally print the ROS 2 message data
            auto component = reinterpret_cast<viz::RasterViz *>(m_components[topic_name]);
            component->updateData(serialized);
        });
    } else if (type == VisualizationType::geometry) {
        m_components[topic_name] = item;
        m_visualizer_states[topic_name] = VisualizerState::running;
        m_subscribers[topic_name] = m_visualizerNode->create_generic_subscription(topic_name, topic_type, 10, [this, topic_name](std::shared_ptr<rclcpp::SerializedMessage> msg) {
            if (m_visualizer_states[topic_name] == VisualizerState::paused)
                return;
            using MessageT = sensor_msgs::msg::PointCloud2;
            MessageT serialized;
            auto serializer = rclcpp::Serialization<MessageT>();
            serializer.deserialize_message(msg.get(), &serialized);
            // Finally print the ROS 2 message data
            auto component = reinterpret_cast<viz::GeometryViz *>(m_components[topic_name]);
            component->updateData(serialized);
        });
    } else if (type == VisualizationType::string) {
        assert(std::find(m_textGroup.cbegin(), m_textGroup.cend(), topic_type) != m_textGroup.cend());
        m_components[topic_name] = item;

        m_visualizer_states[topic_name] = VisualizerState::running;
        m_subscribers[topic_name] = m_visualizerNode->create_generic_subscription(topic_name, topic_type, 10, [this, topic_name, topic_type](std::shared_ptr<rclcpp::SerializedMessage> msg) {
            if (m_visualizer_states[topic_name] == VisualizerState::paused)
                return;

            // Convert received message to YAML format
            RosMessage_Cpp ros_msg;
            InterfaceTypeName interface {
                "std_msgs", "Header"
            };
            ros_msg.type_info = dynmsg::cpp::get_type_info(interface);
            ros_msg.data = reinterpret_cast<uint8_t *>(msg.get());
            YAML::Node yaml_msg = dynmsg::cpp::message_to_yaml(ros_msg);
            const std::string yaml_string = dynmsg::yaml_to_string(yaml_msg);

            auto component = reinterpret_cast<viz::GenericTextViz *>(m_components[topic_name]);
            component->updateData(yaml_string);
        });
    }

    if (!m_nodeFuture.isRunning()) {
        m_daemon_client->killNodeRequest("visualization_node");
        m_nodeFuture = QtConcurrent::run([this]() {
            Logger::debug("Running node");
            rclcpp::spin(m_visualizerNode);
            rclcpp::shutdown();
        });
    }
}

QVariant VisualizerModel::data(const QModelIndex &index, int role) const
{
    // TODO: deprecated. I have the separate topic list mode for this purpose
    if (!index.isValid())
        return QVariant{};

    size_t idx = index.row();
    auto it = m_components.begin();
    std::advance(it, idx);

    QVariant value;

    if (role == TopicRoleName) {
        /*TopicQml topic(parent());
        topic.setName()
        value.setValue()*/
    }
    return value;
}

int VisualizerModel::rowCount(const QModelIndex &parent) const
{
    return m_components.size();
}

QHash<int, QByteArray> VisualizerModel::roleNames() const
{
    QHash<int, QByteArray> roles;
    //roles[ComponentRoleName] = "component";
    roles[TopicRoleName] = "topic";
    return roles;
}

VisualizerModel::~VisualizerModel()
{
    rclcpp::shutdown();
    m_nodeFuture.waitForFinished();
}

bool VisualizerModel::hasTopicViz(const QString &topic_name)
{
    return m_components.contains(topic_name.toStdString());
}

QString VisualizerModel::getTopicCategory(const QString &topic_type)
{
    bool isText = inTextGroup(topic_type.toStdString());
    bool isGeometry = inGeometryGroup(topic_type.toStdString());
    bool isRaster = inRasterGroup(topic_type.toStdString());

    if (isText)
        return "text";

    if (isGeometry)
        return "geometry";

    if (isRaster)
        return "raster";

    return "unknown";
}

bool VisualizerModel::inTextGroup(const std::string &topic_type)
{
    return (std::find(m_textGroup.cbegin(), m_textGroup.cend(), topic_type) != m_textGroup.cend());
}

bool VisualizerModel::inGeometryGroup(const std::string &topic_type)
{
    return (std::find(m_geometryGroup.cbegin(), m_geometryGroup.cend(), topic_type) != m_geometryGroup.cend());
}

bool VisualizerModel::inRasterGroup(const std::string &topic_type)
{
    return (std::find(m_rasterGroup.cbegin(), m_rasterGroup.cend(), topic_type) != m_rasterGroup.cend());
}

void VisualizerModel::pauseViz(const QString &topic_name)
{
    if (!m_components.contains(topic_name.toStdString()))
        return;

    m_visualizer_states[topic_name.toStdString()] = VisualizerState::paused;
}

void VisualizerModel::resumeViz(const QString &topic_name)
{
    if (!m_components.contains(topic_name.toStdString()))
        return;

    m_visualizer_states[topic_name.toStdString()] = VisualizerState::running;
}

void VisualizerModel::removeViz(const QString &topic_name)
{
    std::string topic_name_std = topic_name.toStdString();

    if (!m_components.contains(topic_name_std))
        return;

    auto comp_it = m_components.find(topic_name_std);
    m_components.erase(comp_it);

    auto viz_state_it = m_visualizer_states.find(topic_name_std);
    m_visualizer_states.erase(viz_state_it);

    auto viz_sub_it = m_subscribers.find(topic_name_std);
    viz_sub_it->second.reset();// TODO: I am not sure yet if it is necessary to explicitly call reset
    m_subscribers.erase(viz_sub_it);

    emit topicVizRemoved(topic_name);
}
}