#pragma once

#include <cstddef>

namespace ros2monitor {
/**
 * @brief The PointCloudSensorInfo struct
 * This struct is used to store information about point cloud sensor and cloud data.
 */
struct PointCloudSensorInfo {
    std::string frame_id;
    int32_t stamp_sec;
    uint32_t stamp_nanosec;
    size_t width;
    size_t height;
    size_t point_step;
    size_t row_step;
    size_t points_num;
    bool is_bigendian;
    bool is_dense;

    std::string to_string() const
    {
        return "frame_id: " + frame_id + "\n" + "stamp_sec: " + std::to_string(stamp_sec) + "\n" + "stamp_nanosec: " + std::to_string(stamp_nanosec) + "\n" + "width: " + std::to_string(width) + "\n" + "height: " + std::to_string(height) + "\n" + "point_step: " + std::to_string(point_step) + "\n" + "row_step: " + std::to_string(row_step) + "\n" + "points_num: " + std::to_string(points_num) + "\n" + "is_bigendian: " + std::to_string(is_bigendian) + "\n" + "is_dense: " + std::to_string(is_dense) + "\n";
    }
};

/**
 * @brief The ImageSensorInfo struct
 * This struct is used to store information about image sensor and raster data
 */
struct ImageSensorInfo {
    std::string frame_id;
    int32_t stamp_sec;
    uint32_t stamp_nanosec;
    size_t width;
    size_t height;
    std::string encoding;
    size_t step;

    std::string to_string() const
    {
        return "frame_id: " + frame_id + "\n" + "stamp_sec: " + std::to_string(stamp_sec) + "\n" + "stamp_nanosec: " + std::to_string(stamp_nanosec) + "\n" + "width: " + std::to_string(width) + "\n" + "height: " + std::to_string(height) + "\n" + "encoding: " + encoding + "\n" + "step: " + std::to_string(step) + "\n";
    }
};

}// namespace ros2monitor
