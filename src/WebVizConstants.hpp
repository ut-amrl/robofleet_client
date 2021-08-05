#include <string>

/**
 * These constants are topic names with special significance to the web
 * visualizer.
 *
 */
namespace webviz_constants {
/**
 * @brief (amrl_msgs/RobofleetStatus) Robofleet status information for listing
 * robot in webviz overview.
 */
static const std::string status_topic = "status";

/**
 * @brief (amrl_msgs/RobofleetSubscription) Robofleet subscriptions topic; for
 * managing your own subscriptions.
 *
 * Note that this is a single, global topic, which is not namespaced for each
 * robot.
 */
static const std::string subscriptions_topic = "/subscriptions";

/**
 * @brief (amrl_msgs/Localization2DMsg) Current robot localization
 */
static const std::string localization_topic = "localization";

/**
 * @brief (nav_msgs/Odometry) Current robot odometry
 */
static const std::string odometry_topic = "odometry/raw";

/**
 * @brief (sensor_msgs/LaserScan) LIDAR scan data for localization
 */
static const std::string lidar_2d_topic = "velodyne_2dscan";

/**
 * @brief (sensor_msgs/LaserScan) LIDAR scan data for navigation
 */
static const std::string obstacle_scan_topic = "obstacle_scan";

/**
 * @brief (sensor_msgs/CompressedImage) prefix for a Compressed Image from a
 * stereo camera.
 */
static const std::string compressed_image_prefix = "image_compressed/";

/**
 * @brief (sensor_msgs/PointCloud2) 3D point cloud
 */
static const std::string point_cloud_topic = "pointcloud";

/**
 * @brief (amrl_msgs/VisualizationMsg) Visualization data
 */
static const std::string visualization_topic = "visualization";
}  // namespace webviz_constants