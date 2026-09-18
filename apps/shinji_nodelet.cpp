#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <optional>
#include <deque>

#include <shinji/Query.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

#include <shinji/shinji.hpp>
#include <shinji/scancontext_matcher.hpp>


namespace shinji {
class ShinjiNodelet : public nodelet::Nodelet {
public:
  ShinjiNodelet() = default;
  virtual ~ShinjiNodelet() override;
  void onInit() override;

private:
  void setupConfigurations();
  void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void publishTransform(const ros::Time& stamp, const Eigen::Isometry3d& pose);
  bool serviceCallback(shinji::Query::Request& request, shinji::Query::Response& response);

private:
  ros::NodeHandle pnh;
  ros::ServiceServer server;

  std::string cloud_topic;
  std::string globalmap_frame;
  std::string lidar_odom_frame;
  std::string lidar_base_frame;
  bool publish_map2odom_tf;
  bool transformation_needed;
  double tf_listen_timeout;

  geometry_msgs::TransformStamped tf_msg;
  tf2_ros::Buffer tf_buffer{};
  tf2_ros::TransformListener tf_listener{tf_buffer};
  tf2_ros::TransformBroadcaster tf_broadcaster;

  ros::Subscriber pointcloud_subscriber;
  ros::Publisher transform_publisher;

  std::atomic_bool tf_thread_running{false};
  std::thread tf_thread;
  std::mutex tf_mutex;

  std::unique_ptr<shinji::Shinji> shinji;

  // SC位姿先验估计
  std::unique_ptr<shinji::ScanContextMatcher> sc_matcher;

  std::mutex query_buffer_mutex;
  std::deque<pcl::PointCloud<PointT>::Ptr> query_buffer;
  size_t query_buffer_capacity{8};
};


ShinjiNodelet::~ShinjiNodelet() {
  tf_thread_running.store(false);
  if (tf_thread.joinable()) {
    tf_thread.join();
  }
}

void ShinjiNodelet::setupConfigurations() {
  const auto& config = shinji->config_server();

  cloud_topic = config->ros.cloud_topic;
  globalmap_frame = config->ros.globalmap_frame;
  lidar_odom_frame = config->ros.lidar_odom_frame;
  lidar_base_frame = config->ros.lidar_base_frame;
  publish_map2odom_tf = config->ros.publish_map2odom_tf;
  transformation_needed = config->common.transformation_needed;
  tf_listen_timeout = config->ros.tf_listen_timeout;

  tf_msg.header.frame_id = globalmap_frame;
  tf_msg.child_frame_id = lidar_odom_frame;
  tf_msg.transform.rotation.w = 1.0;
}

void ShinjiNodelet::onInit() {
  pnh = getPrivateNodeHandle();

  const std::string config_file = pnh.param<std::string>("config_file", "");
  shinji = std::make_unique<shinji::Shinji>();
  shinji->initialize(config_file);

  setupConfigurations();

  const auto& config = shinji->config_server();
  if (config->scancontext.enable) {
    sc_matcher = std::make_unique<shinji::ScanContextMatcher>(config->scancontext);

    const auto gms = shinji->globalmap_server();
    const auto& map = (config->scancontext.map_type == "origin") ? gms->origin() : gms->filtered();
    if (!map || map->empty()) {
      NODELET_ERROR("Scancontext is enabled but the globalmap \"%s\" could not be loaded!", config->scancontext.map_type.c_str());
      sc_matcher.reset();
    } else {
      sc_matcher->set_globalmap(map);
      query_buffer_capacity = static_cast<size_t>(config->gicp.source_frames);
      NODELET_INFO("Scancontext enabled, cropping globalmap \"%s\" within %.1f m (points: %zu)", config->scancontext.map_type.c_str(),
                   config->scancontext.crop_radius, map->size());
    }
  }

  pointcloud_subscriber = pnh.subscribe(cloud_topic, 5, &ShinjiNodelet::pointcloudCallback, this);

  transform_publisher = pnh.advertise<geometry_msgs::TransformStamped>("/shinji/result", 5, false);
  server = pnh.advertiseService("/shinji/query", &ShinjiNodelet::serviceCallback, this);

  if (publish_map2odom_tf) {
    tf_thread_running.store(true);
    tf_thread = std::thread([this]() {
      ros::Rate rate(10.0);  // 10 Hz
      while (tf_thread_running.load() && ros::ok()) {
        {
          std::lock_guard<std::mutex> lock(tf_mutex);
          tf_msg.header.stamp = ros::Time::now();
          tf_broadcaster.sendTransform(tf_msg);
        }
        rate.sleep();
      }
    });
  }
}

void ShinjiNodelet::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};

  if (transformation_needed) {
    try {
      const auto& tf = tf_buffer.lookupTransform(lidar_odom_frame, lidar_base_frame, msg->header.stamp, ros::Duration(tf_listen_timeout));
      pose.translation() << tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z;
      pose.rotate(Eigen::Quaterniond(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z));
    } catch (const tf2::TransformException& ex) {
      NODELET_WARN("Transform lookup failed: %s", ex.what());
      return;
    }
  }

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  pcl::fromROSMsg(*msg, *cloud);

  shinji->insert_frame(pose, cloud);

  // Keep the raw (odom/sensor frame) cloud for the scancontext descriptor.
  if (sc_matcher) {
    std::lock_guard<std::mutex> lock(query_buffer_mutex);
    query_buffer.push_back(cloud);
    while (query_buffer.size() > query_buffer_capacity) {
      query_buffer.pop_front();
    }
  }

  return;
}


void ShinjiNodelet::publishTransform(const ros::Time& stamp, const Eigen::Isometry3d& pose) {
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = globalmap_frame;
  msg.child_frame_id = lidar_odom_frame;

  msg.transform.translation.x = pose.translation().x();
  msg.transform.translation.y = pose.translation().y();
  msg.transform.translation.z = pose.translation().z();

  Eigen::Quaterniond quat(pose.rotation());
  msg.transform.rotation.x = quat.x();
  msg.transform.rotation.y = quat.y();
  msg.transform.rotation.z = quat.z();
  msg.transform.rotation.w = quat.w();

  transform_publisher.publish(msg);

  if (publish_map2odom_tf) {
    std::unique_lock<std::mutex> lock(tf_mutex, std::try_to_lock);
    tf_msg = msg;
  }
}

bool ShinjiNodelet::serviceCallback(shinji::Query::Request& req, shinji::Query::Response& resp) {
  resp.success = true;

  std::optional<Eigen::Isometry3d> guess;
  if (req.use_guess) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() << req.guess_pose.position.x, req.guess_pose.position.y, req.guess_pose.position.z;

    // 启用SC->姿态估计
    if (sc_matcher) {
      pcl::PointCloud<PointT>::Ptr query_cloud(new pcl::PointCloud<PointT>());
      {
        std::lock_guard<std::mutex> lock(query_buffer_mutex);
        for (const auto& c : query_buffer) {
          if (c && !c->empty()) {
            *query_cloud += *c;
          }
        }
      }

      if (query_cloud->empty()) {
        NODELET_WARN("Scancontext: query buffer is empty, only UWB translation will be used.");
      } else {
        const Eigen::Vector3d center = pose.translation();
        auto yaw_result = sc_matcher->estimate_yaw(query_cloud, center);
        if (yaw_result) {
          pose.linear() = Eigen::AngleAxisd(yaw_result.data, Eigen::Vector3d::UnitZ()).toRotationMatrix();
          NODELET_INFO("Scancontext yaw: %.3f rad (%.1f deg)", yaw_result.data, yaw_result.data * 180.0 / M_PI);
        } else {
          NODELET_WARN("Scancontext yaw estimation failed: %s", yaw_result.message.c_str());
        }
      }
    }

    guess = pose;
  }

  std::thread([this, guess]() {
    const auto result = shinji->try_request(guess);

    if (result) {
      auto time = ros::Time::now();
      publishTransform(time, result.data.pose);
    }
  }).detach();
  return true;
}


}  // namespace shinji

PLUGINLIB_EXPORT_CLASS(shinji::ShinjiNodelet, nodelet::Nodelet)