#pragma once
#include <memory>

#include <shinji/config_server.hpp>
#include <shinji/utility.hpp>

// Forward declaration of the global-namespace scancontext manager.
// The full Scancontext.h pollutes the global namespace (using namespace Eigen / nanoflann),
// so it is only included inside the .cpp translation unit (pimpl idiom).
class SCManager;

namespace shinji {

// Estimates the yaw of the T_map_odom transform using a Scan Context descriptor.
//
// Workflow:
//   1. crop the global map (map frame) around the UWB position (`center`),
//   2. translate the crop so the robot sits at the origin,
//   3. build the target Scan Context descriptor,
//   4. build the source descriptor from the query cloud (sensor frame),
//   5. align both descriptors with `distanceBtnScanContext` and convert the
//      column shift into a yaw angle.
class ScanContextMatcher {
public:
  explicit ScanContextMatcher(const ScancontextConfig& config);
  ~ScanContextMatcher();

  // `map` must be expressed in the global map frame.
  void set_globalmap(const pcl::PointCloud<PointT>::Ptr& map);

  // `source` is the aggregated query cloud (sensor/odom frame).
  // `center` is the UWB position (map frame) used to crop the global map.
  ResultT<double> estimate_yaw(const pcl::PointCloud<PointT>::ConstPtr& source, const Eigen::Vector3d& center);

private:
  ScancontextConfig config;
  pcl::PointCloud<PointT>::Ptr globalmap;
  std::unique_ptr<SCManager> sc;
};

}  // namespace shinji
