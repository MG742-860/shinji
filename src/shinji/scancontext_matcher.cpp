#include <shinji/scancontext_matcher.hpp>

#include <cmath>

// Scancontext.h must stay isolated inside this translation unit because it does
// `using namespace Eigen;` / `using namespace nanoflann;` at global scope.
#include <scancontext/Scancontext.h>

namespace shinji {

ScanContextMatcher::ScanContextMatcher(const ScancontextConfig& config) : config(config), sc(std::make_unique<SCManager>()) {
  sc->setMaximumRadius(config.crop_radius);
  sc->setSCdistThres(config.dist_thres);
}

ScanContextMatcher::~ScanContextMatcher() = default;

void ScanContextMatcher::set_globalmap(const pcl::PointCloud<PointT>::Ptr& map) {
  globalmap = map;
}

ResultT<double> ScanContextMatcher::estimate_yaw(const pcl::PointCloud<PointT>::ConstPtr& source, const Eigen::Vector3d& center) {
  if (!globalmap || globalmap->empty()) {
    return ResultT<double>::failure(ErrorCode::INVALID_POINTCLOUD, "empty globalmap for scancontext");
  }
  if (!source || source->empty()) {
    return ResultT<double>::failure(ErrorCode::INVALID_POINTCLOUD, "empty source cloud for scancontext");
  }

  // 将UWB的坐标作为裁剪中心，然后以机器人作为原点
  pcl::PointCloud<PointT>::Ptr target(new pcl::PointCloud<PointT>());
  target->points.reserve(globalmap->size());

  const double radius_sq = config.crop_radius * config.crop_radius;
  for (const auto& p : globalmap->points) {
    const double dx = p.x - center.x();
    const double dy = p.y - center.y();
    if (dx * dx + dy * dy > radius_sq) {
      continue;
    }
    PointT q;
    q.x = static_cast<float>(p.x - center.x());
    q.y = static_cast<float>(p.y - center.y());
    // z方向的高度补偿
    q.z = static_cast<float>(p.z - center.z() - config.lidar_height);
    q.intensity = p.intensity;
    target->points.push_back(q);
  }
  target->width = static_cast<uint32_t>(target->points.size());
  target->height = 1;
  target->is_dense = true;

  if (static_cast<int>(target->size()) < config.min_points) {
    return ResultT<double>::failure(ErrorCode::INVALID_POINTCLOUD, "too few points in cropped globalmap: " + std::to_string(target->size()));
  }

  auto target_down = voxelgrid_sampling(target, config.voxel_resolution);
  auto source_down = voxelgrid_sampling(source, config.voxel_resolution);

  if (target_down->empty() || source_down->empty()) {
    return ResultT<double>::failure(ErrorCode::INVALID_POINTCLOUD, "empty cloud after downsampling");
  }

  Eigen::MatrixXd target_sc = sc->makeScancontext(*target_down);
  Eigen::MatrixXd source_sc = sc->makeScancontext(*source_down);

  auto align_result = sc->distanceBtnScanContext(target_sc, source_sc);
  const double dist = align_result.first;
  const int shift = align_result.second;

  const double yaw = config.yaw_sign * static_cast<double>(shift) * sc->PC_UNIT_SECTORANGLE * M_PI / 180.0;

  if (dist > config.dist_thres) {
    return ResultT<double>::failure(ErrorCode::INVALID_INLIER_FRACTION, "scancontext dist=" + std::to_string(dist) + " exceeds threshold");
  }

  return ResultT<double>::success(yaw);
}

}  // namespace shinji
