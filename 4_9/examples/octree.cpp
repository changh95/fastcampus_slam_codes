//
// Original Tutorial Author: https://scm_mos.gitlab.io/algorithm/icp/

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

pcl::PointCloud<pcl::PointXYZ>::ConstPtr load_bin(const std::string &filename) {
  std::ifstream file(filename, std::ios::binary);
  if (!file) {
    std::cerr << "Error: failed to load " << filename << std::endl;
    return nullptr;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointXYZ point;
  while (file) {
    // Read x, y and z coordinates
    file.read((char *)&point.x, sizeof(point.x));
    file.read((char *)&point.y, sizeof(point.y));
    file.read((char *)&point.z, sizeof(point.z));
    // Ignore the intensity value if there is one
    file.ignore(sizeof(float));

    if (file) // If the reads above were successful
      cloud->push_back(point);
  }

  return cloud;
}

void colorize(const pcl::PointCloud<pcl::PointXYZ> &pc,
              pcl::PointCloud<pcl::PointXYZRGB> &pc_colored,
              const std::vector<int> &color) {
  int N = pc.points.size();
  pc_colored.clear();

  pcl::PointXYZRGB pt_rgb;
  for (int i = 0; i < N; ++i) {
    const auto &pt = pc.points[i];
    pt_rgb.x = pt.x;
    pt_rgb.y = pt.y;
    pt_rgb.z = pt.z;
    pt_rgb.r = color[0];
    pt_rgb.g = color[1];
    pt_rgb.b = color[2];
    pc_colored.points.emplace_back(pt_rgb);
  }
}

Eigen::Isometry3d icp_point2point(
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
        &reference,
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
        &current) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  if (reference.size() != current.size()) {
    return transform;
  }
  int N = reference.size();
  Eigen::Map<Eigen::Matrix3Xd> ps(&reference[0].x(), 3,
                                  N); // maps vector<Vector3d>
  Eigen::Map<Eigen::Matrix3Xd> qs(&current[0].x(), 3,
                                  N); // to Matrix3Nf columnwise
  Eigen::Vector3d p_dash = ps.rowwise().mean();
  Eigen::Vector3d q_dash = qs.rowwise().mean();
  Eigen::Matrix3Xd ps_centered = ps.colwise() - p_dash;
  Eigen::Matrix3Xd qs_centered = qs.colwise() - q_dash;
  Eigen::Matrix3d K = qs_centered * ps_centered.transpose();
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(K, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
  if (R.determinant() < 0) {
    R.col(2) *= -1;
  }
  transform.linear() = R;
  transform.translation() = q_dash - R * p_dash;
  return transform;
}

int main(int argc, const char **argv) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr align(new pcl::PointCloud<pcl::PointXYZ>);
  *src = *load_bin("000000.bin");

  /** Test를 위해 z축으로 10도 회전 및 앞으로 2m 전진시킨 target을 만듦 */
  Eigen::Matrix4f tf;
  // clang-format off
  tf << 0.9848077, -0.1736482, 0.0, 2.0,
      0.1736482, 0.9848077, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0;
  // clang-format on
  pcl::transformPointCloud(*src, *tgt, tf);

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      reference, current;

  /** reference와 current에 각각 pointcloud의 point를 저장 */
  for (int pos = 0; pos < src->size(); pos++) {
    Eigen::Vector3d point;
    point.x() = src->at(pos).x;
    point.y() = src->at(pos).y;
    point.z() = src->at(pos).z;
    reference.push_back(point);
  }
  for (int pos = 0; pos < tgt->size(); pos++) {
    Eigen::Vector3d point;
    point.x() = tgt->at(pos).x;
    point.y() = tgt->at(pos).y;
    point.z() = tgt->at(pos).z;
    current.push_back(point);
  }

  {
    auto estimate = icp_point2point(reference, current);
    std::cout << "ground truth: " << std::endl << tf << std::endl;
    std::cout << "estimate: " << std::endl << estimate.matrix() << std::endl;
  }
  return 0;
}