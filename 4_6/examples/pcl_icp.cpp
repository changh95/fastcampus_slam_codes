//
// Original Tutorial Author: shapelim@kaist.ac.kr (임형태)

#include <chrono>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
using namespace std;

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

int main(int argc, char **argv) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);
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

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaxCorrespondenceDistance(2.5);
  icp.setTransformationEpsilon(1E-9);
  icp.setMaximumIterations(1000);

  pcl::PointCloud<pcl::PointXYZ>::Ptr align(new pcl::PointCloud<pcl::PointXYZ>);

  chrono::system_clock::time_point t_start = chrono::system_clock::now();

  icp.setInputSource(src);
  icp.setInputTarget(tgt);
  icp.align(*align);

  chrono::system_clock::time_point t_end = chrono::system_clock::now();
  chrono::duration<double> t_reg = t_end - t_start;
  cout << "Takes " << t_reg.count() << " sec..." << endl;

  Eigen::Matrix4f src2tgt = icp.getFinalTransformation();
  double score = icp.getFitnessScore();

  cout << "ICP output: " << std::endl << src2tgt << endl;
  cout << "Score: " << score << endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_colored(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_colored(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  colorize(*src, *src_colored, {255, 0, 0});
  colorize(*tgt, *tgt_colored, {0, 255, 0});
  colorize(*align, *align_colored, {0, 0, 255});

  pcl::visualization::PCLVisualizer viewer1("Simple Cloud Viewer");
  viewer1.addPointCloud<pcl::PointXYZRGB>(src_colored, "src_viz");
  viewer1.addPointCloud<pcl::PointXYZRGB>(tgt_colored, "tgt_viz");
  viewer1.addPointCloud<pcl::PointXYZRGB>(align_colored, "align_viz");

  viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "src_viz");
  viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tgt_viz");
  viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "align_viz");

  while (!viewer1.wasStopped()) {
    viewer1.spinOnce();
  }
}
