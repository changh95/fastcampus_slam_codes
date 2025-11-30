#include <iostream>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

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

int main() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  *cloud = *load_bin("./000000.bin");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers(
      new pcl::PointCloud<pcl::PointXYZ>);

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(200);
  sor.setStddevMulThresh(5.0);
  sor.filter(*cloud_inliers);

  sor.setNegative(true);
  sor.filter(*cloud_outliers);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers_colored(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr outliers_colored(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  colorize(*cloud_inliers, *inliers_colored, {0, 255, 0});
  colorize(*cloud_outliers, *outliers_colored, {255, 0, 0});

  pcl::visualization::PCLVisualizer viewer1("Statistical Outlier Removal");
  viewer1.addPointCloud<pcl::PointXYZRGB>(inliers_colored, "inliers");
  viewer1.addPointCloud<pcl::PointXYZRGB>(outliers_colored, "outliers");

  viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "inliers");
  viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "outliers");

  while (!viewer1.wasStopped()) {
    viewer1.spinOnce();
  }

  return 0;
}
