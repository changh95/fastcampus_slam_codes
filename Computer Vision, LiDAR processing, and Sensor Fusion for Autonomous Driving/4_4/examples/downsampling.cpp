// 출처: 임형태님 pcl_tutorial 코드
// https://github.com/LimHyungTae/pcl_tutorial

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>

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
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  *cloud = *load_bin("./000000.bin");

  pcl::PointCloud<pcl::PointXYZ>::Ptr ds_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::VoxelGrid<pcl::PointXYZ> vox;
  vox.setInputCloud(cloud);
  vox.setLeafSize(0.5f, 0.5f, 0.5f);
  vox.filter(*ds_cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_cloud_colored(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  colorize(*cloud, *cloud_colored, {0, 255, 0});
  colorize(*ds_cloud, *ds_cloud_colored, {0, 255, 0});

  pcl::visualization::PCLVisualizer viewer1("Simple Cloud Viewer");
  viewer1.addPointCloud<pcl::PointXYZRGB>(cloud_colored, "cloud");

  pcl::visualization::PCLVisualizer viewer2("Downsampled Cloud Viewer");
  viewer2.addPointCloud<pcl::PointXYZRGB>(ds_cloud_colored, "cloud");

  viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
  viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

  while (!viewer1.wasStopped() || !viewer2.wasStopped()) {
    viewer1.spinOnce();
    viewer2.spinOnce();
  }
}
