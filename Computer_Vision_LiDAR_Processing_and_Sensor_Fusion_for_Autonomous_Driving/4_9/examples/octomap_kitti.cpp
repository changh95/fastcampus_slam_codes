//
// Original Tutorial Author: shapelim@kaist.ac.kr (임형태)

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <octomap/octomap.h>

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
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  *cloud = *load_bin("000000.bin");

  octomap::OcTree tree(0.1);

  octomap::Pointcloud octo_cloud;
  for (const auto &pt : cloud->points) {
    octo_cloud.push_back(pt.x, pt.y, pt.z);
  }

  tree.insertPointCloud(octo_cloud, octomap::point3d(0, 0, 0));

  tree.updateInnerOccupancy();
  tree.writeBinary("./results/000000.bt");
}