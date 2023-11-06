//
// Original Tutorial Author: shapelim@kaist.ac.kr (임형태)

#include <chrono>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/incremental_registration.h>
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

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <path_to_bin_file>" << std::endl;
    return -1;
  }

  std::string path_to_bin_file = argv[1];

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr icp(
      new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>);
  icp->setMaxCorrespondenceDistance(10.0);
  icp->setTransformationEpsilon(0.001);
  icp->setMaximumIterations(1000);

  pcl::registration::IncrementalRegistration<pcl::PointXYZ> iicp;
  iicp.setRegistration(icp);

  pcl::PointCloud<pcl::PointXYZ>::Ptr poses(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_colored(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  char filename[256];

  pcl::visualization::PCLVisualizer viewer1("Simple Cloud Viewer");
  viewer1.addPointCloud<pcl::PointXYZRGB>(cloud_colored, "cloud");
  viewer1.addPointCloud<pcl::PointXYZRGB>(align_colored, "align");
  viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
  viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "align");

  for (int i = 0; i < 4000; ++i) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    sprintf(filename, "%06d.bin", i);
    const std::string bin_file = path_to_bin_file + "/" + filename;
    *cloud = *load_bin(bin_file);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ds_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vox1;
    vox1.setInputCloud(cloud);
    vox1.setLeafSize(0.5f, 0.5f, 0.5f);
    vox1.filter(*ds_cloud);

    iicp.registerCloud(ds_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr align(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *align, iicp.getAbsoluteTransform());

    std::cout << "ICP result: " << std::endl
              << iicp.getAbsoluteTransform() << std::endl;

    colorize(*cloud, *cloud_colored, {0, 255, 0});
    colorize(*align, *align_colored, {255, 0, 0});
    viewer1.updatePointCloud(cloud_colored, "cloud");
    viewer1.updatePointCloud(align_colored, "align");

    viewer1.spinOnce(1);
  }
}