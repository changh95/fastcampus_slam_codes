#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

#include <ctime>
#include <iostream>
#include <vector>


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

  pcl::PointCloud<pcl::PointXYZ>::Ptr radius_search(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr knn_search(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  kdtree.setInputCloud(cloud);
  pcl::PointXYZ searchPoint;
  searchPoint.x = 8.0;
  searchPoint.y = 10.0;
  searchPoint.z = 0.1;
  // K nearest neighbor search
  int K = 100;
  std::vector<int> pointIdxKNNSearch(K);
  std::vector<float> pointKNNSquaredDistance(K);
  std::cout << "K nearest neighbor search at (" << searchPoint.x << " "
            << searchPoint.y << " " << searchPoint.z << ") with K=" << K
            << std::endl;
  if (kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch,
                            pointKNNSquaredDistance) > 0) {
    for (std::size_t i = 0; i < pointIdxKNNSearch.size(); ++i)
    {
      std::cout << "    " << (*cloud)[pointIdxKNNSearch[i]].x << " "
                << (*cloud)[pointIdxKNNSearch[i]].y << " "
                << (*cloud)[pointIdxKNNSearch[i]].z
                << " (squared distance: " << pointKNNSquaredDistance[i] << ")"
                << std::endl;
      knn_search->push_back((*cloud)[pointIdxKNNSearch[i]]);
    }
  }

  // Neighbors within radius search
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  float radius = 5.0;
  std::cout << "Neighbors within radius search at (" << searchPoint.x << " "
            << searchPoint.y << " " << searchPoint.z
            << ") with radius=" << radius << std::endl;
  if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch,
                          pointRadiusSquaredDistance) > 0) {
    for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
    {
      std::cout << "    " << (*cloud)[pointIdxRadiusSearch[i]].x << " "
                << (*cloud)[pointIdxRadiusSearch[i]].y << " "
                << (*cloud)[pointIdxRadiusSearch[i]].z
                << " (squared distance: " << pointRadiusSquaredDistance[i]
                << ")" << std::endl;
      radius_search->push_back((*cloud)[pointIdxRadiusSearch[i]]);
    }

  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr radius_search_c(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr knn_search_c(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  colorize(*radius_search, *radius_search_c, {255, 0, 0});
  colorize(*knn_search, *knn_search_c, {0, 255, 0});

  pcl::visualization::PCLVisualizer viewer1("Input point cloud");
  viewer1.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
  viewer1.addPointCloud<pcl::PointXYZRGB>(radius_search_c, "radius_search");
  viewer1.addPointCloud<pcl::PointXYZRGB>(knn_search_c, "knn_search");

  viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
  viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "radius_search");
  viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "knn_search");

  while (!viewer1.wasStopped()) {
    viewer1.spinOnce();
  }

  return 0;
}
