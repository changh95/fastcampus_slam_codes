//
// Original Tutorial Author: shapelim@kaist.ac.kr (임형태)

#include <chrono>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Eigen/Core>

#include <teaser/ply_io.h>
#include <teaser/registration.h>


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
        file.read((char *) &point.x, sizeof(point.x));
        file.read((char *) &point.y, sizeof(point.y));
        file.read((char *) &point.z, sizeof(point.z));
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

Eigen::Matrix<double, 3, Eigen::Dynamic> convert_to_eigen(const pcl::PointCloud<pcl::PointXYZ> &pc) {
    Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, pc.size());
    for (size_t i = 0; i < pc.size(); ++i) {
        src.col(i) << pc[i].x, pc[i].y, pc[i].z;
    }
    return src;
}


int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_bin_file>" << std::endl;
        return -1;
    }

    std::string path_to_bin_file = argv[1];

    char src_filename[256];
    char tgt_filename[256];

    pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr align(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_colored(
            new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_colored(
            new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::visualization::PCLVisualizer viewer1("Simple Cloud Viewer");
    viewer1.addPointCloud<pcl::PointXYZRGB>(tgt_colored, "tgt_viz");
    viewer1.addPointCloud<pcl::PointXYZRGB>(align_colored, "align_viz");
    viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tgt_viz");
    viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "align_viz");

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance(1.0);
    icp.setTransformationEpsilon(1E-9);
    icp.setMaximumIterations(1000);

    teaser::RobustRegistrationSolver::Params params;
    params.rotation_estimation_algorithm =
            teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::QUATRO;
    teaser::RobustRegistrationSolver solver(params);

    for (int i = 0; i < 4000; ++i) {
        sprintf(src_filename, "%06d.bin", i);
        sprintf(tgt_filename, "%06d.bin", i + 1);
        const std::string src_bin_file = path_to_bin_file + "/" + src_filename;
        const std::string tgt_bin_file = path_to_bin_file + "/" + tgt_filename;
        *src = *load_bin(src_bin_file);
        *tgt = *load_bin(tgt_bin_file);

        pcl::PointCloud<pcl::PointXYZ>::Ptr ds_src(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vox1;
        vox1.setInputCloud(src);
        vox1.setLeafSize(0.5f, 0.5f, 0.5f);
        vox1.filter(*ds_src);

        pcl::PointCloud<pcl::PointXYZ>::Ptr ds_tgt(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vox2;
        vox2.setInputCloud(tgt);
        vox2.setLeafSize(0.5f, 0.5f, 0.5f);
        vox2.filter(*ds_tgt);

        Eigen::Matrix<double, 3, Eigen::Dynamic> ds_src_eigen = convert_to_eigen(*ds_src);
        Eigen::Matrix<double, 3, Eigen::Dynamic> ds_tgt_eigen = convert_to_eigen(*ds_tgt);

        solver.solve(ds_src_eigen, ds_tgt_eigen);
        auto solution = solver.getSolution();

        Eigen::Matrix4f transform;
        transform.block<3, 3>(0, 0) = solution.rotation.cast<float>();
        transform.block<3, 1>(0, 3) = solution.translation.cast<float>();
        transform(3, 3) = 1;

        pcl::transformPointCloud(*src, *align, transform);

        colorize(*tgt, *tgt_colored, {255, 0, 0});
        colorize(*align, *align_colored, {0, 255, 0});

        viewer1.updatePointCloud(tgt_colored, "tgt_viz");
        viewer1.updatePointCloud(align_colored, "align_viz");

        viewer1.spinOnce(1);
    }
}