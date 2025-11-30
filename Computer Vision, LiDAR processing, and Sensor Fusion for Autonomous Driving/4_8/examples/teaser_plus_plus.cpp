// MIT License
//
// Copyright (c) 2020 Massachusetts Institute of Technology
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  OFTWARE.

// An example showing TEASER++ registration with the Stanford bunny model
#include <chrono>
#include <iostream>
#include <random>

#include <Eigen/Core>

#include <teaser/ply_io.h>
#include <teaser/registration.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>

// Macro constants for generating noise and outliers
#define NOISE_BOUND 0.001
#define N_OUTLIERS 1700
#define OUTLIER_TRANSLATION_LB 5
#define OUTLIER_TRANSLATION_UB 10

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

inline double getAngularError(Eigen::Matrix3d R_exp, Eigen::Matrix3d R_est) {
    return std::abs(std::acos(fmin(fmax(((R_exp.transpose() * R_est).trace() - 1) / 2, -1.0), 1.0)));
}

void addNoiseAndOutliers(Eigen::Matrix<double, 3, Eigen::Dynamic> &tgt) {
    // Add uniform noise
    Eigen::Matrix<double, 3, Eigen::Dynamic> noise =
            Eigen::Matrix<double, 3, Eigen::Dynamic>::Random(3, tgt.cols()) * NOISE_BOUND / 2;
    tgt = tgt + noise;

    // Add outliers
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis2(0, tgt.cols() - 1); // pos of outliers
    std::uniform_int_distribution<> dis3(OUTLIER_TRANSLATION_LB,
                                         OUTLIER_TRANSLATION_UB); // random translation
    std::vector<bool> expected_outlier_mask(tgt.cols(), false);
    for (int i = 0; i < N_OUTLIERS; ++i) {
        int c_outlier_idx = dis2(gen);
        assert(c_outlier_idx < expected_outlier_mask.size());
        expected_outlier_mask[c_outlier_idx] = true;
        tgt.col(c_outlier_idx).array() += dis3(gen); // random translation
    }
}

int main() {
    // Load the .ply file
    teaser::PLYReader reader;
    teaser::PointCloud src_cloud;
    auto status = reader.read("./bun_zipper_res3.ply", src_cloud);
    int N = src_cloud.size();

    // Convert the point cloud to Eigen
    Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, N);
    for (size_t i = 0; i < N; ++i) {
        src.col(i) << src_cloud[i].x, src_cloud[i].y, src_cloud[i].z;
    }

    // Homogeneous coordinates
    Eigen::Matrix<double, 4, Eigen::Dynamic> src_h;
    src_h.resize(4, src.cols());
    src_h.topRows(3) = src;
    src_h.bottomRows(1) = Eigen::Matrix<double, 1, Eigen::Dynamic>::Ones(N);

    // Apply an arbitrary SE(3) transformation
    Eigen::Matrix4d T;
    // clang-format off
    T << 9.96926560e-01, 6.68735757e-02, -4.06664421e-02, -1.15576939e-01,
            -6.61289946e-02, 9.97617877e-01, 1.94008687e-02, -3.87705398e-02,
            4.18675510e-02, -1.66517807e-02, 9.98977765e-01, 1.14874890e-01,
            0, 0, 0, 1;
    // clang-format on

    // Apply transformation
    Eigen::Matrix<double, 4, Eigen::Dynamic> tgt_h = T * src_h;
    Eigen::Matrix<double, 3, Eigen::Dynamic> tgt = tgt_h.topRows(3);

    // Add some noise & outliers
    addNoiseAndOutliers(tgt);

    // Run TEASER++ registration
    // Prepare solver parameters
    teaser::RobustRegistrationSolver::Params params;
    params.noise_bound = NOISE_BOUND;
    params.cbar2 = 1;
    params.estimate_scaling = false;
    params.rotation_max_iterations = 100;
    params.rotation_gnc_factor = 1.4;
    params.rotation_estimation_algorithm =
            teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    params.rotation_cost_threshold = 0.005;

    // Solve with TEASER++
    teaser::RobustRegistrationSolver solver(params);
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    solver.solve(src, tgt);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    auto solution = solver.getSolution();

    // Compare results
    std::cout << "=====================================" << std::endl;
    std::cout << "          TEASER++ Results           " << std::endl;
    std::cout << "=====================================" << std::endl;
    std::cout << "Expected rotation: " << std::endl;
    std::cout << T.topLeftCorner(3, 3) << std::endl;
    std::cout << "Estimated rotation: " << std::endl;
    std::cout << solution.rotation << std::endl;
    std::cout << "Error (rad): " << getAngularError(T.topLeftCorner(3, 3), solution.rotation)
              << std::endl;
    std::cout << std::endl;
    std::cout << "Expected translation: " << std::endl;
    std::cout << T.topRightCorner(3, 1) << std::endl;
    std::cout << "Estimated translation: " << std::endl;
    std::cout << solution.translation << std::endl;
    std::cout << "Error (m): " << (T.topRightCorner(3, 1) - solution.translation).norm() << std::endl;
    std::cout << std::endl;
    std::cout << "Number of correspondences: " << N << std::endl;
    std::cout << "Number of outliers: " << N_OUTLIERS << std::endl;
    std::cout << "Time taken (s): "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() /
                 1000000.0
              << std::endl;

    //----- PCL ---

    pcl::PointCloud<pcl::PointXYZ>::Ptr src_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr align_pcl(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPLYFile("./bun_zipper_res3.ply", *src_pcl);

    Eigen::Matrix4f transform;

    transform.block<3, 3>(0, 0) = solution.rotation.cast<float>();
    transform.block<3, 1>(0, 3) = solution.translation.cast<float>();
    transform(3, 3) = 1;

    pcl::transformPointCloud(*src_pcl, *tgt_pcl, T);
    pcl::transformPointCloud(*src_pcl, *align_pcl, transform);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(
            new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_colored(
            new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_colored(
            new pcl::PointCloud<pcl::PointXYZRGB>);

    colorize(*src_pcl, *src_colored, {255, 0, 0});
    colorize(*tgt_pcl, *tgt_colored, {0, 255, 0});
    colorize(*align_pcl, *align_colored, {0, 0, 255});

    pcl::visualization::PCLVisualizer viewer("TEASER++ demo");
    viewer.addPointCloud<pcl::PointXYZRGB>(src_colored, "src_viz");
    viewer.addPointCloud<pcl::PointXYZRGB>(tgt_colored, "tgt_viz");
    viewer.addPointCloud<pcl::PointXYZRGB>(align_colored, "align_viz");

    viewer.spinOnce();
}