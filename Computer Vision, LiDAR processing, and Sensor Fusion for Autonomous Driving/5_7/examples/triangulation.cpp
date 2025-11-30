#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <unistd.h>

#include <filesystem>
#include <iostream>

int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << "left_image_path right_image_path num_frames " << std::endl;
    return 0;
  }

  auto left_image_path = std::filesystem::path(argv[1]);
  auto right_image_path = std::filesystem::path(argv[2]);
  const int num_frames = std::atoi(argv[3]);

  std::vector<std::string> left_image_filenames, right_image_filenames;
  left_image_filenames.reserve(5000);
  right_image_filenames.reserve(5000);

  for (const auto &entry :
       std::filesystem::directory_iterator(left_image_path)) {
    left_image_filenames.push_back(entry.path());
  }

  for (const auto &entry :
       std::filesystem::directory_iterator(right_image_path)) {
    right_image_filenames.push_back(entry.path());
  }

  std::sort(left_image_filenames.begin(), left_image_filenames.end());
  std::sort(right_image_filenames.begin(), right_image_filenames.end());

  left_image_filenames.resize(num_frames);
  right_image_filenames.resize(num_frames);

  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Point cloud Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1280, 720, 500, 500, 512, 389, 0.0001, 1000),
      pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

  auto handler = std::make_unique<pangolin::Handler3D>(s_cam);
  pangolin::View &d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, 0.0, 1.0, -1280.0f / 720.0f)
                              .SetHandler(handler.get());

  cv::Mat img_left, img_right;
  std::vector<cv::KeyPoint> kpts_left, kpts_right;
  cv::Mat desc_left, desc_right;

  // Make ORB detector
  auto feature_detector = cv::ORB::create(1000);
  auto bf_matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
  std::vector<cv::DMatch> bf_matches;

  // Projection matrix for cam0 and cam1 (from KITTI calibration)
  // clang-format off
    cv::Mat P0 = (cv::Mat_<float>(3, 4) <<
                                        7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02, 0.000000000000e+00,
            0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02, 0.000000000000e+00,
            0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00);
    cv::Mat P1 = (cv::Mat_<float>(3, 4) <<
                                        7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02, -3.861448000000e+02,
            0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02, 0.000000000000e+00,
            0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00);
  // clang-format on

  int i = 0;
  while (!pangolin::ShouldQuit()) {
    if (i == num_frames) {
      break;
    }

    img_left = cv::imread(left_image_filenames[i], cv::IMREAD_GRAYSCALE);
    img_right = cv::imread(right_image_filenames[i], cv::IMREAD_GRAYSCALE);

    feature_detector->detectAndCompute(img_left, cv::Mat(), kpts_left,
                                       desc_left);
    feature_detector->detectAndCompute(img_right, cv::Mat(), kpts_right,
                                       desc_right);

    if (desc_left.empty() || desc_right.empty()) {
      continue;
    }

    // Brute-force matching
    bf_matcher->match(desc_left, desc_right, bf_matches);
    std::vector<cv::DMatch> good_bf_matches;
    for (const auto &match : bf_matches) {
      if (match.distance < 40) {
        good_bf_matches.push_back(match);
      }
    }

    // Draw Brute-force matches
    cv::Mat img_bf;
    cv::drawMatches(img_left, kpts_left, img_right, kpts_right, good_bf_matches,
                    img_bf, cv::Scalar::all(-1), cv::Scalar::all(-1),
                    std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    // Show matches
    cv::imshow("BF Matches", img_bf);
    cv::waitKey(30);

    // Convert keypoints into Point2f
    std::vector<cv::Point2f> left_pts, right_pts;
    for (const auto &match : good_bf_matches) {
      left_pts.push_back(kpts_left[match.queryIdx].pt);
      right_pts.push_back(kpts_right[match.trainIdx].pt);
    }

    // Triangulate points
    cv::Mat pts_4d;
    cv::triangulatePoints(P0, P1, left_pts, right_pts, pts_4d);

    // 3D visualize triangulated points
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glPointSize(3);

    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);
    for (int i = 0; i < pts_4d.cols; i++) {
      cv::Mat x = pts_4d.col(i);
      x /= x.at<float>(3, 0);
      glVertex3d(x.at<float>(0, 0), x.at<float>(1, 0), x.at<float>(2, 0));
      //            std::cout << x.at<float>(0, 0) << " " << x.at<float>(1, 0)
      //            << " "
      //                      << x.at<float>(2, 0) << std::endl;
    }
    glEnd();

    pangolin::FinishFrame();
    usleep(5000); // sleep 5 ms

    // Loop increment
    i++;
  }
}
