#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>

int main() {
  cv::Mat img_left = cv::imread("left.png", cv::IMREAD_GRAYSCALE);
  cv::Mat img_right = cv::imread("right.png", cv::IMREAD_GRAYSCALE);

  // Detect ORB and match
  auto feature_detector = cv::ORB::create(1000);
  auto bf_matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
  std::vector<cv::DMatch> bf_matches;

  std::vector<cv::KeyPoint> kpts_left, kpts_right;
  cv::Mat desc_left, desc_right;

  feature_detector->detectAndCompute(img_left, cv::noArray(), kpts_left,
                                     desc_left);
  feature_detector->detectAndCompute(img_right, cv::noArray(), kpts_right,
                                     desc_right);

  bf_matcher->match(desc_left, desc_right, bf_matches);
  std::vector<cv::DMatch> good_bf_matches;
  for (const auto &match : bf_matches) {
    if (match.distance < 50) {
      good_bf_matches.push_back(match);
    }
  }

  // Display feature matches
  cv::Mat img_bf;
  cv::drawMatches(img_left, kpts_left, img_right, kpts_right, good_bf_matches,
                  img_bf, cv::Scalar::all(-1), cv::Scalar::all(-1),
                  std::vector<char>(),
                  cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

  cv::imshow("BF Matches", img_bf);
  cv::waitKey(0);

  // Convert matching points to point2f to find E/F matrices
  std::vector<cv::Point2f> pts_left, pts_right;
  for (int i = 0; i < bf_matches.size(); i++) {
    pts_left.push_back(kpts_left[bf_matches[i].queryIdx].pt);
    pts_right.push_back(kpts_right[bf_matches[i].trainIdx].pt);
  }

  // Find fundamental matrix
  // clang-format off
  cv::Mat K = (cv::Mat_<double>(3, 3)
               << 9.799200e+02, 0.000000e+00, 6.900000e+02,
                  0.000000e+00, 9.741183e+02, 2.486443e+02,
                  0.000000e+00, 0.000000e+00, 1.000000e+00);
  // clang-format on

  cv::Mat F =
      cv::findFundamentalMat(pts_left, pts_right, cv::FM_RANSAC, 3, 0.99);
  std::cout << "F = " << std::endl << F << std::endl;

  // Find essential matrix
  cv::Point2d pp(690, 248.6443);
  double focal = 979.92;
  cv::Mat E = cv::findEssentialMat(pts_left, pts_right, focal, pp, cv::RANSAC,
                                   0.999, 1.0);
  std::cout << "E = " << std::endl << E << std::endl;
}