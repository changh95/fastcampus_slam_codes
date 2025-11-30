#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <iostream>

int main() {
  cv::Mat img_left = cv::imread("left.png", cv::IMREAD_COLOR);
  cv::Mat img_right = cv::imread("right.png", cv::IMREAD_COLOR);

  // Detect ORB features
  const auto feature_detector = cv::ORB::create(1000);
  const auto knn_matcher =
      cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
  std::vector<std::vector<cv::DMatch>> knn_matches;

  std::vector<cv::KeyPoint> kpts_left, kpts_right;
  cv::Mat desc_left, desc_right;

  feature_detector->detectAndCompute(img_left, cv::Mat(), kpts_left, desc_left);
  feature_detector->detectAndCompute(img_right, cv::Mat(), kpts_right,
                                     desc_right);

  knn_matcher.knnMatch(desc_left, desc_right, knn_matches, 2);

  constexpr auto ratio_thresh = 0.8;
  std::vector<cv::DMatch> good_knn_matches;
  for (const auto &match : knn_matches) {
    if (match[0].distance < ratio_thresh * match[1].distance) {
      good_knn_matches.push_back(match[0]);
    }
  }

  std::vector<cv::Point2f> pts_left, pts_right;
  for (int i = 0; i < good_knn_matches.size(); i++) {
    pts_left.push_back(kpts_left[good_knn_matches[i].queryIdx].pt);
    pts_right.push_back(kpts_right[good_knn_matches[i].trainIdx].pt);
  }

  cv::UsacParams usac_params = cv::UsacParams();
  usac_params.sampler = cv::SamplingMethod::
      SAMPLING_PROSAC; // SAMPLING_UNIFORM=0,
                                   // SAMPLING_PROGRESSIVE_NAPSAC=1,
                                   // SAMPLING_NAPSAC=2,
                                   // SAMPLING_PROSAC=3
  usac_params.loMethod = cv::LocalOptimMethod::
      LOCAL_OPTIM_INNER_AND_ITER_LO; // LOCAL_OPTIM_NULL=0, LOCAL_OPTIM_INNER_LO=1,
                      // LOCAL_OPTIM_INNER_AND_ITER_LO=2, LOCAL_OPTIM_GC=3,
                      // LOCAL_OPTIM_SIGMA=4
  usac_params.loIterations = 10;
  usac_params.score = cv::ScoreMethod::
      SCORE_METHOD_RANSAC; // SCORE_METHOD_RANSAC=0, SCORE_METHOD_MSAC=1,
                           // SCORE_METHOD_MAGSAC=2, SCORE_METHOD_LMEDS=3
  usac_params.neighborsSearch = cv::NeighborSearchMethod::
      NEIGH_FLANN_KNN; // NEIGH_FLANN_KNN=0, NEIGH_GRID=1, NEIGH_FLANN_RADIUS=2
  usac_params.final_polisher =
      cv::PolishingMethod::MAGSAC; // NONE_POLISHER=0, LSQ_POLISHER=1, MAGSAC=2,
                                   // COV_POLISHER=3
  usac_params.isParallel = true;
  usac_params.confidence = 0.99;
  usac_params.maxIterations = 100;
  usac_params.threshold = 3.0;

  // clang-format off
  cv::Mat rotation = (cv::Mat_<double>(3, 3) <<
                      9.993513e-01, 1.860866e-0, -3.083487e-02,
                      -1.887662e-02, 9.997863e-01, -8.421873e-03,
                      3.067156e-02, 8.998467e-03, 9.994890e-01);
  cv::Mat translation =
      (cv::Mat_<double>(3, 1) << -5.370000e-01, 4.822061e-03, -1.252488e-02);
  cv::Mat skew_translation = (cv::Mat_<double>(3, 3) <<
                      0, -translation.at<double>(2, 0), translation.at<double>(1, 0),
                      translation.at<double>(2, 0), 0, -translation.at<double>(0, 0),
                      -translation.at<double>(1, 0), translation.at<double>(0, 0), 0);
  cv::Mat K_00 = (cv::Mat_<double>(3, 3) <<
      9.842439e+02, 0.000000e+00, 6.900000e+02,
      0.000000e+00, 9.808141e+02, 2.331966e+02,
      0.000000e+00, 0.000000e+00, 1.000000e+00);
  cv::Mat K_01 = (cv::Mat_<double>(3, 3) <<
    9.895267e+02, 0.000000e+00, 7.020000e+02,
    0.000000e+00, 9.878386e+02, 2.455590e+02,
    0.000000e+00, 0.000000e+00, 1.000000e+00);
  cv::Mat gt_fundamental_matrix = K_00.inv() * skew_translation * rotation * K_01.inv();
  // clang-format on

  cv::Mat mask;

  auto start = std::chrono::high_resolution_clock::now();
  cv::Mat F_1 =
      cv::findFundamentalMat(pts_left, pts_right, cv::FM_8POINT, 3, 0.99);
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_seconds_F1 = 1000 * (end - start);

  start = std::chrono::high_resolution_clock::now();
  cv::Mat F_2 =
      cv::findFundamentalMat(pts_left, pts_right, cv::USAC_FM_8PTS, 3, 0.99);
  end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_seconds_F2 = 1000 * (end - start);

  start = std::chrono::high_resolution_clock::now();
  cv::Mat F_3 = cv::findFundamentalMat(pts_left, pts_right, mask, usac_params);
  end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_seconds_F3 = 1000 * (end - start);

  std::cout << "Ground truth fundamental matrix is " << std::endl
            << gt_fundamental_matrix << std::endl;

  std::cout << "OpenCV 8-point fundamental matrix (norm:  "
            << cv::norm(gt_fundamental_matrix - F_1)
            << ") (processing time: " << elapsed_seconds_F1.count() << ")"
            << std::endl
            << F_1 << std::endl;
  std::cout << "USAC 8-point fundamental matrix (norm:  "
            << cv::norm(gt_fundamental_matrix - F_2)
            << ") (processing time: " << elapsed_seconds_F2.count() << ")"
            << std::endl
            << F_2 << std::endl;
  std::cout << "USAC fundamental matrix (norm:  "
            << cv::norm(gt_fundamental_matrix - F_3)
            << ") (processing time: " << elapsed_seconds_F3.count() << ")"
            << std::endl
            << F_3 << std::endl;

  cv::Mat img_matches;
  cv::drawMatches(img_left, kpts_left, img_right, kpts_right, good_knn_matches,
                  img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), mask,
                  cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

  cv::imshow("Matches", img_matches);
  cv::waitKey(0);

  return 0;
}