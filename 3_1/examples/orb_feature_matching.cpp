#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>

#include <filesystem>

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

  cv::Mat img_left, img_right;
  std::vector<cv::KeyPoint> kpts_left, kpts_right;
  cv::Mat desc_left, desc_right;

  // Make ORB detector
  auto feature_detector = cv::ORB::create(1000);
  auto bf_matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
  auto knn_matcher =
      cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
  std::vector<cv::DMatch> bf_matches;
  std::vector<std::vector<cv::DMatch>> knn_matches;

  for (int i = 0; i < num_frames; i++) {
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
      if (match.distance < 50) {
        good_bf_matches.push_back(match);
      }
    }

    // KNN matching
    knn_matcher.knnMatch(desc_left, desc_right, knn_matches, 2);

    constexpr auto ratio_thresh = 0.8;
    std::vector<cv::DMatch> good_knn_matches;
    for (const auto &match : knn_matches) {
      if (match[0].distance < ratio_thresh * match[1].distance) {
        good_knn_matches.push_back(match[0]);
      }
    }

    // Draw Brute-force matches
    cv::Mat img_bf;
    cv::drawMatches(img_left, kpts_left, img_right, kpts_right, good_bf_matches,
                    img_bf, cv::Scalar::all(-1), cv::Scalar::all(-1),
                    std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    // Draw KNN matches
    cv::Mat img_knn;
    cv::drawMatches(img_left, kpts_left, img_right, kpts_right,
                    good_knn_matches, img_knn, cv::Scalar::all(-1),
                    cv::Scalar::all(-1), std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    // Show matches
    cv::imshow("BF Matches", img_bf);
    cv::imshow("KNN Matches", img_knn);
    cv::waitKey(0);
  }
}