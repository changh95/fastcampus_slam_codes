#include "DBoW2/DBoW2.h"
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

int main() {
  std::vector<cv::Mat> images(5);

  images[0] = cv::imread("data/000024.png", cv::IMREAD_GRAYSCALE);
  images[1] = cv::imread("data/000025.png", cv::IMREAD_GRAYSCALE);
  images[2] = cv::imread("data/001382.png", cv::IMREAD_GRAYSCALE);
  images[3] = cv::imread("data/002105.png", cv::IMREAD_GRAYSCALE);
  images[4] = cv::imread("data/003295.png", cv::IMREAD_GRAYSCALE);

  const auto feature_detector = cv::ORB::create();
  std::vector<std::vector<cv::Mat>> v_descriptors;

  for (const auto &img : images) {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    feature_detector->detectAndCompute(img, cv::Mat(), keypoints, descriptors);
    v_descriptors.push_back(std::vector<cv::Mat>());
    v_descriptors.back().resize(descriptors.rows);

    for (int i = 0; i < descriptors.rows; i++) {
      v_descriptors.back()[i] = descriptors.row(i);
    }
  }

  // Vocabulary creation
  const int k = 9;
  const int L = 3;
  const DBoW2::WeightingType weight = DBoW2::TF_IDF;
  const DBoW2::ScoringType score = DBoW2::L1_NORM;

  DBoW2::OrbVocabulary voc(k, L, weight, score);
  voc.create(v_descriptors);
  voc.save("vocabulary.yml.gz");

  // Global feature database creation
  DBoW2::OrbDatabase db(voc, false, 0);

  for (int i = 0; i < images.size(); i++) {
    db.add(v_descriptors[i]);
  }

  // Query
  for (int i = 0; i < images.size(); i++) {
    DBoW2::QueryResults ret;
    db.query(v_descriptors[i], ret, 4);

    std::cout << "Searching for image " << i << ". " << ret << std::endl;
  }

  //  // create vocabulary
  //  DBoW2::OrbVocabulary vocab;
  //  vocab.create(descriptors);
  //  cout << "vocabulary info: " << vocab << endl;
  //  vocab.save("vocabulary.yml.gz");
  //  cout << "done" << endl;
}