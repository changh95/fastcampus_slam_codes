#include "DBoW2/DBoW2.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <memory>

int main()
{
  cv::Mat query_img, img_1, img_2, img_3, img_4;

  query_img= cv::imread("data/000024.png", cv::IMREAD_GRAYSCALE);
  img_1 = cv::imread("data/000025.png", cv::IMREAD_GRAYSCALE);
  img_2 = cv::imread("data/001382.png", cv::IMREAD_GRAYSCALE);
  img_3 = cv::imread("data/002105.png", cv::IMREAD_GRAYSCALE);
  img_4 = cv::imread("data/003295.png", cv::IMREAD_GRAYSCALE);

  const auto feature_detector = cv::ORB::create();
  const auto ptrVocabulary = std::make_unique<OrbVocabulary>("data/ORBvoc.yaml");

  std::vector<cv::KeyPoint> query_kpts, kpts1, kpts2, kpts3, kpts4;
  cv::Mat query_desc, desc1, desc2, desc3, desc4;

  feature_detector->detectAndCompute(query_img, cv::Mat(), query_kpts, query_desc);
  feature_detector->detectAndCompute(img_1, cv::Mat(), kpts1, desc1);
  feature_detector->detectAndCompute(img_2, cv::Mat(), kpts2, desc2);
  feature_detector->detectAndCompute(img_3, cv::Mat(), kpts3, desc3);
  feature_detector->detectAndCompute(img_4, cv::Mat(), kpts4, desc4);
//
//
//  dbTable.push_back(fileName);
//  vfeatures.emplace_back(Descriptors());
//  changeStructure(descriptors, vfeatures.back());
}