#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>

int main() {
  cv::Mat img_front = cv::imread("000000.png", cv::IMREAD_COLOR);
  cv::Mat img_front_vis = img_front.clone();
  cv::Mat img_bev;

  cv::Point tl = {500, 200};
  cv::Point tr = {600, 200};
  cv::Point bl = {180, 330};
  cv::Point br = {750, 330};

  cv::circle(img_front_vis, tl, 5, cv::Scalar(0, 0, 255), 10);
  cv::circle(img_front_vis, tr, 5, cv::Scalar(0, 0, 255), 10);
  cv::circle(img_front_vis, bl, 5, cv::Scalar(0, 0, 255), 10);
  cv::circle(img_front_vis, br, 5, cv::Scalar(0, 0, 255), 10);

  cv::imshow("img_front_vis", img_front_vis);
  cv::waitKey(0);

  cv::Point target_tl = {0, 0};
  cv::Point target_tr = {640, 0};
  cv::Point target_bl = {0, 480};
  cv::Point target_br = {640, 480};

  std::vector<cv::Point2f> src_points = {tl, tr, bl, br};
  std::vector<cv::Point2f> dst_points = {target_tl, target_tr, target_bl,
                                         target_br};

  cv::Mat M = cv::getPerspectiveTransform(src_points, dst_points);

  cv::warpPerspective(img_front, img_bev, M, cv::Size(640, 480));

  cv::imshow("img_bev", img_bev);
  cv::waitKey(0);
}