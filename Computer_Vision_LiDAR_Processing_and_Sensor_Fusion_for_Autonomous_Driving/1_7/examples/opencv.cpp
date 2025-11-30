#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>

int main() {
  cv::Mat img = cv::imread("data/000024.png");
  cv::imshow("test", img);
  cv::waitKey(0);

  cv::Mat img2(img);
  cv::imshow("test2", img2);
  cv::waitKey(0);

  cv::Mat img3;
  img3 = img;
  cv::imshow("test3", img3);
  cv::waitKey(0);

  cv::Mat img4 = img.clone();
  cv::Mat img5;
  img.copyTo(img5);
  cv::imshow("test4", img4);
  cv::imshow("test5", img5);
  cv::waitKey(0);

  return 0;
}