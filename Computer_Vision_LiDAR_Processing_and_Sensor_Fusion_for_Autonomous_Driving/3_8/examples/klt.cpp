#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>

#include <filesystem>

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "image_path num_frames " << std::endl;
    return 0;
  }

  auto image_path = std::filesystem::path(argv[1]);
  const int num_frames = std::atoi(argv[2]);

  std::vector<std::string> image_filenames;
  image_filenames.reserve(5000);

  for (const auto &entry : std::filesystem::directory_iterator(image_path)) {
    image_filenames.push_back(entry.path());
  }

  std::sort(image_filenames.begin(), image_filenames.end());
  image_filenames.resize(num_frames);

  cv::Mat img, img_next;
  std::vector<cv::Point2f> kpts, kpts_next;
  cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
  std::vector<uchar> status;
  std::vector<float> err;

  for (int i = 0; i < num_frames-1; i++) {
    img = cv::imread(image_filenames[i], cv::IMREAD_GRAYSCALE);
    img_next = cv::imread(image_filenames[i+1], cv::IMREAD_GRAYSCALE);
    cv::Mat img_vis = img_next.clone();
    cv::cvtColor(img_vis, img_vis, cv::COLOR_GRAY2BGR);

    if (kpts.size() < 50)
    {
      cv::goodFeaturesToTrack(img, kpts, 1000, 0.01, 15);
    }

    cv::calcOpticalFlowPyrLK(img, img_next, kpts, kpts_next, status, err, cv::Size(21,21), 2, criteria);

    // Visualize optical flow
    for (const auto& corner: kpts_next)
    {
      cv::circle(img_vis, corner, 4, cv::Scalar(0, 0, 255));
    }

    cv::imshow("img_vis", img_vis);
    cv::waitKey(33);

    // Organize data
    int indexCorrection = 0;
    for( int i=0; i<status.size(); i++)
    {  cv::Point2f pt = kpts_next.at(i- indexCorrection);
      if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
        if((pt.x<0)||(pt.y<0))	{
          status.at(i) = 0;
        }
        kpts.erase (kpts.begin() + (i - indexCorrection));
        kpts_next.erase (kpts_next.begin() + (i - indexCorrection));
        indexCorrection++;
      }

    }

    kpts = kpts_next;
    kpts_next.clear();
    status.clear();
    err.clear();
  }
}