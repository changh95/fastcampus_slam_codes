#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

int main()
{
    cv::Mat img_left = cv::imread("left.png", cv::IMREAD_GRAYSCALE);
    cv::Mat img_right = cv::imread("right.png", cv::IMREAD_GRAYSCALE);

    // Detect ORB and match
    auto feature_detector = cv::ORB::create(1000);
    auto bf_matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
    std::vector<cv::DMatch> bf_matches;

    std::vector<cv::KeyPoint> kpts_left, kpts_right;
    cv::Mat desc_left, desc_right;
    std::vector<cv::DMatch> matches;

    feature_detector->detectAndCompute(img_left, cv::noArray(), kpts_left, cv::noArray());
    feature_detector->detectAndCompute(img_right, cv::noArray(), kpts_right, cv::noArray());

    bf_matcher->match(desc_left, desc_right, bf_matches);
    std::vector<cv::DMatch> good_bf_matches;
    for (const auto &match : bf_matches) {
        if (match.distance < 50) {
            good_bf_matches.push_back(match);
        }
    }

    cv::Mat img_bf;
    cv::drawMatches(img_left, kpts_left, img_right, kpts_right, good_bf_matches,
                    img_bf, cv::Scalar::all(-1), cv::Scalar::all(-1),
                    std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    cv::Mat K = (cv::Mat_<double>(3,3) << 9.799200e+02, 0.000000e+00, 6.900000e+02, 0.000000e+00, 9.741183e+02, 2.486443e+02, 0.000000e+00, 0.000000e+00, 1.000000e+00);

    cv::imshow("BF Matches", img_bf);
    cv::waitKey(0);
}