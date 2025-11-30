#include <iostream>
#include <Eigen/Core>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

int main()
{
    // SO(3) uses
    // Generate a random vector
    Eigen::Vector3d rand_vec3 = Eigen::Vector3d::Random() / 100.0;
    std::cout << "rand_vec3: " << rand_vec3.transpose() << std::endl;

    // Convert rotation vector into SO(3) via exponential map
    Sophus::SO3d rand_R = Sophus::SO3d::exp(rand_vec3);
    std::cout << "rand_R:\n" << rand_R.matrix() << std::endl;

    // Convert back to rotation vector via log map
    Eigen::Vector3d log_rand_R = rand_R.log();
    std::cout << "log_rand_R: " << log_rand_R.transpose() << std::endl << std::endl;

    // SE(3) uses
    // Generate a random vector

    // If you use Eigen 3.4.0 or higher, the code below will work (and is better to read!)
    // Eigen::Vector<double, 6> rand_vec6 = Eigen::Vector<double, 6>::Random() / 100.0;
    // If you use Eigen <3.3.9, Vector<double, 6> is not possible, therefore must use Matrix instead.
    Eigen::Matrix<double, 6, 1> rand_vec6 = Eigen::Matrix<double, 6, 1>::Random() / 100.0;
    std::cout << "rand_vec6: " << rand_vec6.transpose() << std::endl;

    // Convert twist vector into SE(3) via exponential map
    Sophus::SE3d rand_T = Sophus::SE3d::exp(rand_vec6);
    std::cout << "rand_T:\n" << rand_T.matrix() << std::endl;

    // Convert back to twist vector via log map
    // Eigen::Vector<double, 6> log_rand_T = rand_T.log();
    Eigen::Matrix<double, 6, 1> log_rand_T = rand_T.log();
    std::cout << "log_rand_T: " << log_rand_T.transpose() << std::endl;


    // SO(3) Perturbation
    Eigen::Vector3d small_rot = Eigen::Vector3d(0.0001, 0.0, 0.0);
    std::cout << "small_rot :" << small_rot.transpose() << std::endl;

    // Convert small rotation into SO(3) via exponential map
    Sophus::SO3d small_R = Sophus::SO3d::exp(small_rot);
    std::cout << "small_R:\n" << small_R.matrix() << std::endl;

    // Apply rotation
    Sophus::SO3d new_R = small_R * rand_R;
    std::cout << "new_R:\n" << new_R.matrix() << std::endl;

    return 0;
}

