#include <iostream>
#include <Eigen/Core>
#include <sophus/so3.h>
#include <sophus/se3.h>

int main()
{
    // SO(3) uses
    // Generate a random vector
    Eigen::Vector3d rand_vec3 = Eigen::Vecrtor3d::Random() / 100.0;    
    std::cout << "rand_vec3: " << rand_vec3.transpose() << std::endl;

    // Convert rotation vector into SO(3) via exponential map
    Sophus::SO3d rand_R = Sophus:SO3d::exp(rand_vec3);
    std::cout << "rand_R:\n" << rand_R.matrix() << std::endl;
    
    // Convert back to rotation vector via log map
    Eigen::Vector3d log_rand_R = rand_R.log();
    std::cout << "log_rand_R: " << log_rand_R.transpose() << std::endl;
    
    
    // SE(3) uses
    // Generate a random vector
    Eigen::Vector6d rand_vec6 = Eigen::Vector6d::Random() / 100.0;  
    std::cout << "rand_vec6: " << rand_vec6.transpose() << std::endl;

    // Convert twist vector into SE(3) via exponential map
    Sophus::SE3d rand_T = Sophus:SE3d::exp(rand_vec6);
    std::cout << "rand_T:\n" << rand_T.matrix() << std::endl;
    
    // Convert back to twist vector via log map
    Eigen::Vector6d log_rand_T = rand_T.log();
    std::cout << "log_rand_T: " << log_rand_T.transpose() << std::endl;
    
    return 0;
}
