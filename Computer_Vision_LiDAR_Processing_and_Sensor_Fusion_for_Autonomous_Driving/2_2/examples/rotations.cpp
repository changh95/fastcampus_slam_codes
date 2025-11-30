#include <Eigen/Geometry>

#include <cmath>
#include <iostream>

int main() {
  // Angle-axis creation -> SO(3) conversion
  Eigen::AngleAxisd rot_vec(M_PI / 4.0, Eigen::Vector3d(0.0, 0.0, 1.0));
  const auto rot_mat = rot_vec.matrix();
  std::cout << "rotation vector = \n"
            << rot_vec.angle() << "," << rot_vec.axis().transpose() << "\n\n";
  std::cout << "rotation vector = \n" << rot_mat << "\n\n";

  // Multiply  by a vector (i.e. Rotate a vector)
  Eigen::Vector3d vec(1.0, 0.0, 0.0);
  const auto rotated_vector = rot_mat * vec;
  std::cout << "rotated vector = \n" << rotated_vector.transpose() << "\n\n";

  // Angle-Axis -> Quaternion conversion
  const auto quat = Eigen::Quaterniond(rot_vec);
  std::cout << "quaternion = \n" << quat.coeffs().transpose() << "\n\n";
  const auto rotated_vector2 = quat * vec;
  std::cout << "rotated vector2 = \n" << rotated_vector2.transpose() << "\n\n";
}