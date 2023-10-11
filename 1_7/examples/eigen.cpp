#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <iostream>

int main() {
  // Matrices
  Eigen::MatrixXd m(2, 2);
  m(0, 0) = 3.0;
  m(1, 0) = 2.5;
  m(0, 1) = -1;
  m(1, 1) = m(1, 0) + m(0, 1);

  Eigen::Matrix2d m2;
  m2 << 1.0, 2.0, 3.0, 4.0;

  std::cout << "m = " << std::endl << m << "\n\n";
  std::cout << "m2 = " << std::endl << m2 << "\n\n";
  std::cout << "m*m2 = " << std::endl << m * m2 << "\n\n";
  std::cout << "m+m2 = " << std::endl << m + m2 << "\n\n";

  // Vectors
  Eigen::VectorXd v(2);
  v << 1.0, 2.0;

  std::cout << "m*v = " << std::endl << m * v << "\n\n";

  // Solving linear systems
  Eigen::VectorXd b(2);
  b << 2.0, -1.0;
  std::cout << "Matrix m:\n" << m << std::endl;
  std::cout << "Vector b:\n" << b << std::endl;
  Eigen::VectorXd x = m.colPivHouseholderQr().solve(b);
  std::cout << "Solution is:\n" << x << std::endl;
  std::cout << "Error: " << (m * x - b).norm() << std::endl << std::endl;

  // Eigenvalues
  Eigen::Matrix2d A;
  A << 1, 2, 2, 3;
  std::cout << "Matrix A:\n" << A << std::endl;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(A);
  if (eigensolver.info() != Eigen::Success)
    abort();
  std::cout << "Eigenvalues of A are:\n"
            << eigensolver.eigenvalues() << std::endl;
  std::cout << "Matrix whose columns are eigenvectors of A \n"
            << "corresponding to these eigenvalues:\n"
            << eigensolver.eigenvectors() << std::endl << std::endl;

  // Sparse matrices
  Eigen::SparseMatrix<double> sp(5, 5);
  sp.insert(0, 0) = 1.0;
  sp.insert(1, 1) = 2.0;
  sp.insert(2, 2) = 3.0;
  sp.insert(3, 3) = 4.0;
  sp.insert(4, 4) = 5.0;

  std::cout << "Sparse matrix sp = " <<  sp << std::endl;
}