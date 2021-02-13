#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#include <ros/ros.h>

#include <cmath>
#include <iostream>
#include <sstream>
#include <stdio.h>  /* pow */
#include <stdlib.h> /* abs */
#include <string>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/Householder>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <Eigen/Sparse>

#include <math.h>

using namespace std;
using namespace Eigen;

std::vector<double> convert_matrix_vector(
    Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> Ro);
Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>
convert_vector_matrix(std::vector<double> Ro);
Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>
convert_vector_matrix_real(std::vector<double> Ro, int rows, int cols);
Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>
convert_vector_to_diag_matrix(std::vector<double> Ro, int n);

/**
 * Sigmoid function: based on the value x it returns the correspondent
 * y=sigmoid(x).
 *
 * @param x is the current value that we should compute its y value.
 * @param xm is the lower bounf where the sigmoid starts to have a value
 * different than 0.
 * @param b is the length of the sigmoid functions, at xm+b the sigmoid function
 * starts to have a value 1.
 */
long double sigmoid(long double x, long double xm, long double b);

/**
 * gbellmf function: Generalized bell-shaped membership function.
 *
 * @param x is the input function.
 * @param a,b,c is the configuration parameters of the membership function.
 */
long double gbellmf(long double x, long double a, long double b, long double c);

/// @brief Functions is a class that provides all the mathematical functions
/// required in task priority control.
class Functions {
public:
  /**
   * Default constructor of Functions
   */
  Functions();

  Functions(Eigen::Matrix<long double, Eigen::Dynamic, 1> xc);

  /**
   * Destructor of Functions
   */
  ~Functions();

  /**
   * weighted_normalized_pinv function: it returns the weighted normalized
   * pseudo-inverse of matrix X.
   *
   * @param X.
   * @param A.
   * @param Q.
   * @param eta.
   */
  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>
  weighted_normalized_pinv(
      Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> X,
      Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> A,
      Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> Q,
      long double eta);

  /**
   * weighted_normalized_pinv_Inertia function: it returns the weighted
   * normalized pseudo-inverse of matrix X considering the inertia matrix of the
   * robot.
   *
   * @param X.
   * @param A.
   * @param Q.
   * @param eta.
   * @param Minv.
   */
  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>
  weighted_normalized_pinv_Inertia(
      Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> X,
      Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> A,
      Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> Q,
      long double eta,
      Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> Minv);

  /**
   * weighted_generalized_inv function: it returns the weighted generalized
   * inverse of matrix A.
   *
   * @param A.
   * @param MX.
   * @param MU.
   */
  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>
  weighted_generalized_inv(
      Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> A,
      Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> MX,
      Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> MU);

  std::vector<Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>>
  frf(Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> X);

  std::vector<Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>>
  splitD(Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> D);

  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>
  round2(Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> X,
         int p = 2);

  // std::vector<Eigen::Matrix<long double,Eigen::Dynamic,1>>
  // T_interpolation(Eigen::Matrix<long double,Eigen::Dynamic,1> vec,
  // Eigen::Matrix<long double,Eigen::Dynamic,1> xc, double t);

  // Eigen::Matrix<long double,Eigen::Dynamic,1>  QuatInterp(Eigen::Matrix<long
  // double,Eigen::Dynamic,1>  q1, Eigen::Matrix<long double,Eigen::Dynamic,1>
  // q2, long double t);

private:
  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> P, I, tmpX, U, V,
      S, AA, Mx, Mu;
  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> F1, C1;
  std::vector<Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>> f;
};

#endif // FUNCTIONS_H
