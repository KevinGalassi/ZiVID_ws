//#include <functions.hpp>
#include <move_rt/functions.hpp>

std::vector<double> convert_matrix_vector(
    Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> Ro) {
  std::vector<double> r;
  r.resize(Ro.size());
  r.clear();
  for (int i = 0; i < Ro.rows(); i++)
    for (int j = 0; j < Ro.cols(); j++)
      r.push_back(Ro(i, j));

  return r;
}

Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>
convert_vector_matrix(std::vector<double> Ro) {
  Eigen::Matrix<long double, Eigen::Dynamic, 1> r;
  r.resize(Ro.size(), 1);
  for (int i = 0; i < Ro.size(); i++)
    r(i, 0) = Ro[i];
  return r;
}

Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>
convert_vector_matrix_real(std::vector<double> Ro, int rows, int cols) {
  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> r;
  r.resize(rows, cols);
  int count = 0;
  for (int i = 0; i < rows; i++)
    for (int j = 0; j < cols; j++)
      r(i, j) = Ro[count++];
  return r;
}

Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>
convert_vector_to_diag_matrix(std::vector<double> Ro, int n) {
  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> r;
  r.resize(n, n);
  r.setZero();
  int count = 0;
  for (int i = 0; i < n; i++)
    r(i, i) = Ro[count++];
  return r;
}

long double sigmoid(long double x, long double xm, long double b) {
  if (x < xm)
    return 1.0;

  if (x > (xm + b))
    return 0.0;

  return (cos((x - xm) * M_PI / b) + 1) / 2;
}

long double gbellmf(long double x, long double a, long double b,
                    long double c) {
  return 1.0 / (1 + pow(abs((x - c) / a), 2 * b));
  ;
}

Functions::Functions() {}

Functions::~Functions() {}

Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>
Functions::weighted_normalized_pinv(
    Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> X,
    Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> A,
    Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> Q,
    long double eta) {
  P.resize(X.cols(), X.cols());
  P.setIdentity();

  I.resize(Q.rows(), Q.cols());
  I.setIdentity();

  tmpX = X.transpose() * A * X + eta * (I - Q).transpose() * (I - Q);

  Eigen::JacobiSVD<Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>>
      svd(tmpX, Eigen::ComputeFullU | Eigen::ComputeFullV);
  U = svd.matrixU();
  V = svd.matrixV();
  S = U.transpose() * tmpX * V;

  for (int i = 0; i < S.rows(); i++) {
    P(i, i) = gbellmf(S(i, i) * pow(10, 6), 0.2, 1.0, 0.0);
  }

  Mx.resize(tmpX.rows(), tmpX.rows());
  Mx.setIdentity();

  Mu.resize(tmpX.cols(), tmpX.cols());
  Mu.setIdentity();

  AA = tmpX + V * P * V.transpose();

  return weighted_generalized_inv(AA, Mx, Mu) * X.transpose() * A * A;
}

Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>
Functions::weighted_normalized_pinv_Inertia(
    Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> X,
    Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> A,
    Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> Q,
    long double eta,
    Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> Minv) {

  P.resize(X.cols(), X.cols());
  P.setIdentity();

  I.resize(Q.rows(), Q.cols());
  I.setIdentity();

  tmpX =
      Minv * X.transpose() * A * X + eta * Minv * (I - Q).transpose() * (I - Q);

  Eigen::JacobiSVD<Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>>
      svd(tmpX, Eigen::ComputeFullU | Eigen::ComputeFullV);
  U = svd.matrixU();
  V = svd.matrixV();
  S = U.transpose() * tmpX * V;

  for (int i = 0; i < S.rows(); i++) {
    P(i, i) = gbellmf(S(i, i) * pow(10, 6), 0.2, 1.0, 0.0);
  }

  Mx.resize(tmpX.rows(), tmpX.rows());
  Mx.setIdentity();

  Mu.resize(tmpX.cols(), tmpX.cols());
  Mu.setIdentity();

  AA = tmpX + V * P * V.transpose();
  // std::cout<<"####################à"<<std::endl;
  // std::cout<<"Minv "<<Minv<<std::endl;

  return weighted_generalized_inv(AA, Mx, Mu) * Minv.transpose() *
         X.transpose() * A * A;
}

Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>
Functions::weighted_generalized_inv(
    Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> A,
    Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> MX,
    Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> MU) {
  f.clear();
  f = frf(A);

  F1 = f[0];
  C1 = f[1];

  return MX.bdcSvd(ComputeThinU | ComputeThinV).solve(C1.transpose()) *
         (C1 * (MX.bdcSvd(ComputeThinU | ComputeThinV).solve(C1.transpose())))
             .bdcSvd(ComputeThinU | ComputeThinV)
             .solve((F1.transpose() * MU * F1)
                        .bdcSvd(ComputeThinU | ComputeThinV)
                        .solve(F1.transpose())) *
         MU;
}

std::vector<Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>>
Functions::frf(Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> X) {
  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> U, V, S;

  Eigen::JacobiSVD<Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>>
      svd(X, Eigen::ComputeFullU | Eigen::ComputeFullV);
  U = svd.matrixU();
  V = svd.matrixV();
  S = U.transpose() * X * V;

  std::vector<Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>> res,
      d;

  d = splitD(S);

  res.clear();
  res.resize(2);
  res[0] = U * d[0];
  res[1] = d[1] * V.transpose();

  res[0] = round2(res[0], 6);
  res[1] = round2(res[1], 6);

  return res;
}

std::vector<Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>>
Functions::splitD(
    Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> D) {
  int n = D.rows(), p = D.cols();
  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> D1, D2, a, sqrt_a;
  D1.resize(n, p);
  D1.setZero();
  D2.resize(p, p);
  D2.setZero();

  a = (D.diagonal()).asDiagonal();

  sqrt_a.resize(a.rows(), a.cols());
  sqrt_a.setZero();

  for (int i = 0; i < a.rows(); i++) {
    sqrt_a(i, i) = pow(a(i, i), 0.5);
  }

  for (int i = 0; i < a.rows(); i++) {
    D1(i, i) = sqrt_a(i, i);
    D2(i, i) = sqrt_a(i, i);
  }

  std::vector<Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>> res;
  res.clear();
  res.push_back(D1);
  res.push_back(D2);

  return res;
}

Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>
Functions::round2(Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> X,
                  int p) {
  Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> res, temp;
  long double dd;

  temp = X * pow(10, p);

  for (int i = 0; i < temp.rows(); i++) {
    for (int j = 0; j < temp.cols(); j++) {
      dd = std::roundl(temp(i, j));
      temp(i, j) = dd;
    }
  }

  res = pow(10, -p) * (temp);

  return res;
}
