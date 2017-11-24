#pragma once
#ifndef __LAB2_MATH__
#define __LAB2_MATH__
#include "Eigen/Dense"
#include <iostream>
#include <random>

#define DEBUG
#ifdef DEBUG
#define PRINT_MATRIX(m) std::cout << #m << " Rows: " << m.rows() << " Cols: " << m.cols() << "\n" << m << std::endl;
#define DEBUG_LINE std::cout << __LINE__ << std::endl;
#else
#define DEBUG_LINE
#define PRINT_MATRIX(m)
#endif

inline double floatMod (
  double x,
  double y
)
{
  double result = fmod(x, y);
  return result >= 0 ? result : result + y;
}

inline Eigen::MatrixXd cumsum1D (
  Eigen::MatrixXd mat
)
{
  uint32_t numPoints = mat.cols();
  Eigen::MatrixXd::Scalar sum = 0;
  Eigen::MatrixXd cumsum;
  cumsum.resizeLike(mat);
  if (cumsum.rows() > 1) {
    std::cout << "cumsum 1d only works on 1d matricies!\n";
    return cumsum;
  }
  for (uint32_t idx = 0; idx < numPoints; ++idx) {
    sum += mat(idx);
    cumsum(idx) = sum;
  }
  return cumsum;
}

/** \brief Compute a normal pdf at a given set of data points
 * \param[in] x data points where rows correspond to dimensions and columns correspond to data points
 * \param[in] mean mean vector of the distribution
 * \param[in] cov covarinace matrix of the distribution
 * \return pdf values evaluated at data points
 * https://github.com/aecins/namaris/blob/master/include/namaris/utilities/math.hpp
 */
template <typename Scalar>
inline
Eigen::Matrix< Scalar, 1, Eigen::Dynamic > normpdf (
  const Eigen::Matrix< Scalar, Eigen::Dynamic, Eigen::Dynamic> x,
  const Eigen::Matrix< Scalar, Eigen::Dynamic, 1> mean,
  const Eigen::Matrix< Scalar, Eigen::Dynamic, Eigen::Dynamic> cov
)
{
  // Get dimensionality of the data
  uint32_t dim = x.rows();
  uint32_t numPoints = x.cols();

  // Check input parameters
  if (mean.rows() != dim)
  {
    std::cout << "[smt::normpdf] mean vector must have same dimensionality as data point vector." << std::endl;
    abort();
  }

  if (cov.rows() != dim || cov.cols() != dim)
  {
    std::cout << "[smt::normpdf] covariance matrix must be square and have same dimensionality as data point vector." << std::endl;
    abort();
  }

  // Precompute constants
  Scalar denom = std::sqrt(pow(2*M_PI, dim) * cov.determinant());
  Eigen::Matrix< Scalar, Eigen::Dynamic, Eigen::Dynamic> cov_invserse = cov.inverse() / -2;
  Eigen::Matrix< Scalar, Eigen::Dynamic, Eigen::Dynamic> x_demeaned = x - mean.replicate(1, numPoints);

  // Compute pdf values
  Eigen::Matrix< Scalar, 1, Eigen::Dynamic > pdf (numPoints);
  for (size_t pointId = 0; pointId < numPoints; pointId++)
    pdf.col(pointId) = (x_demeaned.col(pointId).transpose() * cov_invserse * x_demeaned.col(pointId));
  pdf = pdf.array().exp();
  pdf /= denom;

  return pdf;
}

namespace Eigen {
namespace internal {
  template<typename Scalar>
    struct scalar_normal_dist_op
    {
static std::mt19937 rng;                        // The uniform pseudo-random algorithm
mutable std::normal_distribution<Scalar> norm; // gaussian combinator

EIGEN_EMPTY_STRUCT_CTOR(scalar_normal_dist_op)

template<typename Index>
inline const Scalar operator() (Index, Index = 0) const { return norm(rng); }
inline void seed(const uint64_t &s) { rng.seed(s); }
    };

  template<typename Scalar>
    std::mt19937 scalar_normal_dist_op<Scalar>::rng;

  template<typename Scalar>
    struct functor_traits<scalar_normal_dist_op<Scalar> >
    { enum { Cost = 50 * NumTraits<Scalar>::MulCost, PacketAccess = false, IsRepeatable = false }; };

} // end namespace internal

/**
  Find the eigen-decomposition of the covariance matrix
  and then store it for sampling from a multi-variate normal
*/
template<typename Scalar>
  class EigenMultivariateNormal
{
  Matrix<Scalar,Dynamic,Dynamic> _covar;
  Matrix<Scalar,Dynamic,Dynamic> _transform;
  Matrix< Scalar, Dynamic, 1> _mean;
  internal::scalar_normal_dist_op<Scalar> randN; // Gaussian functor
  bool _use_cholesky;
  SelfAdjointEigenSolver<Matrix<Scalar,Dynamic,Dynamic> > _eigenSolver; // drawback: this creates a useless eigenSolver when using Cholesky decomposition, but it yields access to eigenvalues and vectors

public:
EigenMultivariateNormal(const Matrix<Scalar,Dynamic,1>& mean,const Matrix<Scalar,Dynamic,Dynamic>& covar,
      const bool use_cholesky=false,const uint64_t &seed=std::mt19937::default_seed)
    :_use_cholesky(use_cholesky)
   {
      randN.seed(seed);
setMean(mean);
setCovar(covar);
    }

  void setMean(const Matrix<Scalar,Dynamic,1>& mean) { _mean = mean; }
  void setCovar(const Matrix<Scalar,Dynamic,Dynamic>& covar)
  {
    _covar = covar;

    // Assuming that we'll be using this repeatedly,
    // compute the transformation matrix that will
    // be applied to unit-variance independent normals

    if (_use_cholesky)
{
  Eigen::LLT<Eigen::Matrix<Scalar,Dynamic,Dynamic> > cholSolver(_covar);
  // We can only use the cholesky decomposition if
  // the covariance matrix is symmetric, pos-definite.
  // But a covariance matrix might be pos-semi-definite.
  // In that case, we'll go to an EigenSolver
  if (cholSolver.info()==Eigen::Success)
    {
      // Use cholesky solver
      _transform = cholSolver.matrixL();
    }
  else
    {
      throw std::runtime_error("Failed computing the Cholesky decomposition. Use solver instead");
    }
}
    else
{
  _eigenSolver = SelfAdjointEigenSolver<Matrix<Scalar,Dynamic,Dynamic> >(_covar);
  _transform = _eigenSolver.eigenvectors()*_eigenSolver.eigenvalues().cwiseMax(0).cwiseSqrt().asDiagonal();
}
  }

  /// Draw nn samples from the gaussian and return them
  /// as columns in a Dynamic by nn matrix
  Matrix<Scalar,Dynamic,-1> samples(int nn)
    {
return (_transform * Matrix<Scalar,Dynamic,-1>::NullaryExpr(_covar.rows(),nn,randN)).colwise() + _mean;
    }
}; // end class EigenMultivariateNormal
} // end namespace Eigen

inline void getDists(
  Eigen::MatrixXd & samples,
  Eigen::MatrixXd & dists
)
{
  const int N = samples.rows();
  //
  // Allocate parts of the expression
  Eigen::MatrixXd XX, YY, XY;
  XX.resize(N,1);
  YY.resize(1,N);
  XY.resize(N,N);
  //
  // Compute norms
  XX = samples.array().square().rowwise().sum();
  YY = XX.transpose();
  XY = (2 * samples) * samples.transpose();
  //
  // Compute final expression
  dists = XX * Eigen::MatrixXd::Ones(1,N);
  dists = dists + Eigen::MatrixXd::Ones(N,1) * YY;
  dists = dists - XY;
  dists = dists.array().sqrt();
}

#endif
