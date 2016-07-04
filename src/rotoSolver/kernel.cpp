/**************************************************************************
** This file is a part of our work (Siggraph'16 paper, binary, code and dataset):
**
** Roto++: Accelerating Professional Rotoscoping using Shape Manifolds
** Wenbin Li, Fabio Viola, Jonathan Starck, Gabriel J. Brostow and Neill D.F. Campbell
**
** w.li AT cs.ucl.ac.uk
** http://visual.cs.ucl.ac.uk/pubs/rotopp
**
** Copyright (c) 2016, Wenbin Li
** All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**
** -- Redistributions of source code and data must retain the above
**    copyright notice, this list of conditions and the following disclaimer.
** -- Redistributions in binary form must reproduce the above copyright
**    notice, this list of conditions and the following disclaimer in the
**    documentation and/or other materials provided with the distribution.
**
** THIS WORK AND THE RELATED SOFTWARE, SOURCE CODE AND DATA IS PROVIDED BY
** THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
** WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
** MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
** NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
** INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
** BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
** USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
** NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
** EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************/

#include "include/rotoSolver/kernel.hpp"

typedef Eigen::MatrixXd Matrix;
typedef Eigen::VectorXd Vector;
typedef Eigen::SparseMatrix<double> SparseMatrix;
typedef Eigen::MatrixXd ExtraDataType;

template <typename DerivedX, typename DerivedY>
Eigen::Matrix<typename DerivedX::Scalar, Eigen::Dynamic, Eigen::Dynamic>

getEuclideanDistanceSquared(const Eigen::MatrixBase<DerivedX>& X,
                            const Eigen::MatrixBase<DerivedY>& Y)
{
    typedef typename DerivedX::Scalar T;
    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Mat;

    Mat XX = (X.array() * X.array()).rowwise().sum();
    Mat YY = (Y.array() * Y.array()).rowwise().sum();
    YY.transposeInPlace();

    Mat D(X.rows(), Y.rows());
    D = XX.replicate(1, Y.rows()) + YY.replicate(X.rows(), 1);
    D -= T(2.0) * X * Y.transpose();

    return D;
}

template <typename DerivedX, typename DerivedY, typename DerivedD>
Eigen::Matrix<typename DerivedX::Scalar, Eigen::Dynamic, Eigen::Dynamic>
getRbfKernel(const Eigen::MatrixBase<DerivedX>& X,
             const Eigen::MatrixBase<DerivedY>& Y,
             const typename DerivedX::Scalar gamma,
             const typename DerivedX::Scalar alpha,
             Eigen::MatrixBase<DerivedD>* Dptr = NULL)
{
    typedef typename DerivedX::Scalar T;
    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Mat;

    auto D = getEuclideanDistanceSquared(X, Y);
    Mat K = alpha * exp((T(-0.5) * gamma * D).array());

    if (Dptr)
    {
        (*Dptr) = D;
    }

    return K;
}

RbfKernel::RbfKernel() : _alpha(1), _gamma(1)
{
}

RbfKernel::RbfKernel(const double alpha, const double gamma) : _alpha(alpha), _gamma(gamma)
{
}

Kernel* RbfKernel::createCopy() const
{
    return new RbfKernel(*this);
}

int RbfKernel::NumberOfParameters() const
{
    return NUMBER_PARAMETERS;
}

void RbfKernel::SetParameters(const Vector& logParams)
{
    assert (logParams.size() == NUMBER_PARAMETERS);

    _alpha = exp(logParams[0]);
    _gamma = exp(logParams[1]);
}

void RbfKernel::GetParameters(Vector& logParams) const
{
    logParams.resize(NUMBER_PARAMETERS, 1);

    logParams[0] = log(_alpha);
    logParams[1] = log(_gamma);
}

void RbfKernel::Print() const
{
    std::cout << "RbfKernel: " << "alpha = " << _alpha;
    std::cout << ", gamma = " << _gamma << std::endl;
}

Matrix RbfKernel::GetKernelMatrix(const Matrix& X, const Matrix& Z,
                                          ExtraDataType* extraDataPtr) const
{
    auto D = getEuclideanDistanceSquared(X, Z);
    Matrix K = _alpha * exp((-0.5 * _gamma * D).array());

    if (extraDataPtr)
    {
        (*extraDataPtr) = D;
    }

    return K;
}

Vector RbfKernel::GetDiagonalOfKernel(const Matrix& X) const
{
    Vector diagK = _alpha * Vector::Ones(X.rows());
    return diagK;
}

std::vector< shared_ptr< Matrix > >
RbfKernel::GetGradientWrtParams(const Matrix& X, const Matrix& K,
                                ExtraDataType* extraDataPtr) const
{
    typedef shared_ptr< Matrix > MatPtr;

    if (!extraDataPtr)
    {
        Matrix KK = GetKernelMatrix(X, X, extraDataPtr);
        assert (K.norm() == KK.norm());
    }
    const Matrix& DD = *(extraDataPtr);

    std::vector< MatPtr > dK_dparams(2);

    MatPtr dK_dalpha(new Matrix(K));
    MatPtr dK_dgamma(new Matrix(-0.5 * _gamma * (DD.array() * K.array())));

    dK_dparams[0].swap(dK_dalpha);
    dK_dparams[1].swap(dK_dgamma);

    return dK_dparams;
}

shared_ptr< SparseMatrix >
RbfKernel::GetGradientWrtData(const Matrix& X, const Matrix& K,
                              ExtraDataType* extraDataPtr) const
{
    (void) extraDataPtr;

    const int N = X.rows();
    const int Q = X.cols();

    Matrix gX = - _gamma * X;

    const int numNonZerosPerCol = 2 * Q;

    // default is column major
    shared_ptr< SparseMatrix > dK_dX(new SparseMatrix(N*Q, N*N));

    // Speed up allocation by specifying num per col..
    dK_dX->reserve(Eigen::VectorXi::Constant(N*N, numNonZerosPerCol));

    for (int q = 0; q < Q; ++q)
    {
        for (int n = 0; n < N; ++n)
        {
            const int i = (q * N) + n;
            assert (i < N*Q);

            for (int j = 0; j < N; ++j)
            {
                // NOTE: THIS IS ONLY ALLOWED WITH K(X, X) __NOT__ WITH K(X,Z)
                if (j == n)
                {
                    // NOTE: THIS IS ONLY ALLOWED WITH K(X, X) __NOT__ WITH K(X,Z)
                    continue;
                }
                assert (n*N + j < N*N);
                assert (j*N + n < N*N);

                dK_dX->insert(i, n*N + j) = K(n,j) * (gX(n,q) - gX(j,q));
                dK_dX->insert(i, j*N + n) = K(j,n) * (gX(n,q) - gX(j,q));
            }
        }
    }

    return dK_dX;
}

shared_ptr< SparseMatrix >
RbfKernel::GetGradientWrtSecondTerm(const Matrix& X, const Matrix& Z, const Matrix& K,
                                    ExtraDataType* extraDataPtr) const
{
    (void) extraDataPtr;
    assert (X.rows() == K.rows());
    assert (Z.rows() == K.cols());
    assert (X.cols() == Z.cols());

    const int M = X.rows();
    const int N = Z.rows();

    const int Q = X.cols();

    const int numNonZerosPerCol = Q;

    // default is column major
    shared_ptr< SparseMatrix > dK_dZ(new SparseMatrix(N*Q, M*N));

    // Speed up allocation by specifying num per col..
    dK_dZ->reserve(Eigen::VectorXi::Constant(M*N, numNonZerosPerCol));

    for (int q = 0; q < Q; ++q)
    {
        for (int n = 0; n < N; ++n)
        {
            const int i = (q * N) + n;
            assert (i < N*Q);

            for (int j = 0; j < M; ++j)
            {
                assert (n*M + j < M*N);

                dK_dZ->insert(i, n*M + j) = _gamma * K(j,n) * (X(j,q) - Z(n,q));
            }
        }
    }

    return dK_dZ;
}

void RbfKernel::AddParametersToVariableMap(
        std::map<std::string, shared_ptr<Matrix> >& variableMap) const
{
    variableMap["RbfKernel_alpha"] = shared_ptr<Matrix>(new Matrix(Matrix::Constant(1, 1, _alpha)));
    variableMap["RbfKernel_gamma"] = shared_ptr<Matrix>(new Matrix(Matrix::Constant(1, 1, _gamma)));
}
