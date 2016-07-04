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

#ifndef GPLVM_HPP
#define GPLVM_HPP

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "ceres/solver.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <Eigen/Eigenvalues>

#include <array>
#include <random>
#include <iostream>
#include <cmath>
#include <fstream>
#include <cassert>
#include <chrono>

#include "include/rotoSolver/kernel.hpp"
#include "include/rotoSolver/baseDefs.hpp"

using namespace std;

class NoiseHyperPriorFunction
{
public:
    virtual double getHyperPrior(const double logBeta) const = 0;
    virtual double getHyperPriorDerivative(const double logBeta) const = 0;
};

class GaussianNoiseHyperPriorFunction : public NoiseHyperPriorFunction
{
private:
    double _logOffset;
    double _logScaleSq;

public:
    GaussianNoiseHyperPriorFunction(double logOffset = 0.0, double logScale = 1.0) :
        _logOffset(logOffset), _logScaleSq(logScale * logScale)
    {}

    virtual double getHyperPrior(const double logBeta) const
    {
        double b = logBeta - _logOffset;
        return (0.5 * _logScaleSq * b * b);
    }

    virtual double getHyperPriorDerivative(const double logBeta) const
    {
        return (_logScaleSq * logBeta);
    }
};

class GPLVM {
public:
    typedef Eigen::VectorXd Vector;
    typedef Eigen::MatrixXd Matrix;
    typedef Eigen::SparseMatrix<double> SparseMatrix;
    explicit GPLVM(const Matrix& Y, const int Q,
                   shared_ptr<Kernel> kernel,
                   const double initBeta = 1,
                   shared_ptr<NoiseHyperPriorFunction> noisePriorFunc =
     shared_ptr<NoiseHyperPriorFunction>(new GaussianNoiseHyperPriorFunction())) :
        _Y(Y), _N(_Y.rows()), _Q(Q), _kernel(kernel), _beta(initBeta),
        _noisePriorFunc(noisePriorFunc)
    {
        assert (_N > 0);
        assert (_Q > 0);

        const double N = double(_Y.rows());

        _Ymean = _Y.array().colwise().mean();
        _Y = _Y.array().rowwise() - _Ymean.array().colwise().mean();
        _Ystd = (_Y.array().pow(2.0).colwise().sum() / (N-1.0)).sqrt();
        _Y = _Y.array().rowwise() / _Ystd.array().colwise().mean();

#ifdef CHECK_NORMALISATION
        {
            vdbg(_Ymean);
            vdbg(_Ystd);
            vdbg(_Y.transpose());

            Matrix yy_mean = _Y;
            yy_mean = yy_mean.array().rowwise() * _Ystd.array().colwise().mean();
            yy_mean = yy_mean.array().rowwise() + _Ymean.array().colwise().mean();

            vdbg((yy_mean - Y).norm());
        }
#endif // CHECK_NORMALISATION

        _X = GetDefaultXInit();

        resetCachedValues();
    }

    void Print() const
    {
        std::cout << "GPLVM: ";
        std::cout << " N = " << _N;
        std::cout << " Q = " << _Q;
        std::cout << " beta = " << _beta << std::endl;
        _kernel->Print();
    }

    void SetInternalsFromMatlabStructure(std::map<string, shared_ptr<Eigen::MatrixXd> >& S)
    {
        _Y = *S["Y"];
        _Ymean = *S["Y_mean"];
        _Ystd = *S["Y_std"];

        _X = *S["X"];

        resetCachedValues();
    }

    inline int NumParameters() const
    {
        return NumPreDataParameters() + (_N * _Q);
    }

    inline int NumPreDataParameters() const
    {
        return (_kernel->NumberOfParameters() + 1);
    }

    int GetN() const { return _N; }

    int GetQ() const { return _Q; }

    // Cannot return ref due to normalisation so don't call too often..
    Matrix GetY() const
    {
        Matrix Y(_Y);
        Y = Y.array().rowwise() * _Ystd.array().colwise().mean();
        Y = Y.array().rowwise() + _Ymean.array().colwise().mean();
        return Y;
    }

    Matrix GetX() const { return _X; }

    double GetBeta() const { return _beta; }

    void SetBeta(const double newBeta)
    {
        _beta = newBeta;

        resetCachedValues();
    }

    template<typename Derived>
    Matrix Predict(const Eigen::MatrixBase<Derived>& x,
                   Vector* y_var_ptr = NULL,
                   std::vector<Matrix>* dy_dx_vec_ptr = NULL) const
    {
        assert (x.cols() == _Q);
        assert (x.rows() > 0);

        Matrix K_x_X = _kernel->GetKernelMatrix(x, _X, NULL);

        const Matrix& KinvY = GetKinvY();

        Matrix y_mean = K_x_X * KinvY;

        if (y_var_ptr) {
            Vector& y_var = *(y_var_ptr);
            y_var.resize(x.rows());

            const Matrix& Kinv = GetKinv();

            Vector K_xx_xx_diag = _kernel->GetDiagonalOfKernel(x);

            y_var = K_xx_xx_diag.array() -
                    (K_x_X.array() * (K_x_X * Kinv.transpose()).array()).rowwise().sum().array();
            y_var.array() += (1.0 / _beta);
        }

        y_mean = y_mean.array().rowwise() * _Ystd.array().colwise().mean();
        y_mean = y_mean.array().rowwise() + _Ymean.array().colwise().mean();

        if (dy_dx_vec_ptr)
        {
            std::vector<Matrix>& dy_dx_vec = *(dy_dx_vec_ptr);
            dy_dx_vec.clear();

            const Matrix& Kinv = GetKinv();

            const int D = _Y.cols();
            const int P = x.rows();

            dy_dx_vec.reserve(P);

            for (int p = 0; p < P; ++p)
            {
                shared_ptr<SparseMatrix> dK_dx = _kernel->GetGradientWrtSecondTerm(_X, x.row(p), K_x_X.row(p).transpose());

                Matrix dy_dx = (*dK_dx) * KinvY;

                dy_dx = dy_dx.array().rowwise() * _Ystd.array().colwise().mean();

                dy_dx_vec.push_back(dy_dx);
            }
        }

        return y_mean;
    }

    void SaveToMatFile(std::string matFilename) const
    {
        std::cerr << "UNSUPPORTED: UNABLE TO WRITE GPLVM MATLAB DATA TO \"" << matFilename << "\"." << std::endl;
    }

private:
    Matrix _Y;
    Matrix _Ymean;
    Matrix _Ystd;
    const int _N;
    const int _Q;
    shared_ptr<Kernel> _kernel;
    double _beta;
    Matrix _X;
    shared_ptr<NoiseHyperPriorFunction> _noisePriorFunc;

private:
    // Cached values (hence mutable)..
    mutable shared_ptr<Matrix> _KinvY_ptr;
    mutable shared_ptr<Matrix> _Kinv_ptr;

    void resetCachedValues()
    {
        _KinvY_ptr.reset();
        _Kinv_ptr.reset();
    }

protected:

    class CostFunction : public ceres::FirstOrderFunction
    {
    private:
        GPLVM* _gplvm;
        shared_ptr<Kernel> _kernel;

    public:
        explicit CostFunction(GPLVM* gplvm) :
            _gplvm(gplvm),
            _kernel(gplvm->_kernel->createCopy())
        {
        }

        virtual bool Evaluate(const double* parameters,
                              double* cost,
                              double* gradient) const {
            const int numParams = _gplvm->NumPreDataParameters();
            const int numKernelParams = _kernel->NumberOfParameters();
            const int N = _gplvm->_N;
            const int Q = _gplvm->_Q;
            const Matrix& Y = _gplvm->_Y;

            const double logBeta = parameters[0];
            const double beta = exp(logBeta);
            const Eigen::Map< const Vector > logKernelParams(&parameters[1], numKernelParams);

            _kernel->SetParameters(logKernelParams);

            const Eigen::Map< const Matrix > X(&parameters[numParams], N, Q);
            const Eigen::Map< const Vector > Xvec(&parameters[numParams], N * Q);

            // Create noise free copy of K and prepare extra data
            Matrix DDmat;
            Matrix KK = _kernel->GetKernelMatrix(X, X, &DDmat);
            Matrix K(KK);

            const double Nd = N;
            const double Dd = Y.cols();

            K.diagonal().array() += (1.0 / beta);
            Matrix L = K.llt().matrixL();

            double logDet = 2.0 * (L.diagonal().array().log()).sum();

            Matrix LinvY = K.llt().matrixL().solve(Y);
            double trace_Kinv_YYt = LinvY.squaredNorm();

            double hyperPriorNoise = _gplvm->_noisePriorFunc->getHyperPrior(logBeta);

            double hyperPriorParams = 0.5 * (logKernelParams.array() * logKernelParams.array()).sum();

            double hyperPriorData = 0.5 * X.squaredNorm();

            double LogLike = (0.5 * Dd * Nd) * log(2.0 * M_PI);
            LogLike += 0.5 * Dd * logDet;
            LogLike += 0.5 * trace_Kinv_YYt;
            LogLike += hyperPriorParams;
            LogLike += hyperPriorNoise;

            LogLike += hyperPriorData;

            cost[0] = LogLike;

            if (gradient != NULL)
            {
                // The beta gradient
                double& dL_dlogBeta = gradient[0];

                // The kernel params gradient
                Eigen::Map< Eigen::VectorXd > dL_dlogKernParams(&gradient[1], numKernelParams);

                // The final data gradient
                Eigen::Map< Eigen::VectorXd > dL_dX(&gradient[numParams], N*Q);


                Matrix dK_dbeta(Matrix::Identity(N, N));
                dK_dbeta.diagonal() /= - beta;

                // std::vector< shared_ptr< Matrix > >
                auto dK_dlogKernParams = _kernel->GetGradientWrtParams(X, KK, &DDmat);

                // shared_ptr< Eigen::SparseMatrix<double> >
                auto dK_dX_ptr = _kernel->GetGradientWrtData(X, KK, &DDmat);

                Matrix KinvY = K.llt().matrixL().transpose().solve(LinvY);

                Matrix dL_dK = - 0.5 * (KinvY * KinvY.transpose() - (Dd * K.llt().solve(Matrix::Identity(N, N))));

                dL_dlogBeta = (dL_dK.array() * dK_dbeta.array()).sum();
                dL_dlogBeta += _gplvm->_noisePriorFunc->getHyperPriorDerivative(logBeta);

                for (int i = 0; i < numKernelParams; ++i)
                {
                    dL_dlogKernParams[i] = (dL_dK.array() * (*(dK_dlogKernParams[i])).array()).sum() + logKernelParams[i];
                }

                Eigen::Map< Vector > dL_dK_vec(dL_dK.data(), N*N);

                dL_dX = (*dK_dX_ptr) * dL_dK_vec;

                // Add hyperPriorData gradient
                dL_dX += Xvec;
            }

            return true;
        }

        virtual int NumParameters() const
        {
            return _gplvm->NumParameters();
        }
    };

private:

    Matrix GetDefaultXInit() const
    {
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(_Y * _Y.transpose());
        Matrix X0 = es.eigenvectors().rowwise().reverse().block(0, 0, _N, _Q);

        const double scaling = ((X0.array().pow(2.0).colwise().sum() / (double(_N)-1.0)).sqrt()).mean();
        X0 /= scaling;

        return X0;
    }

    Vector GetParameterVector() const
    {
        Vector p(NumParameters());

        p[0] = log(_beta);

        Vector logKernParams;
        _kernel->GetParameters(logKernParams);
        p.block(1, 0, _kernel->NumberOfParameters(), 1) = logKernParams;

        const Eigen::Map< const Vector > Xvec(_X.data(), _N*_Q);
        p.block(NumPreDataParameters(), 0, _N*_Q, 1) = Xvec;

        return p;
    }

    void SetFromParameterVector(const Vector& p)
    {
        _beta = exp(p[0]);

        Vector logKernParams = p.block(1, 0, _kernel->NumberOfParameters(), 1);
        _kernel->SetParameters(logKernParams);

        Eigen::Map< Vector > Xvec(_X.data(), _N*_Q);
        Xvec = p.block(NumPreDataParameters(), 0, _N*_Q, 1);

        resetCachedValues();
    }

    const Matrix& GetKinvY() const
    {
        if (!_KinvY_ptr.get())
        {
            std::cout << "Computing KinvY cached value.." << std::endl;

            Matrix DDmat;
            Matrix K = _kernel->GetKernelMatrix(_X, _X, &DDmat);

            K.diagonal().array() += (1.0 / _beta);

            Matrix LinvY = K.llt().matrixL().solve(_Y);

            shared_ptr<Matrix> tmp(new Matrix(K.llt().matrixL().transpose().solve(LinvY)));

            _KinvY_ptr.swap(tmp);
        }

        return (*_KinvY_ptr);
    }

    const Matrix& GetKinv() const
    {
        if (!_Kinv_ptr.get()) {
            std::cout << "Computing Kinv cached value.." << std::endl;

            Matrix DDmat;
            Matrix K = _kernel->GetKernelMatrix(_X, _X, &DDmat);

            K.diagonal().array() += (1.0 / _beta);

            Matrix Linv = K.llt().matrixL().solve(Matrix::Identity(_N, _N));
            shared_ptr<Matrix> tmp(new Matrix(Linv.transpose() * Linv));

            _Kinv_ptr.swap(tmp);

#ifdef TEST_OTHER_INV_METHOD
//            vdbg(K * (*_Kinv_ptr));
//            vdbg((*_Kinv_ptr) * K);

//            vdbg((K * (*_Kinv_ptr) - Matrix::Identity(_N,_N)).norm());
//            vdbg(((*_Kinv_ptr) * K - Matrix::Identity(_N,_N)).norm());
            vdbg((K * Kinv - Matrix::Identity(_N,_N)).norm());
            vdbg((Kinv * K - Matrix::Identity(_N,_N)).norm());

            start = std::chrono::high_resolution_clock::now();
            Matrix Kinv2 = K.inverse();
            stop = std::chrono::high_resolution_clock::now();
            vdbg(std::chrono::duration_cast<std::chrono::microseconds>(stop-start).count());

//            vdbg(K * Kinv);
//            vdbg(Kinv * K);

            vdbg((K * Kinv2 - Matrix::Identity(_N,_N)).norm());
            vdbg((Kinv2 * K - Matrix::Identity(_N,_N)).norm());
#endif // TEST_OTHER_INV_METHOD
        }

        return (*_Kinv_ptr);
    }

public:

    void TestGplvmGradients(const double step = 1e-8)
    {
        using std::cout; using std::endl;
        cout << endl << " TESTING GPLVM GRADIENTS ";
        cout << endl << " ======================= " << endl;

        CostFunction costFunc(this);

        Vector parameters(GetParameterVector());

        parameters.setRandom();

        double C0 = 0.0;
        Vector gradients(NumParameters());

        bool r = costFunc.Evaluate(parameters.data(), &C0, gradients.data());
        assert (r);

        vdbg(C0);
        vdbg(gradients.transpose());

        Vector estGradients(NumParameters());
        estGradients.setZero();

        for (int i = 0, I = NumParameters(); i < I; ++i)
        {
            Vector p(parameters);
            p[i] += step;
            double C = 0;

            bool r = costFunc.Evaluate(p.data(), &C, NULL);
            assert (r);

            estGradients[i] = (C - C0) / step;
        }

        vdbg(estGradients.transpose());

        vdbg((estGradients - gradients).transpose());
        vdbg((estGradients - gradients).norm() / gradients.norm());

        assert ((estGradients - gradients).norm() / gradients.norm() < 1e-4);

        cout << " ======================= " << endl << endl;
    }

    void TestPredictionGradients(Matrix x = Matrix(0,0), const double step = 1e-8)
    {
        using std::cout; using std::endl;
        cout << endl << " TESTING PREDICTION GRADIENTS ";
        cout << endl << " ============================ " << endl;

        if (x.size() == 0)
        {
            x = GetX();
        }

        Vector yVar;
        std::vector<Matrix> dy_dx_vec;
        Matrix y = Predict(x, &yVar, &dy_dx_vec);

        const int P = x.rows();

        for (int p = 0; p < P; ++p)
        {
            Matrix xx = x.row(p);
            xx = xx + Matrix::Constant(xx.rows(), xx.cols(), step);
            Matrix yy = (Predict(xx) - y.row(p));
            yy /= step;

            vdbg((yy - dy_dx_vec[p]).norm());

            assert ((yy - dy_dx_vec[p]).norm() / (dy_dx_vec[p]).norm() < (10 * step));
        }

        cout << " ============================ " << endl << endl;
    }

    // Optimise to convergence or until the first of the specified limits is reached..
    void LearnParameters(const double maxSolverTimeInSeconds = 1.0,
                         const int maxNumIterations = 10000,
                         const bool showProgress = false)
    {
        ceres::GradientProblem problem(new CostFunction(this));

        Vector parameters(GetParameterVector());

        ceres::GradientProblemSolver::Options options;
        options.minimizer_progress_to_stdout = showProgress;

        options.function_tolerance = 1e-6;
        options.gradient_tolerance = 1e-4;

        options.max_num_iterations = maxNumIterations;
        options.max_solver_time_in_seconds = maxSolverTimeInSeconds;

        ceres::GradientProblemSolver::Summary summary;
        ceres::Solve(options, problem, parameters.data(), &summary);

        SetFromParameterVector(parameters);
    }

};

#endif // GPLVM_HPP
