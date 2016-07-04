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

#include "include/rotoSolver/transformations.hpp"
#include "include/rotoSolver/eigenUtils.hpp"

struct RigidBodyResidual
{
private:
    static const int NUM_RESIDUALS = 2;
    static const int NUM_DIMS = NUM_RESIDUALS;
    Eigen::Matrix<double, NUM_DIMS, 1> _observedPt;

public:
    RigidBodyResidual(const Matrix& observedPt) : _observedPt(observedPt)
    {}

    template <typename T>
    bool operator() (const T* const translation, const T* const rotation, const T* const point, T* residual) const
    {
        Eigen::Matrix<T, NUM_DIMS, 1> obs = _observedPt.cast<T>();

        const Eigen::Map< const Eigen::Matrix<T, NUM_DIMS, 1> > s(translation);
        const T& alpha = *rotation;
        const Eigen::Map< const Eigen::Matrix<T, NUM_DIMS, 1> > p(point);

        Eigen::Map< Eigen::Matrix<T, NUM_RESIDUALS, 1> > R(residual);

        Eigen::Matrix<T, 2, 2> Q(RotationMatrixFromAngle<T>(alpha));

        R = ((Q * p) + s) - obs;

        return true;
    }

    static ceres::CostFunction* create(const Matrix& observedPt)
    {
        ceres::CostFunction* costFunc =
                new ceres::AutoDiffCostFunction<RigidBodyResidual, NUM_RESIDUALS, NUM_DIMS, 1, NUM_DIMS>(
                    new RigidBodyResidual(observedPt));
        return costFunc;
    }
};

RigidMotionEstimator::RigidMotionEstimator(const Matrix& Y) :
    _Y(Y), _translations(), _rotations(), _referencePoints()
{
    const int D = _Y.cols();
    const int N = _Y.rows();

    nassert (N > 0);
    nassert (remainder(D, 2) == 0);
    _numPoints = D / 2;

    _translations = Matrix::Zero(N, 2);
    _rotations = Vector::Constant(N, 0.0);

    _referencePoints = Matrix::Zero(2, _numPoints);

    for (int n = 0; n < N; ++n)
    {
        Matrix obsPointsData(_Y.row(n));
        Eigen::Map<Matrix> obsPoints(obsPointsData.data(), 2, _numPoints);

        _translations.row(n) = obsPoints.array().rowwise().mean();

        _referencePoints = _referencePoints.array() + (obsPoints.array().colwise() - _translations.transpose().col(n).array());
    }
    _referencePoints.array() /= double(N);

    bool solveSuccessful = Solve();

    assert (solveSuccessful);
}

void RigidMotionEstimator::Initialise()
{
    const int N = _Y.rows();

    for (int n = 0; n < N; ++n)
    {
        Matrix obsPointsData(_Y.row(n));
        Eigen::Map<Matrix> obsPoints(obsPointsData.data(), 2, _numPoints);

        _translations.row(n) = obsPoints.array().rowwise().mean();

        _referencePoints = _referencePoints.array() + (obsPoints.array().colwise() - _translations.transpose().col(n).array());
    }
    _referencePoints.array() /= double(N);
}

bool RigidMotionEstimator::Solve()
{
    Initialise();

    const int N = _Y.rows();

    // Transpose for ColMajor ordering..
    Matrix tmpTrans(_translations.transpose());

    ceres::Problem problem;

    for (int n = 0; n < N; ++n)
    {
        Matrix obsPointsData(_Y.row(n));
        Eigen::Map<Matrix> obsPoints(obsPointsData.data(), 2, _numPoints);

        for (int i = 0; i < _numPoints; ++i)
        {
            problem.AddResidualBlock(RigidBodyResidual::create(obsPoints.col(i)),
                                     NULL,
                                     tmpTrans.data() + n * 2,
                                     _rotations.data() + n,
                                     _referencePoints.data() + i * 2);
        }
    }

    // Run the solver!
    ceres::Solver::Options options;

    options.minimizer_progress_to_stdout = true;

    options.function_tolerance = 1e-9;

    options.max_num_iterations = 500;

    std::string errorMsg;
    if (!options.IsValid(&errorMsg))
    {
        std::cerr << errorMsg << std::endl;
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";

    _translations = tmpTrans.transpose();

    vdbg(summary.IsSolutionUsable());

    assert (summary.IsSolutionUsable());

    return summary.IsSolutionUsable();
}

Matrix RigidMotionEstimator::CalcNormalisedY() const
{
    const int N = _Y.rows();
    const int D = _Y.cols();

    // The normalised matrix..
    Matrix U(N, D);

    for (int n = 0; n < N; ++n)
    {
        auto YPoints(MatrixRowMapper(_Y, n, 2, _numPoints));
        auto UPoints(MatrixRowMapper(U, n, 2, _numPoints));

        Eigen::Matrix2d Q = GetRotationMatrix(n);

        UPoints = YPoints.colwise() - _translations.transpose().col(n);

        UPoints = Q.transpose() * UPoints;
    }

    return U;
}

void RigidMotionEstimator::SaveToFile(const std::string matlabFilename) const
{
    (void) matlabFilename;
}
