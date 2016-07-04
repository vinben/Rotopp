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

#include "include/rotoSolver/planarTrackerSolver.hpp"

DECLARE_bool(testGradients);

using namespace gflags;

struct RotationSmoothnessPrior : public ceres::CostFunction
{
private:
    static const int NUM_PARAM_BLOCKS = 2;
    const shared_ptr<double> _scalingFactorPtr;

public:
    RotationSmoothnessPrior(const shared_ptr<double>& scalingFactor) :
        _scalingFactorPtr(scalingFactor)
    {
        nassert ((*_scalingFactorPtr) > 0);

        // Set number of parameters:
        std::vector<ceres::int32>& param_block_sizes = *(mutable_parameter_block_sizes());

        param_block_sizes.clear();
        param_block_sizes.reserve(NUM_PARAM_BLOCKS);

        for (int n = 0; n < NUM_PARAM_BLOCKS; ++n)
        {
            param_block_sizes.push_back(1);
        }

        nassert (param_block_sizes.size() == NUM_PARAM_BLOCKS);

        const int numResiduals = 2;

        set_num_residuals(numResiduals);
    }

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const
    {
        const double scalingFactor = (*_scalingFactorPtr);

        residuals[0] = scalingFactor * (sin(parameters[0][0]) - sin(parameters[1][0]));
        residuals[1] = scalingFactor * (cos(parameters[0][0]) - cos(parameters[1][0]));

        if (jacobians != NULL)
        {
            if (jacobians[0] != NULL)
            {
                jacobians[0][0] = scalingFactor * cos(parameters[0][0]);
                jacobians[0][1] = scalingFactor * -sin(parameters[0][0]);
            }
            if (jacobians[1] != NULL)
            {
                jacobians[1][0] = scalingFactor * -cos(parameters[1][0]);
                jacobians[1][1] = scalingFactor * sin(parameters[1][0]);
            }
        }

        return true;
    }
};

struct TranslationSmoothnessPrior : public ceres::CostFunction
{
private:
    static const int NUM_PARAM_BLOCKS = 2;
    const shared_ptr<double> _scalingFactorPtr;

public:
    TranslationSmoothnessPrior(const shared_ptr<double>& scalingFactor) :
        _scalingFactorPtr(scalingFactor)
    {
        nassert ((*_scalingFactorPtr) > 0);

        // Set number of parameters:
        std::vector<ceres::int32>& param_block_sizes = *(mutable_parameter_block_sizes());

        param_block_sizes.clear();
        param_block_sizes.reserve(NUM_PARAM_BLOCKS);

        for (int n = 0; n < NUM_PARAM_BLOCKS; ++n)
        {
            param_block_sizes.push_back(2);
        }

        nassert (param_block_sizes.size() == NUM_PARAM_BLOCKS);

        const int numResiduals = 2;

        set_num_residuals(numResiduals);
    }

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const
    {
        const double scalingFactor = (*_scalingFactorPtr);

        residuals[0] = scalingFactor * (parameters[0][0] - parameters[1][0]);
        residuals[1] = scalingFactor * (parameters[0][1] - parameters[1][1]);

        if (jacobians != NULL) {
            if (jacobians[0] != NULL) {
                Eigen::Map<PlanarTrackSolve::ColMatrix> J(jacobians[0], 2, 2);
                J = PlanarTrackSolve::ColMatrix::Identity(2, 2) * scalingFactor;
            }
            if (jacobians[1] != NULL)
            {
                Eigen::Map<PlanarTrackSolve::ColMatrix> J(jacobians[1], 2, 2);
                J = PlanarTrackSolve::ColMatrix::Identity(2, 2) * -scalingFactor;
            }
        }

        return true;
    }
};

struct ScaleSmoothnessPrior : public ceres::CostFunction
{
private:
    static const int NUM_PARAM_BLOCKS = 2;
    const shared_ptr<double> _scalingFactorPtr;

public:
    ScaleSmoothnessPrior(const shared_ptr<double>& scalingFactor) :
        _scalingFactorPtr(scalingFactor)
    {
        nassert ((*_scalingFactorPtr) > 0);

        // Set number of parameters:
        std::vector<ceres::int32>& param_block_sizes = *(mutable_parameter_block_sizes());

        param_block_sizes.clear();
        param_block_sizes.reserve(NUM_PARAM_BLOCKS);

        for (int n = 0; n < NUM_PARAM_BLOCKS; ++n)
        {
            param_block_sizes.push_back(1);
        }

        nassert (param_block_sizes.size() == NUM_PARAM_BLOCKS);

        const int numResiduals = 1;

        set_num_residuals(numResiduals);
    }

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const
    {
        const double scalingFactor = (*_scalingFactorPtr);

        residuals[0] = scalingFactor * (parameters[0][0] - parameters[1][0]);

        if (jacobians != NULL)
        {
            if (jacobians[0] != NULL)
            {
                jacobians[0][0] = scalingFactor;
            }
            if (jacobians[1] != NULL)
            {
                jacobians[1][0] = - scalingFactor;
            }
        }

        return true;
    }
};


struct ManifoldSmoothnessPrior : public ceres::CostFunction
{
private:
    static const int NUM_PARAM_BLOCKS = 2;
    const int _Q;
    const shared_ptr<double> _scalingFactorPtr;

public:
    ManifoldSmoothnessPrior(const int Q, const shared_ptr<double>& scalingFactor) :
        _Q(Q), _scalingFactorPtr(scalingFactor)
    {
        nassert (_Q > 0);
        nassert ((*_scalingFactorPtr) > 0);

        // Set number of parameters:
        std::vector<ceres::int32>& param_block_sizes = *(mutable_parameter_block_sizes());

        param_block_sizes.clear();
        param_block_sizes.reserve(NUM_PARAM_BLOCKS);

        for (int n = 0; n < NUM_PARAM_BLOCKS; ++n)
        {
            param_block_sizes.push_back(_Q);
        }

        nassert (param_block_sizes.size() == NUM_PARAM_BLOCKS);

        const int numResiduals = _Q;

        set_num_residuals(numResiduals);
    }

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const
    {
        const double scalingFactor = (*_scalingFactorPtr);

        for (int q = 0; q < _Q; ++q)
        {
            residuals[q] = scalingFactor * (parameters[0][q] - parameters[1][q]);
        }

        if (jacobians != NULL)
        {
            if (jacobians[0] != NULL)
            {
                Eigen::Map<PlanarTrackSolve::ColMatrix> J(jacobians[0], _Q, _Q);
                J = PlanarTrackSolve::ColMatrix::Identity(_Q, _Q) * scalingFactor;
            }
            if (jacobians[1] != NULL)
            {
                Eigen::Map<PlanarTrackSolve::ColMatrix> J(jacobians[1], _Q, _Q);
                J = PlanarTrackSolve::ColMatrix::Identity(_Q, _Q) * -scalingFactor;
            }
        }

        return true;
    }
};

struct AnchorPrior : public ceres::CostFunction
{
private:
    int NUM_PARAM_BLOCKS;
    const int _Q;
    const shared_ptr<double> _scalingFactorPtr;
    Vector _anchorPoint;

public:
    AnchorPrior(const int Q,
                const Vector& anchorPoint,
                const shared_ptr<double>& scalingFactor) :
        _Q(Q), _scalingFactorPtr(scalingFactor), _anchorPoint(anchorPoint)
    {
        nassert (_Q > 0);
        nassert ((*_scalingFactorPtr) > 0);
        nassert (_anchorPoint.size() == _Q);
        NUM_PARAM_BLOCKS = 1;
        // Set number of parameters:
        std::vector<ceres::int32>& param_block_sizes = *(mutable_parameter_block_sizes());

        param_block_sizes.clear();
        param_block_sizes.reserve(NUM_PARAM_BLOCKS);

        for (int n = 0; n < NUM_PARAM_BLOCKS; ++n)
        {
            param_block_sizes.push_back(_Q);
        }

        nassert (param_block_sizes.size() == NUM_PARAM_BLOCKS);

        const int numResiduals = _Q;

        set_num_residuals(numResiduals);
    }

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const
    {
        const double scalingFactor = (*_scalingFactorPtr);

        for (int q = 0; q < _Q; ++q)
        {
            residuals[q] = scalingFactor * (parameters[0][q] - _anchorPoint[q]);
        }

        if (jacobians != NULL)
        {
            if (jacobians[0] != NULL)
            {
                Eigen::Map<PlanarTrackSolve::ColMatrix> J(jacobians[0], _Q, _Q);
                J = PlanarTrackSolve::ColMatrix::Identity(_Q, _Q) * scalingFactor;
            }
        }

        return true;
    }
};

struct ResidualForFrame : public ceres::CostFunction
{
private:
    const int _Q;
    const std::vector<PlanarTrackingData>& _targetTracks;
    const int _frameIdx;
    shared_ptr<GPLVM> _gplvm;
    const Vector& _normalisationWeight;

public:
    ResidualForFrame(const int Q,
                     const std::vector<PlanarTrackingData>& targetTracks,
                     const int frameIdx,
                     const shared_ptr<GPLVM>& gplvm,
                     const Vector& normalisationWeight) :
        _Q(Q),
        _targetTracks(targetTracks),
        _frameIdx(frameIdx),
        _gplvm(gplvm),
        _normalisationWeight(normalisationWeight)
    {
        // Set number of parameters:
        std::vector<ceres::int32>& param_block_sizes = *(mutable_parameter_block_sizes());

        param_block_sizes.clear();

        /*
         * Parameters for:
         * [0] = manifold location U (Q)
         * [1] = translation (2)
         * [2] = rotation (1)
         * [3] = scale (1)
         */
        param_block_sizes.push_back(_Q);
        param_block_sizes.push_back(2);
        param_block_sizes.push_back(1);
        param_block_sizes.push_back(1);

        nassert (param_block_sizes.size() == 4);

        int numResiduals = 0;

        // Calc num residuals..

        for(int i = 0; i < _targetTracks.size(); i++){
            numResiduals += _targetTracks[i].PointIDs.size();
        }

        set_num_residuals(numResiduals);
    }

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const
    {
        // Parameter matrix in the FIRST parameter block..
        const Eigen::Map<Matrix> U(const_cast<double*>(parameters[0]), 1, _Q);

        const Eigen::Map<const Eigen::Vector2d> translation(parameters[1]);
        const double& alpha = *parameters[2];
        const double& scale = *parameters[3];

        Eigen::Map<Vector> R(residuals, num_residuals());

        std::vector<Matrix> dy_dx_vec;
        std::vector<Matrix>* dy_dx_vec_ptr = NULL;
        if ((jacobians != NULL) && (jacobians[0] != NULL))
        {
            dy_dx_vec_ptr = &dy_dx_vec;
        }

        Matrix Y = _gplvm->Predict(U, NULL, dy_dx_vec_ptr);

        Matrix Y_orig(Y);

        assert (Y.rows() == 1);

        // Apply transformation..

        auto YPoints(MatrixRowMapper(Y, 0, 2, Y.cols()/2));

        const Eigen::Matrix2d RMat(RotationMatrixFromAngle<double>(alpha));

        YPoints.array() *= scale;
        YPoints = RMat * YPoints;
        YPoints = YPoints.colwise() + translation;

        // Y should now be correctly transformed..

        int currResidualIdx = 0;
        for(int i = 0; i < _targetTracks.size(); i++)
        {
            PlanarTrackingData ptd = _targetTracks[i];
            const double frameWeight = ptd.FrameWeights[_frameIdx];
            for(int iPt = 0; iPt < ptd.PointIDs.size(); iPt ++){
                int ptID = ptd.PointIDs[iPt];
                if (ptd.TrackingData(_frameIdx, iPt) > 0)
                {
                    R[currResidualIdx] = Y(0, ptID) - ptd.TrackingData(_frameIdx, iPt);
                    R[currResidualIdx] *= frameWeight / _normalisationWeight[ptID];//ptd.TrackingData(_frameIdx, idx);
                }
                else
                {
                    R[currResidualIdx] = 0.0;
                }
                ++currResidualIdx;
            }
        }

        assert (currResidualIdx == num_residuals());

        if (jacobians != NULL)
        {
            assert (parameter_block_sizes().size() == 4);

            // Check for jacobians for manifold..
            if (jacobians[0] != NULL)
            {
                Eigen::Map<Matrix> J(jacobians[0], _Q, num_residuals());
                Matrix& dy_dx = dy_dx_vec[0];

                nassert (dy_dx.cols() == Y.cols());
                nassert (dy_dx.rows() == _Q);

                // Transform the derivatives..
                for (int q = 0; q < _Q; ++q)
                {
                    auto dy_dx_pts(MatrixRowMapper(dy_dx, q, 2, dy_dx.cols()/2));
                    dy_dx_pts = RMat * dy_dx_pts;
                }

                int currResidualIdx = 0;

                for(int i = 0; i < _targetTracks.size(); i++)
                {
                    PlanarTrackingData ptd = _targetTracks[i];
                    const double frameWeight = ptd.FrameWeights[_frameIdx];


                    for(int idx = 0; idx < ptd.PointIDs.size(); idx++){
                        int ptID = ptd.PointIDs[idx];
                        if (ptd.TrackingData(_frameIdx, idx) > 0)
                        {
                            for (int q = 0; q < _Q; ++q)
                            {
                                J(q, currResidualIdx) = dy_dx(q, ptID);
                                J(q, currResidualIdx) *= frameWeight / _normalisationWeight[ptID];
                            }
                        }
                        else
                        {
                            for (int q = 0; q < _Q; ++q)
                            {
                                J(q, currResidualIdx) = 0.0;
                            }
                        }

                        ++currResidualIdx;
                    }
                }

                assert (currResidualIdx == num_residuals());
            }

            // Check for jacobians for translation..
            if (jacobians[1] != NULL)
            {
                Eigen::Map<Matrix> J(jacobians[1], 2, num_residuals());

                int currResidualIdx = 0;
                for(int i = 0; i < _targetTracks.size(); i++)
                {
                    PlanarTrackingData ptd = _targetTracks[i];
                    const double frameWeight = ptd.FrameWeights[_frameIdx];

                    for(int idx = 0; idx < ptd.PointIDs.size(); idx++){
                        int ptID = ptd.PointIDs[idx];
                        if (ptd.TrackingData(_frameIdx, idx) > 0)
                        {
                            if ((ptID % 2) == 0)
                            {
                                J(0, currResidualIdx) = 1.0 * frameWeight / _normalisationWeight[ptID];
                                J(1, currResidualIdx) = 0.0;
                            }
                            else
                            {
                                J(0, currResidualIdx) = 0.0;
                                J(1, currResidualIdx) = 1.0 * frameWeight / _normalisationWeight[ptID];
                            }
                        }
                        else
                        {
                            J(0, currResidualIdx) = 0.0;
                            J(1, currResidualIdx) = 0.0;
                        }

                        ++currResidualIdx;
                    }
                }

                assert (currResidualIdx == num_residuals());
            }

            // Check for jacobians for rotation..
            if (jacobians[2] != NULL)
            {
                Eigen::Map<Matrix> J(jacobians[2], 1, num_residuals());

                const Eigen::Matrix2d RMat_dalpha(RotationMatrixDerivativeFromAngle<double>(alpha));

                Matrix YY(Y_orig);

                auto YY_pts(MatrixRowMapper(YY, 0, 2, YY.cols()/2));
                YY_pts = RMat_dalpha * YY_pts;

                int currResidualIdx = 0;
                for(int i = 0; i < _targetTracks.size(); i++)
                {
                    PlanarTrackingData ptd = _targetTracks[i];
                    const double frameWeight = ptd.FrameWeights[_frameIdx];

                    for(int idx = 0; idx < ptd.PointIDs.size(); idx++){
                        int ptID = ptd.PointIDs[idx];
                        if (ptd.TrackingData(_frameIdx, idx) > 0)
                        {
                            J(0, currResidualIdx) = YY(0, ptID);
                            J(0, currResidualIdx) *= frameWeight / _normalisationWeight[ptID];
                        }
                        else
                        {
                            J(0, currResidualIdx) = 0.0;
                        }

                        ++currResidualIdx;
                    }
                }

                assert (currResidualIdx == num_residuals());
            }

            // Check for jacobians for scale..
            if (jacobians[3] != NULL)
            {
                Eigen::Map<Matrix> J(jacobians[3], 1, num_residuals());

                Matrix YY(Y_orig);

                auto YY_pts(MatrixRowMapper(YY, 0, 2, YY.cols()/2));
                YY_pts = RMat * YY_pts;

                int currResidualIdx = 0;
                for(int i = 0; i < _targetTracks.size(); i++)
                {
                    PlanarTrackingData ptd = _targetTracks[i];
                    const double frameWeight = ptd.FrameWeights[_frameIdx];

                    for(int idx = 0; idx < ptd.PointIDs.size(); idx++){
                        int ptID = ptd.PointIDs[idx];
                        if (ptd.TrackingData(_frameIdx, idx) > 0) {
                            J(0, currResidualIdx) = YY(0, ptID);
                            J(0, currResidualIdx) *= frameWeight / _normalisationWeight[ptID];
                        }
                        else
                        {
                            J(0, currResidualIdx) = 0.0;
                        }

                        ++currResidualIdx;
                    }
                }

                assert (currResidualIdx == num_residuals());
            }
        }

        return true;
    }
};

PlanarTrackSolve::PlanarTrackSolve(shared_ptr<GPLVM> gplvm,
                                   const Matrix& U,
                                   const std::vector<PlanarTrackingData>& targetTracks,
                                   const int startIdx, const int endIdx,
                                   const PlanarTrackingData& keyFrames,
                                   const std::vector<int>& keyFramesIndex,
                                   shared_ptr<RigidMotionEstimator> motionEstimator) :
    _gplvm(gplvm),
    _U(U),
    _N(U.rows()),
    _Q(U.cols()),
    _startIdx(startIdx),
    _endIdx(endIdx),
    _keyFrames(keyFrames),
    _keyFramesIndex(keyFramesIndex),
    _targetTracks(targetTracks),
    _normalisationWeight(1.0),
    _motionEstimator(motionEstimator),
    _residualLossFunctionPtr(new ceres::LossFunctionWrapper(new ceres::TrivialLoss, ceres::TAKE_OWNERSHIP))
{
    nassert (_N > 0);
    nassert (_Q > 0);

    nassert ((_endIdx - _startIdx + 1) == _N);

    nassert (_targetTracks.size() > 0);

    Matrix Y = _gplvm->Predict(_U);

    _normalisationWeight = Y.array().colwise().mean();
    _normalisationWeight = _normalisationWeight.array().abs();

    _normalisationWeight = _normalisationWeight.array().max(1e-12);
    _normalisationWeight.setConstant(double(Y.cols()));
    _translations = RowMatrix::Zero(_N, 2);
    _rotations = Vector::Zero(_N);

    // Set scales to 1
    _scales = Vector::Ones(_N);

    nassert (_rotations.cols() == 1);

    if (UsingRotationAndTranslation())
    {
        InitRotationAndTranslation();
    }

    CropTargetTracks();
}

void PlanarTrackSolve::CropTargetTracks(int width, int height)
{
    int numCropped = 0;
    const double outOfRangeValue = -1.0;
    int check = 0;

    for(int i = 0; i < _targetTracks.size(); i++)
    {
        PlanarTrackingData ptd = _targetTracks[i];
        nassert (remainder(ptd.TrackingData.cols(), 2) == 0);

        const int P = ptd.TrackingData.cols() / 2;
        for (int i = 0, I = ptd.TrackingData.rows(); i < I; ++i)
        {
            auto pts(MatrixRowMapper(ptd.TrackingData, i, 2, P));

            for (int p = 0; p < P; ++p)
            {
                if ((pts(0, p) > width) || (pts(1, p) > height))
                {
                    pts(0, p) = outOfRangeValue;
                    pts(1, p) = outOfRangeValue;
                    ++numCropped;
                }
            }
        }
    }

    for(int i = 0; i < _targetTracks.size(); i++)
    {
        PlanarTrackingData ptd = _targetTracks[i];
        check += (ptd.TrackingData.array() == -1).sum();
    }
}

void PlanarTrackSolve::InitRotationAndTranslation()
{
    Matrix sparseT = _motionEstimator->translations();
    Vector sparstR = _motionEstimator->rotations();

    nassert (sparseT.rows() == _keyFramesIndex.size());
    nassert (0 == *std::min_element(_keyFramesIndex.begin(), _keyFramesIndex.end()));
    nassert ((_N-1) == *std::max_element(_keyFramesIndex.begin(), _keyFramesIndex.end()));

    std::vector<int> keys(_keyFramesIndex.begin(), _keyFramesIndex.end());
    std::sort(_keyFramesIndex.begin(), _keyFramesIndex.end());

    for(int i = 0; i < keys.size(); i++){
        int k = keys[i];
        _translations.row(k) = sparseT.row(i);
        _rotations[k] = sparstR[i];
    }

    // Do linear interpolation for init..
    for (int i = 0, I = keys.size() - 1; i < I; ++i)
    {
        const int a = keys[i];
        const int b = keys[i+1];

        Interpolator<int> interpT(a, b, _translations.row(a), _translations.row(b));
        Interpolator<int> interpR(a, b, _rotations.row(a), _rotations.row(b));

        for (int k = a+1; k < b; ++k)
        {
            _translations.row(k) = interpT.get(k);
            _rotations.row(k) = interpR.get(k);
        }
    }
}

void PlanarTrackSolve::SpecialTest()
{
    {
        shared_ptr<double> sfPtr(new double(1.0));
        RotationSmoothnessPrior rp(sfPtr);

        double** params = new double*[rp.parameter_block_sizes().size()];
        nassert (rp.parameter_block_sizes().size() == 2);
        nassert (rp.parameter_block_sizes()[0] == 1);
        nassert (rp.parameter_block_sizes()[1] == 1);
        params[0] = getFrameParameterPointer(_rotations, 0);
        params[1] = getFrameParameterPointer(_rotations, 1);

        vdbg(_rotations.transpose());
        vdbg(*params[0]);
        vdbg(*params[1]);

        double** jacobians = new double*[rp.parameter_block_sizes().size()];
        jacobians[0] = new double[rp.num_residuals()];

    }
    return;

    ceres::Problem problem;

    for (int frameIdx = 0; frameIdx < _N; ++frameIdx)
    {
        problem.AddParameterBlock(getFrameParameterPointer(_U, frameIdx), _Q);
        problem.AddParameterBlock(getFrameParameterPointer(_translations, frameIdx), 2);
        problem.AddParameterBlock(getFrameParameterPointer(_rotations, frameIdx), 1);
    }

    const int frameIdx = 0;

    problem.AddResidualBlock(new ResidualForFrame(_Q, _targetTracks, frameIdx,
                                                  _gplvm, _normalisationWeight),
                             NULL, getFrameParameterPointer(_U, frameIdx),
                             getFrameParameterPointer(_translations, frameIdx),
                             getFrameParameterPointer(_rotations, frameIdx));

    // Run the solver!
    Solver::Options options;

    options.check_gradients = true;

    options.num_threads = 8;

    options.function_tolerance = 1e-5;

    options.max_num_iterations = 500;

    std::string errorMsg;
    if (!options.IsValid(&errorMsg))
    {
        std::cerr << errorMsg << std::endl;
    }

    Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    vdbg(summary.IsSolutionUsable());

    vdbg(_U.transpose());
    vdbg(_translations.transpose());
    vdbg(_rotations.transpose());
}



bool PlanarTrackSolve::RunSolve()
{
    ceres::Problem problem;

    for (int frameIdx = 0; frameIdx < _N; ++frameIdx)
    {
        problem.AddParameterBlock(getFrameParameterPointer(_U, frameIdx), _Q);
        problem.AddParameterBlock(getFrameParameterPointer(_translations, frameIdx), 2);
        problem.AddParameterBlock(getFrameParameterPointer(_rotations, frameIdx), 1);
        problem.AddParameterBlock(getFrameParameterPointer(_scales, frameIdx), 1);

        problem.SetParameterLowerBound(getFrameParameterPointer(_scales, frameIdx), 0, 0.5);
        problem.SetParameterUpperBound(getFrameParameterPointer(_scales, frameIdx), 0, 2.0);
    }

    ceres::LossFunction* residualLossFunction = NULL;

    bool usingHuberLoss = (FLAGS_r > 0.0);
    vdbg(usingHuberLoss);
    if (usingHuberLoss)
    {
        const double huberLossParameter = FLAGS_r;
        vdbg(huberLossParameter);

        residualLossFunction = new ceres::ArctanLoss(huberLossParameter);

        std::cout << "\n\nUSING ArctanLoss !!!\n\n\n";
    }

    // Add matches to targets..
    for (int frameIdx = 0; frameIdx < _N; ++frameIdx)
    {
        problem.AddResidualBlock(new ResidualForFrame(_Q, _targetTracks, frameIdx,
                                                      _gplvm, _normalisationWeight),
                                 _residualLossFunctionPtr,
                                 getFrameParameterPointer(_U, frameIdx),
                                 getFrameParameterPointer(_translations, frameIdx),
                                 getFrameParameterPointer(_rotations, frameIdx),
                                 getFrameParameterPointer(_scales, frameIdx));
    }

    const double priorScalingFactor = FLAGS_s;
    const double anchorScalingFactor = FLAGS_a;

    vdbg(priorScalingFactor);

    const double rotationScalingFactor = FLAGS_rot_weight;
    double translationScalingFactor = FLAGS_trans_weight;
    const double scaleScalingFactor = FLAGS_scale_weight;

    vdbg(translationScalingFactor);

    if (UsingRotationAndTranslation())
    {
        double translationNormFactor = ( ( _translations.array().rowwise() -
                                           _translations.array().colwise().mean() ).pow(2).colwise().sum() /
                                         (double(_translations.rows()) - 1.0)
                                         ).sqrt().mean();
        translationNormFactor = std::max(translationNormFactor, 1e-6);
        vdbg(translationNormFactor);
        translationScalingFactor /= translationNormFactor;
    }

    vdbg(rotationScalingFactor);
    vdbg(translationScalingFactor);
    vdbg(scaleScalingFactor);

    shared_ptr<double> rotationScalingFactorPtr(new double(rotationScalingFactor));
    shared_ptr<double> translationScalingFactorPtr(new double(translationScalingFactor));
    shared_ptr<double> scaleScalingFactorPtr(new double(scaleScalingFactor));

    // Add manifold prior..
    shared_ptr<double> manifoldScalingFactorPtr(new double(priorScalingFactor));
    for (int i = 0, I = (_N - 1); i < I; ++i)
    {
        if (*manifoldScalingFactorPtr > 0)
        {
            problem.AddResidualBlock(new ManifoldSmoothnessPrior(_Q, manifoldScalingFactorPtr),
                                     new ceres::HuberLoss(0.1), getFrameParameterPointer(_U, i), getFrameParameterPointer(_U, i+1));
        }

        if (*rotationScalingFactorPtr > 0)
        {
            problem.AddResidualBlock(new RotationSmoothnessPrior(rotationScalingFactorPtr),
                                     NULL, getFrameParameterPointer(_rotations, i),
                                     getFrameParameterPointer(_rotations, i+1));
        }

        if (*translationScalingFactorPtr > 0)
        {
            problem.AddResidualBlock(new TranslationSmoothnessPrior(translationScalingFactorPtr),
                                     NULL, getFrameParameterPointer(_translations, i),
                                     getFrameParameterPointer(_translations, i+1));
        }

        if (*scaleScalingFactorPtr > 0)
        {
            problem.AddResidualBlock(new ScaleSmoothnessPrior(scaleScalingFactorPtr),
                                     NULL, getFrameParameterPointer(_scales, i),
                                     getFrameParameterPointer(_scales, i+1));
        }
    }

    // Add anchor prior for keyframes..
    Matrix manifoldAnchorLocations = _gplvm->GetX();
    nassert (manifoldAnchorLocations.rows() == _keyFramesIndex.size());

    shared_ptr<double> anchorScalingFactorPtr(new double(anchorScalingFactor));

    if (*anchorScalingFactorPtr > 0)
    {
        nassert (manifoldAnchorLocations.rows() == _keyFramesIndex.size());

        for(int i = 0; i < _keyFramesIndex.size(); i++){
            int k = _keyFramesIndex[i];
            problem.AddResidualBlock(new AnchorPrior(_Q, manifoldAnchorLocations.row(i), anchorScalingFactorPtr),
                                     NULL, getFrameParameterPointer(_U, k));

            problem.AddResidualBlock(new AnchorPrior(2, _translations.row(k), anchorScalingFactorPtr),
                                     NULL, getFrameParameterPointer(_translations, k));
            problem.AddResidualBlock(new AnchorPrior(1, _rotations.row(k), anchorScalingFactorPtr),
                                     NULL, getFrameParameterPointer(_rotations, k));
            problem.AddResidualBlock(new AnchorPrior(1, _scales.row(k), anchorScalingFactorPtr),
                                     NULL, getFrameParameterPointer(_scales, k));
        }
    }

    // Run the solver!
    Solver::Options options;

    options.check_gradients = FLAGS_testGradients;

    options.num_threads = 8;

    options.function_tolerance = 1e-8;

    options.max_num_iterations = 500;

    std::string errorMsg;
    if (!options.IsValid(&errorMsg))
    {
        std::cerr << errorMsg << std::endl;
    }

    Solver::Summary summary;

    if (!FLAGS_skip_initial_optimisation)
    {

        for (int frameIdx = 0; frameIdx < _N; ++frameIdx)
        {
            problem.SetParameterBlockConstant(getFrameParameterPointer(_U, frameIdx));
        }

        ceres::Solve(options, &problem, &summary);

        std::cout << summary.BriefReport() << std::endl;

        for (int frameIdx = 0; frameIdx < _N; ++frameIdx) {
            problem.SetParameterBlockVariable(getFrameParameterPointer(_U, frameIdx));
        }

        ceres::Solve(options, &problem, &summary);

        std::cout << summary.BriefReport() << std::endl;
    }

    // Set the final loss function..
    _residualLossFunctionPtr->Reset(residualLossFunction, ceres::TAKE_OWNERSHIP);
    ceres::Solve(options, &problem, &summary);

    if (false)
    {
        std::cout << summary.BriefReport() << std::endl;
        const double finalArcTanLoss = 0.5;
        vdbg(finalArcTanLoss);
        _residualLossFunctionPtr->Reset(new ceres::ArctanLoss(finalArcTanLoss), ceres::TAKE_OWNERSHIP);
        ceres::Solve(options, &problem, &summary);
    }
    return summary.IsSolutionUsable();
}

DECLARE_bool(overrideKeys);

Matrix PlanarTrackSolve::GetOutput(Vector* gplvmVariance) const
{
    Matrix Y = _gplvm->Predict(_U, gplvmVariance);

    if (UsingRotationAndTranslation())
    {
        // Apply transformations..
        nassert (remainder(Y.cols(), 2) == 0);
        const int numPoints = Y.cols() / 2;

        for (int n = 0; n < _N; ++n)
        {
            auto YPoints(MatrixRowMapper(Y, n, 2, numPoints));
            Eigen::Matrix2d RMat = RotationMatrixFromAngle(_rotations[n]);

            YPoints.array() *= _scales[n];
            YPoints = RMat * YPoints;
            YPoints.colwise() += _translations.transpose().col(n);
        }
    }

    const int startIndex = _keyFrames.StartIndex;

    const bool overrideKeyFrames = FLAGS_overrideKeys;
    if (!overrideKeyFrames) {
        for (int i = 0; i < _keyFrames.KeyFrames.size(); ++i) {
            const int k = _keyFrames.KeyFrames[i];
            Y.row(k-startIndex) = _keyFrames.TrackingData.row(i);
        }
    }

    return Y;
}

Matrix PlanarTrackSolve::GetOutputLinearInterp(Vector* gplvmVariance) const
{
    (void) gplvmVariance;

    Matrix Ykey = _gplvm->GetY();
    Matrix Y(_N, Ykey.cols());

    const int startIndex = _keyFrames.StartIndex;

    for (int i = 0, I = _keyFrames.KeyFrames.size() - 1; i < I; ++i)
    {
        const int a = _keyFrames.KeyFrames[i];
        const int b = _keyFrames.KeyFrames[i+1];

        Interpolator<int> interp(a, b, Ykey.row(i), Ykey.row(i+1));

        Y.row(a-startIndex) = Ykey.row(i);
        Y.row(b-startIndex) = Ykey.row(i+1);
        for (int k = a+1; k < b; ++k)
        {
            Y.row(k-startIndex) = interp.get(k);
        }
    }

    if (UsingRotationAndTranslation())
    {
        // Apply transformations..
        nassert (remainder(Y.cols(), 2) == 0);
        const int numPoints = Y.cols() / 2;

        for (int n = 0; n < _N; ++n)
        {
            auto YPoints(MatrixRowMapper(Y, n, 2, numPoints));
            Eigen::Matrix2d RMat = RotationMatrixFromAngle(_rotations[n]);

            YPoints.array() *= _scales[n];
            YPoints = RMat * YPoints;
            YPoints.colwise() += _translations.transpose().col(n);
        }
    }

    const bool overrideKeyFrames = FLAGS_overrideKeys;
    if (!overrideKeyFrames) {
        for (int i = 0; i < _keyFrames.KeyFrames.size(); ++i) {
            const int k = _keyFrames.KeyFrames[i];
            Y.row(k-startIndex) = _keyFrames.TrackingData.row(i);
        }
    }

    return Y;
}

Vector PlanarTrackSolve::CalcConfidence(const Vector& gplvmVariance) const
{
    Vector confidence(Vector::Zero(_N));

    confidence = gplvmVariance.array().log();
    for(int i = 0; i < _keyFramesIndex.size(); i++)
    {
        int k = _keyFramesIndex[i];
        confidence[k] = confidence.minCoeff();
    }

    // Remap to 0 -> 1
    confidence = confidence.array() - confidence.array().minCoeff();
    confidence.array() /= confidence.array().maxCoeff();

    // Ensure confidence > 1 for keyframes..
    for(int i = 0; i < _keyFramesIndex.size(); i++)
    {
        int k = _keyFramesIndex[i];
        confidence[k] = 2.0;
    }

    return confidence;
}

void PlanarTrackSolve::SaveToFile(std::string textFilename,
                                  std::string confidenceOutputFile) const
{
    // Save to our output format:

    /*    # Number of points
          # tStart tEnd
          # Px
          # Py
          # Trx
          # Try
          # Tlx
          # Tly
    */
    shared_ptr<Vector> gplvmVariance;
    if (!confidenceOutputFile.empty()) {
        shared_ptr<Vector> tmp(new Vector(_N));
        gplvmVariance.swap(tmp);
    }

    Matrix Y = GetOutput(gplvmVariance.get());

    SaveSolverOutputFile(textFilename, Y, _startIdx, _endIdx);

    if (!confidenceOutputFile.empty())
    {
        std::ofstream ofs(confidenceOutputFile.c_str());

        Vector confidence = CalcConfidence((*gplvmVariance));

        ofs << _startIdx << " " << _endIdx << std::endl;
        ofs << confidence << std::endl;

        ofs.close();
    }
}


void PlanarTrackSolve::SaveToFileLinearInterp(std::string textFilename,
                                              std::string confidenceOutputFile) const
{
    // Save to our output format:

    /*    # Number of points
          # tStart tEnd
          # Px
          # Py
          # Trx
          # Try
          # Tlx
          # Tly
    */

    shared_ptr<Vector> gplvmVariance;
    if (!confidenceOutputFile.empty())
    {
        shared_ptr<Vector> tmp(new Vector(_N));
        gplvmVariance.swap(tmp);
    }

    Matrix Y = GetOutputLinearInterp(gplvmVariance.get());

    SaveSolverOutputFile(textFilename, Y, _startIdx, _endIdx);

    if (!confidenceOutputFile.empty())
    {
        std::ofstream ofs(confidenceOutputFile.c_str());

        Vector confidence = CalcConfidence((*gplvmVariance));

        ofs << _startIdx << " " << _endIdx << std::endl;
        ofs << confidence << std::endl;

        ofs.close();
    }
}
