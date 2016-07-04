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

#ifndef PLANARTRACKERSOLVER_HPP
#define PLANARTRACKERSOLVER_HPP

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "ceres/solver.h"
#include "gflags/gflags.h"
#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <Eigen/Eigenvalues>

#include <array>
#include <random>
#include <cmath>
#include <fstream>

#include "include/rotoSolver/gplvm.hpp"
#include "include/rotoSolver/fileUtils.hpp"
#include "include/rotoSolver/baseDefs.hpp"
#include "include/rotoSolver/eigenUtils.hpp"
#include "include/rotoSolver/transformations.hpp"

DECLARE_double(s);
DECLARE_double(a);
DECLARE_double(r);
DECLARE_double(trans_weight);
DECLARE_double(rot_weight);
DECLARE_double(scale_weight);
DECLARE_bool(skip_initial_optimisation);

struct PlanarTrackSolve
{
    typedef Eigen::MatrixXd Matrix;
    typedef Eigen::VectorXd Vector;

    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMatrix;
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> ColMatrix;

private:

    shared_ptr<GPLVM> _gplvm;

    // Very important that it's RowMajor here..
    RowMatrix _U;

    const int _N, _Q;
    const int _startIdx, _endIdx;

    PlanarTrackingData _keyFrames;

    std::vector<int> _keyFramesIndex;

    std::vector<PlanarTrackingData> _targetTracks;

    Vector _normalisationWeight;

    shared_ptr<RigidMotionEstimator> _motionEstimator;

    RowMatrix _translations;
    Vector _rotations;
    Vector _scales;

    ceres::LossFunctionWrapper* _residualLossFunctionPtr;

public:

    PlanarTrackSolve(shared_ptr<GPLVM> gplvm,
                     const Matrix& U,
                     const std::vector<PlanarTrackingData>& targetTracks,
                     const int startIdx, const int endIdx,
                     const PlanarTrackingData& keyFrames,
                     const std::vector<int>& keyFramesIndex,
                     shared_ptr<RigidMotionEstimator> motionEstimator = shared_ptr<RigidMotionEstimator>());

    void SpecialTest();

    bool RunSolve();

    Matrix GetOutput(Vector* gplvmVariance = NULL) const;
    Matrix GetOutputLinearInterp(Vector* gplvmVariance = NULL) const;

    void SaveToFile(std::string textFilename, std::string confidenceOutputFile = "") const;
    void SaveToFileLinearInterp(std::string textFilename, std::string confidenceOutputFile = "") const;

    bool UsingRotationAndTranslation() const
    {
        return (_motionEstimator.get() != 0);
    }

    const RowMatrix& GetU() const
    {
        return _U;
    }

private:

    Vector CalcConfidence(const Vector& gplvmVariance) const;

    void CropTargetTracks(int width = 1920, int height = 1080);

    void InitRotationAndTranslation();

    template <typename Mat>
    double* getFrameParameterPointer(Mat& M, const int frameIdx)
    {
        nassert (frameIdx >= 0);
        nassert (frameIdx < _N);
        double* ptr = M.data();
        ptr += (frameIdx * M.cols());
        return ptr;
    }

};

#endif // PLANARTRACKERSOLVER_HPP
