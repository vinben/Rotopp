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

#ifndef TRANSFORMATIONS_H
#define TRANSFORMATIONS_H

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "ceres/solver.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <Eigen/Eigenvalues>

typedef Eigen::MatrixXd Matrix;
typedef Eigen::VectorXd Vector;

#include "baseDefs.hpp"

template <typename T>
Eigen::Matrix<T, 2, 2> RotationMatrixFromAngle(const T& alpha)
{
    Eigen::Matrix<T, 2, 2> Q;
    Q << cos(alpha), -sin(alpha), sin(alpha), cos(alpha);
    return Q;
}

template <typename T>
Eigen::Matrix<T, 2, 2> RotationMatrixDerivativeFromAngle(const T& alpha)
{
    Eigen::Matrix<T, 2, 2> Q;
    Q << -sin(alpha), -cos(alpha), cos(alpha), -sin(alpha);
    return Q;
}

struct RigidMotionEstimator
{
private:
    Eigen::MatrixXd _Y;
    Eigen::MatrixXd _translations;
    Eigen::VectorXd _rotations;
    Eigen::MatrixXd _referencePoints;
    int _numPoints;

public:
    RigidMotionEstimator(const Eigen::MatrixXd& Y);

    Eigen::MatrixXd CalcNormalisedY() const;

    void SaveToFile(const std::string matlabFilename) const;

    const Eigen::MatrixXd& translations() const
    {
        return _translations;
    }

    const Eigen::VectorXd& rotations() const
    {
        return _rotations;
    }

    const Eigen::MatrixXd& referencePoints() const
    {
        return _referencePoints;
    }

    int numPoints() //const
    {
        return _numPoints;
    }

    Eigen::Matrix2d GetRotationMatrix(int index) const
    {
        nassert ((index >=0) && (index < _Y.rows()));

        return Eigen::Matrix2d(RotationMatrixFromAngle(_rotations[index]));
    }

private:
    void Initialise();

    bool Solve();

    void testRotation() const
    {
        const double alpha = 0.5;
        Eigen::Matrix2d Q(RotationMatrixFromAngle(alpha));
        vdbg(Q);
    }

};

#endif // TRANSFORMATIONS_H
