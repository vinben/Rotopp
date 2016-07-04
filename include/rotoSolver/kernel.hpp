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

#ifndef KERNEL_HPP
#define KERNEL_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SparseCore>

#include <string>
#include <iostream>
#include <cassert>
#include <memory>

#include "include/rotoSolver/baseDefs.hpp"



class Kernel {
public:
    typedef Eigen::MatrixXd Matrix;
    typedef Eigen::VectorXd Vector;
    typedef Eigen::SparseMatrix<double> SparseMatrix;
    typedef Eigen::MatrixXd ExtraDataType;

    virtual Kernel* createCopy() const = 0;

    virtual int NumberOfParameters() const = 0;

    virtual void SetParameters(const Vector& logParams) = 0;

    virtual void GetParameters(Vector& logParams) const = 0;

    virtual void Print() const = 0;

    virtual Matrix GetKernelMatrix(const Matrix& X, const Matrix& Z,
                                   ExtraDataType* extraDataPtr = NULL) const = 0;

    virtual Vector GetDiagonalOfKernel(const Matrix& X) const = 0;

    virtual std::vector< shared_ptr< Matrix > >
    GetGradientWrtParams(const Matrix& X, const Matrix& K,
                         ExtraDataType* extraDataPtr) const = 0;

    virtual shared_ptr< SparseMatrix >
    GetGradientWrtData(const Matrix& X, const Matrix& K,
                       ExtraDataType* extraDataPtr) const = 0;

    virtual shared_ptr< SparseMatrix >
    GetGradientWrtSecondTerm(const Matrix& X, const Matrix& Z, const Matrix& K,
                             ExtraDataType* extraDataPtr = NULL) const = 0;

    virtual void AddParametersToVariableMap(
            std::map<std::string, shared_ptr<Matrix> >& variableMap) const = 0;

};

class RbfKernel : public Kernel
{
private:
    double _alpha;
    double _gamma;
    static const int NUMBER_PARAMETERS = 2;

public:
    typedef Eigen::MatrixXd Matrix;
    typedef Eigen::VectorXd Vector;
    typedef Eigen::SparseMatrix<double> SparseMatrix;
    typedef Eigen::MatrixXd ExtraDataType;

public:
    explicit RbfKernel();

    explicit RbfKernel(const double alpha, const double gamma);

    virtual Kernel* createCopy() const;

    virtual int NumberOfParameters() const;

    virtual void SetParameters(const Vector& logParams);

    virtual void GetParameters(Vector& logParams) const;

    virtual void Print() const;

    virtual Matrix GetKernelMatrix(const Matrix& X, const Matrix& Z,
                                   ExtraDataType* extraDataPtr = NULL) const;

    virtual Vector GetDiagonalOfKernel(const Matrix& X) const;

    virtual std::vector< shared_ptr< Matrix > >
    GetGradientWrtParams(const Matrix& X, const Matrix& K,
                         ExtraDataType* extraDataPtr = NULL) const;

    virtual shared_ptr< SparseMatrix >
    GetGradientWrtData(const Matrix& X, const Matrix& K,
                       ExtraDataType* extraDataPtr = NULL) const;

    virtual shared_ptr< SparseMatrix >
    GetGradientWrtSecondTerm(const Matrix& X, const Matrix& Z, const Matrix& K,
                             ExtraDataType* extraDataPtr = NULL) const;

    virtual void AddParametersToVariableMap(
            std::map<std::string, shared_ptr<Matrix> >& variableMap) const;
};


#endif // KERNEL_HPP
