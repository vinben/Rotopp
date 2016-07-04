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

#ifndef EIGENUTILS_HPP
#define EIGENUTILS_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <Eigen/Eigenvalues>

typedef Eigen::MatrixXd Matrix;
typedef Eigen::VectorXd Vector;

#include "baseDefs.hpp"

template <typename T>
Eigen::Map< Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>,
            Eigen::Unaligned,
            Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic> >
            MatrixRowMapper(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& mat, int row, int M, int N)
{
    typedef Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic> DynamicStride;
    assert (row < mat.rows());
    return Eigen::Map< Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>,
            Eigen::Unaligned,
            DynamicStride >(mat.data() + row, M, N, DynamicStride(M * mat.rows(), mat.rows()));
}

template <typename T>
Eigen::Map< Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>,
            Eigen::Unaligned,
            Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic> >
            MatrixRowMapperMutable(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& mat, int row, int M, int N)
{
    typedef Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic> DynamicStride;
    assert (row < mat.rows());
    return Eigen::Map< Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>,
            Eigen::Unaligned,
            DynamicStride >(mat.data() + row, M, N, DynamicStride(M * mat.rows(), mat.rows()));
}

template <typename T>
Eigen::Map< const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>,
            Eigen::Unaligned,
            Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic> >
            MatrixRowMapper(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& mat, int row, int M, int N)
{
    typedef Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic> DynamicStride;
    assert (row < mat.rows());
    return Eigen::Map< const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>,
            Eigen::Unaligned,
            DynamicStride >(mat.data() + row, M, N, DynamicStride(M * mat.rows(), mat.rows()));
}

template<typename T>
class Interpolator
{
    typedef Eigen::MatrixXd Matrix;
    typedef Eigen::VectorXd Vector;

public:
    Interpolator(T start, T end, const Vector& a, const Vector& b) :
        _begin(start), _end(end), _a(a), _b(b)
    {
        nassert (_a.size() == _b.size());
    }

    Vector get(T interpPoint)
    {
        const double x(interpPoint);

        nassert (x >= _begin);
        nassert (x <= _end);

        const double xb = (x - _begin) / (_end - _begin);
        const double xa = 1.0 - xb;

        return Vector(_a * xa) + (_b * xb);
    }

private:
    double _begin, _end;
    Vector _a, _b;
};

#endif // EIGENUTILS_HPP
