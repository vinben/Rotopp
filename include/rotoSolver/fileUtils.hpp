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

#ifndef FILEUTILS_HPP
#define FILEUTILS_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SparseCore>

#include <string>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <cassert>
#include <functional>
#include <memory>
#include "include/rotoSolver/baseDefs.hpp"
#include <cstdio>

using std::string;

class FilePtr
{
public:
        FilePtr() : _fp()
        {}

        FilePtr(const char* path, const char* mode) : _fp(std::fopen(path, mode), std::ptr_fun(std::fclose))
        {}

        void close()
        {
                _fp.reset();
        }

        void open(const char* path, const char* mode)
        {
                FilePtr newFP(path, mode);

                _fp.swap(newFP._fp);
        }

        operator std::FILE*() const
        {
                return _fp.get();
        }

        operator bool() const
        {
                return _fp.get() != NULL;
        }

private:

        shared_ptr<std::FILE> _fp;
};

// The missing string trim function - v. useful..
// Taken from http://www.codeproject.com/KB/stl/stdstringtrim.aspx
void trim( string& str );

string readLineFromFile(FilePtr& fp);

typedef Eigen::MatrixXd Matrix;

void SaveSolverOutputFile(string textFilename,
                         const Eigen::MatrixXd& Data,
                         const int startIdx,
                         const int stopIdx);

enum TrackingDataType
{
    Unknown,
    ForwardPlanar,
    BackwardPlanar,
    Point
};

TrackingDataType TrackingDataTypeMapper(const string str);

string TrackingDataTypeToString(const TrackingDataType& t);

struct  PlanarTrackingData
{
    string ShapeName;
    std::vector<int> KeyFrames;
    int NumFrames;
    int StartIndex;
    int EndIndex;
    std::vector<int> PointIDs;
    Eigen::MatrixXd TrackingData;
    TrackingDataType DataType;
    Eigen::VectorXd FrameWeights;
    string OrigFileName;

    PlanarTrackingData(){DataType = Unknown;}
    PlanarTrackingData(string txtFilename);

    void SaveToOutputFile(const string textFilename) const;

    void SetFrameWeights();

    void Print() const;
};

#endif // FILEUTILS_HPP
