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

#include "include/rotoSolver/fileUtils.hpp"
#include "include/rotoSolver/eigenUtils.hpp"

#include <gflags/gflags.h>

#include <fstream>

DECLARE_bool(use_planar_tracker_weights);

// The missing string trim function - v. useful..
// Taken from http://www.codeproject.com/KB/stl/stdstringtrim.aspx
void trim( string& str )
{
    string::size_type pos = str.find_last_not_of(' ');
    if (pos != string::npos)
    {
        str.erase(pos + 1);
        pos = str.find_first_not_of(' ');
        if (pos != string::npos)
            str.erase(0, pos);
    }
    else
    {
        str.erase(str.begin(), str.end());
    }
}

void removeWhiteSpace(string& str)
{
    trim(str);
    std::vector<char> charsToRemove({'\t', '\r', '\n'});
    //for (const char c : charsToRemove)
    for (int i = 0; i < charsToRemove.size(); i++)
    {
	const char c = charsToRemove[i];
        str.erase(std::remove(str.begin(), str.end(), c), str.end());
    }
    trim(str);
}

string readLineFromFile(FilePtr& fp)
{
    const int maxLen = 2048;
    char buffer[maxLen];

    if(std::fgets(buffer, maxLen, fp)==NULL)
        cout << "";
    buffer[maxLen-1] = '\0';

    string line(buffer);

    removeWhiteSpace(line);

    return line;
}

void SaveSolverOutputFile(string textFilename,
                         const Matrix& Data,
                         const int startIdx,
                         const int stopIdx)
{
    const int NUM_VALUES_PER_ROTO_POINT = 6;

    int D = Data.cols();

    nassert (remainder(D, NUM_VALUES_PER_ROTO_POINT) == 0);

    D /= NUM_VALUES_PER_ROTO_POINT;

    std::ofstream ofs(textFilename.c_str());

    ofs << D << "\n";

    ofs << startIdx << " " << stopIdx << "\n";

    ofs << Data.transpose();

    ofs.close();

    std::cout << "Saved output to \"" << textFilename << "\"." << std::endl;
}

TrackingDataType TrackingDataTypeMapper(string str)
{
    removeWhiteSpace(str);

    if (str.compare("forward") == 0)
        return ForwardPlanar;
    else if (str.compare("backward") == 0)
        return BackwardPlanar;
    else if (str.compare("point") == 0)
        return Point;

    return Unknown;
}

string TrackingDataTypeToString(const TrackingDataType& t)
{
    switch (t)
    {
    case (ForwardPlanar):
        return "ForwardPlanar";
        break;
    case (BackwardPlanar):
        return "BackwardPlanar";
        break;
    case (Point):
        return "Point";
        break;
    case (Unknown):
    default:
        return "Unknown";
        break;
    }
}

PlanarTrackingData::PlanarTrackingData(string txtFilename)
{
    DataType = Unknown;
    FilePtr fp(txtFilename.c_str(), "r");

    OrigFileName = txtFilename;

    ShapeName = readLineFromFile(fp);

    KeyFrames.clear();
    std::stringstream keyFramesStr(readLineFromFile(fp));
    while (!keyFramesStr.eof())
    {
        try
        {
            int i = -1;
            keyFramesStr >> i;
            if (i > 0)
                KeyFrames.push_back(i);
        }
        catch (...)
        {}
    }
    std::sort(KeyFrames.begin(), KeyFrames.end());

    StartIndex = *(std::min_element(KeyFrames.begin(), KeyFrames.end()));
    EndIndex = *(std::max_element(KeyFrames.begin(), KeyFrames.end()));
    NumFrames = EndIndex - StartIndex + 1;

    //std::vector<int> ptID;
    std::vector<Eigen::VectorXd> data;
    std::stringstream sstream;

    int numFramesOfData = -1;

    while (!feof(fp))
    {
        Eigen::VectorXd v(NumFrames);
        int pt = -1;
        int numRead = 0;

        std::stringstream s(readLineFromFile(fp));
        s.exceptions(std::stringstream::failbit | std::stringstream::badbit);

        try
        {
            if (feof(fp))
            {
                break;
            }

            s >> pt;
            if (pt < 0)
            {
                break;
            }
            for (int i = 0; i < NumFrames; ++i)
            {
                s >> v[i];
                ++numRead;
            }

            if (s.bad())
            {
                break;
            }
        }
        catch (...)
        {
        }

        if (numFramesOfData < 0)
            numFramesOfData = numRead;
        if (numRead == numFramesOfData)
        {
            PointIDs.push_back(pt);
            data.push_back(v.head(numFramesOfData));
        }
        else
        {
            break;
        }
    }

    DataType = TrackingDataTypeMapper(sstream.str());
    vdbg(DataType);

    vdbg(numFramesOfData);
    vdbg(data.size());

    TrackingData.resize(numFramesOfData, data.size());

    for(int i = 0; i < data.size(); i++){
        Eigen::VectorXd v = data[i];
        TrackingData.col(i) = v;
    }

    FrameWeights = Eigen::VectorXd::Ones(NumFrames);

    vdbg(FLAGS_use_planar_tracker_weights);
    if (FLAGS_use_planar_tracker_weights)
    {
        SetFrameWeights();
    }
}

void PlanarTrackingData::SaveToOutputFile(const string textFilename) const
{
    SaveSolverOutputFile(textFilename, TrackingData, StartIndex, EndIndex);
}

void PlanarTrackingData::SetFrameWeights()
{
    typedef Eigen::Matrix<double, 1, 1> Vector1d;

    double startWeight = 0.0;
    double stopWeight = 0.0;

    switch (DataType)
    {
    case Unknown:
    case Point:
        return;
        break;

    case ForwardPlanar:

        startWeight = 1.0;
        stopWeight = 0.0;

        break;

    case BackwardPlanar:

        startWeight = 0.0;
        stopWeight = 1.0;

        break;
    }

    // REMEMBER TO TAKE THE SQUARING OF THE COST INTO ACCOUNT..

    for (int i = 0, I = KeyFrames.size() - 1; i < I; ++i)
    {
        const int a = KeyFrames[i] - StartIndex;
        const int b = KeyFrames[i+1] - StartIndex;

        Interpolator<int> interp(a, b, Vector1d::Constant(startWeight), Vector1d::Constant(stopWeight));

        for (int k = a+1; k < b; ++k)
        {
            nassert (k < NumFrames);
            FrameWeights.row(k) = interp.get(k);
        }
    }

    vdbg(FrameWeights.transpose());
}

void PlanarTrackingData::Print() const
{
    vdbg(ShapeName);
    vdbg(NumFrames);
    vdbg(StartIndex);
    vdbg(EndIndex);
    vdbg(TrackingData.rows());
    vdbg(TrackingData.cols());
    vdbg(TrackingData(0,0));
    vdbg(TrackingData(0,1));
    vdbg(TrackingData(1,0));
    vdbg(TrackingData(TrackingData.rows()-1, TrackingData.cols()-1));
}

