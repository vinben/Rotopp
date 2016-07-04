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

#ifndef UTILS
#define UTILS

#include<include/designZone/SpNode.hpp>
#include<include/designZone/SpCurve.hpp>
#include<QImage>
#include<vector>
#include<QDebug>

typedef QList<SpNode*> NodeList;
typedef QList<NodeList*> ARotoCurve;
typedef QList<QImage*> ImageList;
typedef QMap<int,NodeList*> KeyFrames;
typedef QList<SpCurve*> CurveList;
typedef QList<bool> FlagList;
typedef QList<QPointF> QtNodeList;
typedef QMap<int,QtNodeList> QtKeyFrames;
typedef QList<QPointF> VecNode; // positions of center, left tangent point and right tangent point
typedef QList<VecNode> VecNodeList;
typedef QMap<int,VecNodeList> VecKeyFrames;

void mul_v3_m3v3(float r[3], float M[3][3], const float a[3]);
void copy_m3_m3d(float R[3][3], double A[3][3]);

struct vec2;
struct vec3;

struct TrackerData {
    QList<QtNodeList> nodes;  // list of nodes
    QList<int> indKeyframe;    // index of keyframes
    bool flagTrackDirection;    // true - forward; false - backward;
};

struct TrackerData3 {
    QList<VecNodeList> nodes;  // list of nodes; for each node, it contains positions of center, left tangent point and right tangent point
    QList<int> indKeyframe;    // index of keyframes
    bool flagTrackDirection;    // true - forward; false - backward;
    QList<QPointF> getPtListofCenter(int);
    void unNorm(QPointF);
    int mostCloseIndKeys(int,bool);
    void clean();
};

enum EditType{
    NOEDIT,
    KEYFRAME,
    SUGGESTED,
    EDITING,
    NEXTEDIT
};

enum TrackType{
    FORWARD,
    BACKWARD
};

enum SolverType{
    TRACKER_GPLVM_NEXTEDIT,
    TRACKER_GPLVM,
    TRACKER,
    LINEAR_INTERP
};

enum ControlType{
    CtrSelect,
    CtrCreate,
    CtrMove,
    CtrDelect,
    CtrDragPts,
    CtrSolverDragPts,
    CtrModPt,
    CtrRotation
};

qreal qtDistance(QPointF,QPointF);

template<class T>
const T& kClamp( const T& x, const T& low, const T& high ){
    if ( x < low ) return low;
    else if ( high < x ) return high;
    else return x;
}

QImage changeBrightness( const QImage&, int);
QImage changeContrast( const QImage&, int);
QImage changeGamma( const QImage&, int);
QImage changeImage( const QImage&, int);

#define DEFAULT_BRIGHTNESS 0
#define DEFAULT_CONTRAST 100
#define DEFAULT_GAMMA 100
#define MAX_IMAGEPROC 255
#define DEFUALT_TANGENT_LENGTH 30
#define DEFUALT_INCREASING_RATIO 1.1

/// Global vars

extern bool isFiled_curveMap;
extern bool isPrintedPt_curveMap;
/// Algebra structures


#endif // UTILS

