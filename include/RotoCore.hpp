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

#ifndef ROTOCORE_H
#define ROTOCORE_H

//this is the core interface for the roto operation.

#include <QtCore>
#include <Eigen/Core>
#include <stdlib.h>
#include <QtMath>

#include <include/utils.hpp>
#include <include/designZone/SpNode.hpp>
#include <include/designZone/DesignView.hpp>
#include <include/FrameFlow.hpp>
#include <include/AvdTracker.hpp>
#include <include/rotoSolver/gplvm.hpp>
#include <include/rotoSolver/fileUtils.hpp>
#include <include/rotoSolver/transformations.hpp>
#include <include/rotoSolver/eigenUtils.hpp>
#include <include/rotoSolver/planarTrackerSolver.hpp>
#include "RotoActiveContour.hpp"

class RotoCore : public QObject{
    Q_OBJECT
private:
    QList<int> currIndKeyframesForGPLVM;
    ImageList* imgs;
    DesignView* view;
    FrameFlow* flowList;
    std::shared_ptr<GPLVM> gplvm;
    shared_ptr<RigidMotionEstimator> motionEst;
    AvdTracker avdtracker; // Blender Tracker
    RotoActiveContour contour; // Active Contour
public:
    RotoCore(DesignView*,FrameFlow*,ImageList*);
    shared_ptr<RigidMotionEstimator> getMotionEstimator(){return motionEst;}
    QPointF getOffset();
//    Normalization methods for both QT and Eigen formats
    QtKeyFrames normQtKeyframes();
    QtNodeList normQtNodeList(NodeList*);
    VecKeyFrames normVecKeyframes();
    VecNodeList normVecFrame(int);
    Eigen::MatrixXd normAllPoints();
    Eigen::MatrixXd normEigenKeyframes();
    Eigen::MatrixXd normEigenAllframes(QList<int>);
    Eigen::MatrixXd normEigenFrame(int);
    QtNodeList unNormQtNodeList(QtNodeList);
    QList<VecNodeList> unNormVecNodeList(QList<VecNodeList>, QPointF);
//    Access methods
    PlanarTrackingData getEigenKeyframes();
    PlanarTrackingData getEigenFrame(int);
    PlanarTrackingData getEigenAllFrame(QList<int>);
    bool checkNumPtsConsistency();
    int getIndClosestKeyframe(int);
    int getIndClosestKeyframe(int,bool);
    void updateTrackingResults(TrackerData3*);
    void updateTrackingResults(Eigen::MatrixXd);
    void updateSolveNodeSingelFrame(Eigen::MatrixXd, int);

//    Data conversion methods for QT points from/to Eigen format
    QList<VecNodeList> EigenMatrixToVecNodeList(Eigen::MatrixXd);
    PlanarTrackingData QtTrackerDataToPlanarTrackingData(TrackerData3*,bool);
    Eigen::MatrixXd VecNodeListToEigenMatrix(QList<VecNodeList>);
    bool copyVecNodeListFrom(VecNodeList,VecNodeList*);
    Eigen::MatrixXd trackerMerge(std::vector<PlanarTrackingData>);
    QList<float> rmsEigenMatrix(Eigen::MatrixXd, Eigen::MatrixXd);

//    Core methods
    void suggestNextFrameForEdit(QList<float>);
    bool runActiveContour(int);
    bool SolveTrackAndGPLVM(std::vector<PlanarTrackingData>,SolverType type);
    bool SolveTrackAndGPLVMSingleFrame(int);
    bool track(TrackerData3*, TrackType, SolverType);
    bool buildGPLVM(std::shared_ptr<RigidMotionEstimator>&);
signals:

public slots:
    void loadImgs(QList<QImage*>);
    bool buildGPLVM();
    bool buildGPLVM_MotionEst();
    bool buildGPLVM_GrabDrag();
};

#endif // ROTOCORE_H
