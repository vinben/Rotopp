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

#ifndef AVDTracker_H
#define AVDTracker_H

#include <QObject>
#include <QImage>
#include <QDebug>
//#include <opencv2/features2d.hpp> // for opencv 3.0
#include <opencv2/features2d/features2d.hpp> // for opencv 2.4.11
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <iomanip>

#include <include/utils.hpp>

using namespace cv;
using namespace std;

class AvdTracker : public QObject {
    Q_OBJECT
private:
    double akaze_thresh; // AKAZE detection threshold set to locate about 1000 keypoints
    double ransac_thresh; // RANSAC inlier threshold
    double nn_match_ratio; // Nearest-neighbour matching ratio
    bool flagInit;
    int rectEnhance;
    int matchNumThreadhold;
public:
    explicit AvdTracker(QObject *parent = 0);
    void initCurFrame(int, QRectF);
    cv::Mat QImageToCvMat(QImage*, bool inCloneImageData = false );
//    Ptr<Feature2D> getDetector() {return detector;}
    void setFrames(QList<QImage*>);
    cv::Mat getMask(cv::Mat, QRectF);
    QRectF getBoundingRectF(VecNodeList);
    bool isCurrArea(QRectF);
    bool checkOverBoundary(QRectF);
    std::vector<cv::Point2f> matPoints(std::vector<KeyPoint>);
    std::vector<cv::Point2f> VecNodeListToVecPoints(VecNodeList);
    bool VecPointsToVecNodeList(std::vector<cv::Point2f>, VecNodeList*);
    bool Track(VecNodeList, VecNodeList*, int, int);
    bool niceHomography(cv::Mat);
protected:
    QList<cv::Mat> matImgs;
    QList<std::vector<KeyPoint> > keyPts;
    QList<cv::Mat > descs;
//    cv::Ptr<cv::AKAZE> detector; // for opencv 3.0
    cv::ORB detector; // for opencv 2.4.11
    cv::Ptr<DescriptorMatcher> matcher;

    int currIndFrame;
    QRectF currRect;
    std::vector<KeyPoint> curKeyPts;
    cv::Mat curDescs;

signals:

public slots:
};

#endif // AvdTracker_H





