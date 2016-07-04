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

#include "include/AvdTracker.hpp"

AvdTracker::AvdTracker(QObject *parent) : QObject(parent) {
    akaze_thresh = 3e-4; // AKAZE detection threshold set to locate about 1000 keypoints
    ransac_thresh = 2.5f; // RANSAC inlier threshold
    nn_match_ratio = 0.8f; // Nearest-neighbour matching ratio
    rectEnhance = 20; // size of area to enhance keypoint detection
    matchNumThreadhold = 40;

//    detector = cv::AKAZE::create(); // for opencv 3.0
//    detector->setThreshold(akaze_thresh); // for opencv 3.0
    detector = cv::ORB(2000); // for opencv 2.4.11
    matcher = DescriptorMatcher::create("BruteForce-Hamming");
    flagInit = false;
}

void AvdTracker::setFrames(QList<QImage*> imgs){
//    qDebug() << "setting images for AvdTracker.";
    if(!keyPts.isEmpty()) keyPts.clear();
    if(!descs.isEmpty()) descs.clear();
    if(!matImgs.isEmpty()) matImgs.clear();

    for(int iImg = 0; iImg < imgs.size(); iImg++){
        std::vector<KeyPoint> keys;
        cv::Mat desc;
        cv::Mat im = QImageToCvMat(imgs[iImg],false);
//        qDebug() << "finish turning qiamge to mat";
        detector(im, noArray(), keys, desc); // for opencv 2.4.11
//        detector->detectAndCompute(im, noArray(), keys, desc); // for opencv 3.0
        keyPts.append(keys);
        descs.append(desc);
        matImgs.append(im);
    }

    flagInit = true;
}

cv::Mat AvdTracker::QImageToCvMat( QImage* inImage, bool inCloneImageData) {
    switch ( inImage->format() ) {
    case QImage::Format_RGB32: {      // 8-bit, 4 channel
        cv::Mat  mat( inImage->height(), inImage->width(), CV_8UC4, const_cast<uchar*>(inImage->bits()), inImage->bytesPerLine() );
        //        namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
        //        imshow( "Display window", mat );
        //        waitKey(0);
        return (inCloneImageData ? mat.clone() : mat);
    }
    case QImage::Format_RGB888: {     // 8-bit, 3 channel
        if ( !inCloneImageData )
            qWarning() << "QImageToCvMat() - Conversion requires cloning since we use a temporary QImage";
        QImage   swapped = inImage->rgbSwapped();
        return cv::Mat( swapped.height(), swapped.width(), CV_8UC3, const_cast<uchar*>(swapped.bits()), swapped.bytesPerLine() ).clone();
    }
    case QImage::Format_Indexed8: {     // 8-bit, 1 channel
        cv::Mat  mat( inImage->height(), inImage->width(), CV_8UC1, const_cast<uchar*>(inImage->bits()), inImage->bytesPerLine() );
        return (inCloneImageData ? mat.clone() : mat);
    }
    default:
        qWarning() << "QImageToCvMat() - QImage format not handled in switch:" << inImage->format();
        break;
    }
    return cv::Mat();
}

cv::Mat AvdTracker::getMask(cv::Mat im, QRectF rect){
    if(rect.size().height() == 0 && rect.size().width() == 0) return cv::Mat();
    float topleftX = rect.topLeft().x() < 0 ? 0 : rect.topLeft().x();
    float topleftY = rect.topLeft().y() < 0 ? 0 : rect.topLeft().y();
    float bottRightX = rect.bottomRight().x() > im.size().width-1 ? im.size().width-1 : rect.bottomRight().x();
    float bottRightY = rect.bottomRight().y() > im.size().height-1 ? im.size().height-1 : rect.bottomRight().y();
//    qDebug() << "AvdTracker mask, topx: " << topleftX << ", topy: " << topleftY
//             << "bottX: " << bottRightX << "bottY: " << bottRightY;
//    cv::Rect2f matrect(cv::Point2f(topleftX,topleftY),cv::Point2f(bottRightX,bottRightY)); // for opencv 3.0
    cv::Rect matrect(cv::Point2f(topleftX,topleftY),cv::Point2f(bottRightX,bottRightY));
    cv::Mat mask = cv::Mat::zeros(im.size(), CV_8UC1);  //NOTE: using the type explicitly
    cv::Mat roi(mask, matrect);
    roi = cv::Scalar(255, 255, 255);
    return mask;
}

QRectF AvdTracker::getBoundingRectF(VecNodeList from){
    QtNodeList tmpFrom;
    for(int i = 0; i < from.size(); i++)
        tmpFrom.append(from.at(i).first());
    QPolygonF poly(tmpFrom.toVector());
    QRectF rect = poly.boundingRect();
    rect.adjust(-rectEnhance,-rectEnhance,2*rectEnhance,2*rectEnhance);
    return rect;
}

bool AvdTracker::isCurrArea(QRectF rect){
    if(currRect.topLeft().x() == rect.topLeft().x()
            && currRect.topLeft().y() == rect.topLeft().y()
            && currRect.size().width() == rect.size().width()
            && currRect.size().height() == rect.size().height())
        return true;
    else return false;
}

bool AvdTracker::checkOverBoundary(QRectF rect){
    int imWid = matImgs[currIndFrame].cols;
    int imHeight = matImgs[currIndFrame].rows;
    return rect.intersects(QRectF(QPointF(0,0),QPointF(imWid,imHeight)));
}

bool AvdTracker::Track(VecNodeList from, VecNodeList* to, int old, int search){
    //    cv::Rect rect = getBoundingRect(from);
    //    if(old!=currIndFrame && )
    QRectF rect = getBoundingRectF(from);
    if(!checkOverBoundary(rect)) return false;
//    qDebug() << "AvdTracker qrect done";
    if(old != currIndFrame || !isCurrArea(rect)){
        currIndFrame = old;
        currRect = rect;
        cv::Mat mask = getMask(matImgs[currIndFrame], rect);
//        detector->detectAndCompute(matImgs[currIndFrame], mask, curKeyPts, curDescs); // for opencv 3.0
        detector(matImgs[currIndFrame], mask, curKeyPts, curDescs);
    }

//    qDebug() << "AvdTracker features done";

    std::vector<KeyPoint> searchKeys = keyPts[search];
    vector< vector<DMatch> > matches;
    vector<KeyPoint> matched1, matched2;
    matcher->knnMatch(curDescs, descs[search], matches, 2);
    for(unsigned i = 0; i < matches.size(); i++) {
        if(matches[i][0].distance < nn_match_ratio * matches[i][1].distance) {
            matched1.push_back( curKeyPts[matches[i][0].queryIdx]);
            matched2.push_back(searchKeys[matches[i][0].trainIdx]);
        }
    }

//    qDebug() << "AvdTracker matching done";

    Mat inlier_mask, homography;
    vector<KeyPoint> inliers1, inliers2;
    vector<DMatch> inlier_matches;
//    qDebug() << "number of matches are found: " << matched1.size();
    if(matched1.size() < matchNumThreadhold) return false;
    if(matched1.size() >= 4) {
        homography = findHomography(matPoints(matched1), matPoints(matched2),
                                    RANSAC, ransac_thresh, inlier_mask);
    }

    if(homography.empty() || !niceHomography(homography)) return false;

//    qDebug() << "AvdTracker findHomography done";

    for(unsigned i = 0; i < matched1.size(); i++) {
        if(inlier_mask.at<uchar>(i)) {
            int new_i = static_cast<int>(inliers1.size());
            inliers1.push_back(matched1[i]);
            inliers2.push_back(matched2[i]);
            inlier_matches.push_back(DMatch(new_i, new_i, 0));
        }
    }

    vector<Point2f> vecFrom = VecNodeListToVecPoints(from);
    vector<Point2f> tmpTo;

    perspectiveTransform(vecFrom, tmpTo, homography);

//    Mat res;
//    drawMatches(matImgs[currIndFrame], inliers1, matImgs[search], inliers2,
//                inlier_matches, res,
//                Scalar(255, 0, 0), Scalar(255, 0, 0));
    //    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    //    imshow( "Display window", res );
    //    waitKey(0);

    if (VecPointsToVecNodeList(tmpTo, to)) return true;
    return false;
}

vector<Point2f> AvdTracker::matPoints(vector<KeyPoint> keypoints) {
    vector<Point2f> res;
    for(unsigned i = 0; i < keypoints.size(); i++) {
        res.push_back(keypoints[i].pt);
    }
    return res;
}

vector<Point2f> AvdTracker::VecNodeListToVecPoints(VecNodeList from) {
    vector<Point2f> res;
    //    qDebug() << "size of points before VecNodeListToVecPoints conversion: " << from.size();
    if(!from.isEmpty()){
        for(unsigned i = 0; i < from.size(); i++) {
            QList<QPointF> node = from[i];
            res.push_back(Point2f(node[0].x(),node[0].y()));
            res.push_back(Point2f(node[1].x(),node[1].y()));
            res.push_back(Point2f(node[2].x(),node[2].y()));
        }
    }
    //    qDebug() << "size of points after VecNodeListToVecPoints conversion: " << res.size();
    return res;
}

bool AvdTracker::VecPointsToVecNodeList(vector<Point2f> from, VecNodeList* to){
    if(from.size()<1) return false;
    if(!to->isEmpty()) to->clear();
    int nQtPt = from.size()/3;
    for(unsigned i = 0; i < nQtPt; i++) {
        VecNode node;
        Point2f center = from[0];
        Point2f left = from[1];
        Point2f right = from[2];
        node.append(QPointF(center.x,center.y));
        node.append(QPointF(left.x,left.y));
        node.append(QPointF(right.x,right.y));
        to->append(node);
        from.erase(from.begin());
        from.erase(from.begin());
        from.erase(from.begin());
    }
    return true;
}

bool AvdTracker::niceHomography(Mat H) {
    const double det = H.at<double>(0, 0) * H.at<double>(1, 1) - H.at<double>(1, 0) * H.at<double>(0, 1);
    if (det < 0) return false;

    const double N1 = sqrt(H.at<double>(0, 0) * H.at<double>(0, 0) + H.at<double>(1, 0) * H.at<double>(1, 0));
    if (N1 > 4 || N1 < 0.1) return false;

    const double N2 = sqrt(H.at<double>(0, 1) * H.at<double>(0, 1) + H.at<double>(1, 1) * H.at<double>(1, 1));
    if (N2 > 4 || N2 < 0.1) return false;

    const double N3 = sqrt(H.at<double>(2, 0) * H.at<double>(2, 0) + H.at<double>(2, 1) * H.at<double>(2, 1));
    if (N3 > 0.002) return false;

    return true;
}














