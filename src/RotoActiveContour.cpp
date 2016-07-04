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

#include "include/RotoActiveContour.hpp"

RotoActiveContour::RotoActiveContour(QObject *parent) : QObject(parent) {

}

bool RotoActiveContour::SolveActiveContour(cv::Mat imageColor, QtNodeList points, QtNodeList* out){
    if(points.isEmpty()) return false;

    cv::Mat grayscaleMat (imageColor.size(), CV_8U);
    cv::cvtColor( imageColor, grayscaleMat, CV_BGR2GRAY );
    cv::Mat binaryMat(grayscaleMat.size(), grayscaleMat.type());
    cv::threshold(grayscaleMat, binaryMat, 100, 255, cv::THRESH_BINARY);
    CvPoint initContour[points.size()];
    if(!QtNodeListToCvPoints(initContour,points)) return false;

    qDebug() << "running active contour, input points:";
    for(int i = 0; i < points.size(); i++){
        qDebug() << "x:" << initContour[i].x << ", y:" << initContour[i].y;
    }

//    cv::imshow("color",grayscaleMat);
//    cv::waitKey(0);
//    cv::imshow("color",binaryMat);
//    cv::waitKey(0);

    IplImage* img = new IplImage(binaryMat);

    CvTermCriteria criteria;
    criteria.type = CV_TERMCRIT_ITER;
    criteria.max_iter=1000;
    criteria.epsilon=0.1;
    float alpha = 0.1f, beta = 0.4f, gama = 0.5f;
    cvSnakeImage(img, initContour, points.size(), &alpha, &beta, &gama, CV_VALUE, cvSize(3, 3), criteria, 0);

    if(!out->isEmpty()) out->clear();
    if(!CvPointsToQtNodeList(out,initContour,points.size())) return false;

    qDebug() << "finish running active contour, input points:";
    for(int i = 0; i < points.size(); i++){
        qDebug() << "x:" << out->at(i).x() << ", y:" << out->at(i).y();
    }

    return true;
}

bool RotoActiveContour::SolveActiveContour(QImage* image, QtNodeList points, QtNodeList* out){
    cv::Mat im = QImageToCvMat(image,false);
    return SolveActiveContour(im,points,out);
}

bool RotoActiveContour::QtNodeListToCvPoints(CvPoint* out, QtNodeList qtnodes){
    if(qtnodes.isEmpty()) return false;
    for(int iPt = 0; iPt < qtnodes.size(); iPt++){
        CvPoint p;
        p.x = float(qtnodes.at(iPt).x());
        p.y = float(qtnodes.at(iPt).y());
        out[iPt] = p;
    }
    return true;
}

bool RotoActiveContour::CvPointsToQtNodeList(QtNodeList* out, CvPoint* in, int cvPointSize){
    if(cvPointSize<1) return false;
    for(int iPt = 0; iPt < cvPointSize; iPt++){
        out->append(QPointF(in[iPt].x, in[iPt].y));
    }
    return true;
}

cv::Mat RotoActiveContour::QImageToCvMat( QImage* inImage, bool inCloneImageData) {
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

int RotoActiveContour::returnLargestContourIndex(vector<vector<Point> > contours)
{
    unsigned int max_contour_size = 0;
    int max_contour_idx = -1;
    for(unsigned int i = 0; i < contours.size(); ++i)
    {
        if(contours[i].size() > max_contour_size)
        {
            max_contour_size = static_cast<int>(contours[i].size());
            max_contour_idx = i;
        }
    }
    return max_contour_idx;
}

