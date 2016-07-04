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

#include "include/ImgIcon.hpp"

ImgIcon::ImgIcon(QImage *image, QWidget * parent):QLabel(parent),img(image){
    this->setStatus(EditType::NOEDIT);
}

ImgIcon::ImgIcon(QImage *image, int i, QWidget * parent):QLabel(parent),img(image){
    num = i;
    this->setStatus(EditType::NOEDIT);
}

void ImgIcon::mousePressEvent(QMouseEvent *event){
    emit pushImgIconClicked(num);
    QLabel::mousePressEvent(event);
}

void ImgIcon::setStatus(EditType c){
    status = c;
    if (status == EditType::NOEDIT)
        color = Qt::black;
    else if (status == EditType::KEYFRAME)
        color = Qt::green;
    else if (status == EditType::SUGGESTED)
        color = Qt::blue;
    else if (status == EditType::EDITING)
        color = Qt::yellow;
    else if (status == EditType::NEXTEDIT)
        color = Qt::red;
    this->update();
}

void ImgIcon::paintEvent(QPaintEvent *event){
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.drawImage(QRectF(0, 0, this->width(), this->height()), *img);
    QPen pen(color,10);
    painter.setPen(pen);
    painter.drawRect(QRectF(0, 0, this->width(), this->height()));
    if(curves.size()>0){
//        qDebug() << "number of curves in an icon: " << curves.size();
        pen.setColor(curves.first()->getColor());
        pen.setWidth(15);
        painter.setPen(pen);
        painter.scale(float(this->width())/float(img->width()),float(this->height())/float(img->height()));
        painter.translate(img->width()/2,img->height()/2);
        for(QList<SpCurve *>::iterator iNode = curves.begin(); iNode!=curves.end();iNode++)
            painter.drawPath(*((*iNode)->getCurve()));
    }
    painter.end();
}

void ImgIcon::updateCurves(QList<SpCurve*> c ){
    if(curves.size()>0) curves.clear();
    if(!c.empty() && c.size()>0){
        for(QList<SpCurve *>::iterator iNode = c.begin(); iNode!=c.end();iNode++){
            curves.append(*iNode);
        }
    }
    this->update();
}

void ImgIcon::setEditingStatus(){
    this->setStatus(EDITING);
    this->update();
}

void ImgIcon::setNoEditStatus(){
    this->setStatus(NOEDIT);
    this->update();
}

void ImgIcon::setKeyFrameStatus(){
    this->setStatus(KEYFRAME);
    this->update();
}

void ImgIcon::setSuggestStatus(){
    this->setStatus(SUGGESTED);
    this->update();
}

void ImgIcon::designActionClick(){
    emit pushImgIconClicked(num);
//    emit pushRecheckDesignActions();
}


