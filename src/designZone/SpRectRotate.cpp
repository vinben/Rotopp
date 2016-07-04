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

#include "include/designZone/SpRectRotate.hpp"

SpRectRotate::SpRectRotate(QList<SpNode*> points)
{
    resetLabels();
    color = Qt::gray;
    QList<QPointF> origPos;
    for(QList<SpNode *>::iterator iNode = points.begin(); iNode!=points.end();iNode++){
        origPos.append((*iNode)->getPosition());
    }
    rect = QPolygonF(origPos.toVector()).boundingRect();
    rect.adjust(-10,-10,10,10);
    scaleNodeTopLeft = new SpNodeRotate(rect.topLeft());
    origTopLeft = rect.topLeft();
    scaleNodeTopLeft->setColor(Qt::blue);
    scaleNodeTopRight = new SpNodeRotate(rect.topRight());
    origTopRight = rect.topRight();
    scaleNodeTopRight->setColor(Qt::blue);
    scaleNodeBottomLeft = new SpNodeRotate(rect.bottomLeft());
    origBottomLeft = rect.bottomLeft();
    scaleNodeBottomLeft->setColor(Qt::blue);
    scaleNodeBottomRight = new SpNodeRotate(rect.bottomRight());
    origBottomRight = rect.bottomRight();
    scaleNodeBottomRight->setColor(Qt::blue);
    rotateCenter = new SpNodeRotate(rect.center());
    handler = new SpNodeRotate((rect.topLeft()+rect.topRight())/2 + QPointF(0,-20));
    this->addToGroup(rotateCenter);
    this->addToGroup(handler);
    this->addToGroup(scaleNodeTopLeft);
    this->addToGroup(scaleNodeTopRight);
    this->addToGroup(scaleNodeBottomLeft);
    this->addToGroup(scaleNodeBottomRight);
    qDebug() << "Center of Rect Box is x:" << rect.center().x() << ", y: " << rect.center().y();
}

QRectF SpRectRotate::boundingRect() const{
    QRectF bound = rect;
    bound.adjust(-10,-30,10,10);
    return bound;
}

void SpRectRotate::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget){
    Q_UNUSED(option);
    Q_UNUSED(widget);
    painter->setRenderHint(QPainter::Antialiasing);
    painter->setPen(color);
//    painter->drawRect(rect);
    painter->drawLine(scaleNodeTopLeft->getPosition(),scaleNodeTopRight->getPosition());
    painter->drawLine(scaleNodeTopRight->getPosition(),scaleNodeBottomRight->getPosition());
    painter->drawLine(scaleNodeBottomRight->getPosition(),scaleNodeBottomLeft->getPosition());
    painter->drawLine(scaleNodeBottomLeft->getPosition(),scaleNodeTopLeft->getPosition());

    handler->setPosition((scaleNodeTopLeft->getPosition()+scaleNodeTopRight->getPosition())/2 + QPointF(0,-20));
    handler->setPos(handler->getPosition());
    painter->drawLine(handler->getPosition(),handler->getPosition()+QPointF(0,20));
    painter->setPen(QPen(color,2,Qt::DashDotLine));
    painter->drawLine(rotateCenter->getPosition(),handler->getPosition()+QPointF(0,20));

}

void SpRectRotate::setColor(QColor c){
    color = c;
}

//SpNodeRotate* SpRectRotate::getHandler(){
//    return handler;
//}

//SpNodeRotate* SpRectRotate::getRotateCenter(){
//    return rotateCenter;
//}

bool SpRectRotate::labelHandler(QPointF pos){
    if(handler->checkContain(pos)) {
        resetLabels();
        sltHandler = true;
    }else if(rotateCenter->checkContain(pos)){
        resetLabels();
        sltRotateCenter = true;
    }else if(scaleNodeTopLeft->checkContain(pos)){
        resetLabels();
        sltTopLeft = true;
    }else if(scaleNodeTopRight->checkContain(pos)){
        resetLabels();
        sltTopRight = true;
    }else if(scaleNodeBottomLeft->checkContain(pos)){
        resetLabels();
        sltBottomLeft = true;
    }else if(scaleNodeBottomRight->checkContain(pos)){
        resetLabels();
        sltBottomRight = true;
    }
    else return false;
    return true;
}

void SpRectRotate::resetLabels(){
    sltHandler = false;
    sltRotateCenter = false;
    sltTopLeft = false;
    sltTopRight = false;
    sltBottomLeft = false;
    sltBottomRight = false;
}

bool SpRectRotate::handlerMoveTo(QPointF pos, QList<SpNode*> pts){
    if(!sltHandler && !sltRotateCenter
            && !sltTopLeft && !sltTopRight && !sltBottomLeft && !sltBottomRight) return false;
    if(sltHandler){
        qreal ang = QLineF(rotateCenter->getPosition(),pos).angle();

        QGraphicsItemGroup* rotatepoints = new QGraphicsItemGroup();

        for(QList<SpNode *>::iterator iNode = pts.begin(); iNode!=pts.end();iNode++){
            rotatepoints->addToGroup(*iNode);
        }
        rotatepoints->addToGroup(this);
        this->rotateByAngleOrig(ang,rotatepoints);


        qDebug() << "handler's position x: " << handler->scenePos().x() << ", y: " << handler->scenePos().y();

//        qDebug() << "angle to center is " << QLineF(rotateCenter->getPosition(),pos).angle() << " degrees.";
        return true;
    }
    else if(sltRotateCenter){
        rotateCenter->setPosition(pos);
        rotateCenter->setPos(pos);
        qDebug() << "RotationCenter is selected";
        return true;
    }else if(sltTopLeft){
        scaleNodeTopLeft->setPosition(pos);
        scaleNodeTopLeft->setPos(pos);
        qDebug() << "RotationTopLeft is selected";
        return true;
    }
    else return false;
}

void SpRectRotate::rotateByAngleOrig(int ang, QGraphicsItemGroup* rotateGroup){
    QTransform tf;
    tf.translate(rotateCenter->getPosition().x(),rotateCenter->getPosition().y());
    tf.rotate(90-ang);
    tf.translate(-rotateCenter->getPosition().x(),-rotateCenter->getPosition().y());
    rotateGroup->setTransform(tf);
}

bool SpRectRotate::checkSlt(){
    return (sltHandler || sltRotateCenter
            || sltTopLeft || sltTopRight || sltBottomLeft || sltBottomRight);
}

QList<float> SpRectRotate::getScaleRate(QPointF newPos){
    QList<float> out;
    QPointF pos(0,0);

    if(sltTopLeft) pos = origTopLeft;
    else if(sltTopRight) pos = origTopRight;
    else if(sltBottomLeft) pos = origBottomLeft;
    else pos = origBottomRight;

    if((pos.x() == rotateCenter->getPosition().x())
       || (pos.y() == rotateCenter->getPosition().y()))
        return out;
    float rateX = abs(newPos.x()-rotateCenter->getPosition().x())/abs(pos.x()-rotateCenter->getPosition().x());
    float rateY = abs(newPos.y()-rotateCenter->getPosition().y())/abs(pos.y()-rotateCenter->getPosition().y());
    out.append(rateX);
    out.append(rateY);
    return out;
}

float SpRectRotate::distance(QPointF from, QPointF to){
    QPointF diff = from - to;
    return sqrt(diff.x()*diff.x() + diff.y()*diff.y());
}






















