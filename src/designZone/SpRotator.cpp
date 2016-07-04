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

#include "include/designZone/SpRotator.hpp"
using namespace std;
SpRotator::SpRotator(SpRectRotate* rotateRect, QList<SpNode*> rotatePts){
    rect = rotateRect;
    pts.clear();

    for(QList<SpNode *>::iterator iNode = rotatePts.begin(); iNode!=rotatePts.end();iNode++){
        pts.append(*iNode);
        this->addToGroup(*iNode);
    }
    this->addToGroup(rect);
}

void SpRotator::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget){
    Q_UNUSED(option);
    Q_UNUSED(widget);
    painter->setRenderHint(QPainter::Antialiasing);

}

QRectF SpRotator::boundingRect() const{
    return QRectF(0, 0, 10, 10);
}

bool SpRotator::addPtsToRotator(QList<SpNode*> pts){
    for(QList<SpNode *>::iterator iNode = pts.begin(); iNode!=pts.end();iNode++){
        this->addToGroup(*iNode);
    }
    return true;
}

bool SpRotator::handlerMoveTo(QPointF pos){
    SpRectRotate* rect = this->getRect();
    QTransform trans = this->transform();
    if(!rect->checkSlt()) return false;
    if(rect->sltHandler){
        rect->getHandler()->resetTransform();
        qreal ang = QLineF(rect->getRotateCenter()->getPosition(),pos).angle();
        this->rotateByAngleOrig(ang);

        for(QList<SpNode *>::iterator iNode = pts.begin(); iNode!=pts.end();iNode++){
            (*iNode)->updateScenePos();
//            qDebug() << "point's Left scene x: " << (*iNode)->getLef()->scenePos().x() << ", y: " << (*iNode)->getLef()->scenePos().y();
//            qDebug() << "point's Left pos   x: " << (*iNode)->getLef()->pos().x() << ", y: " << (*iNode)->getLef()->pos().y();
        }
//        qDebug() << "handler's position x: " << getRect()->getHandler()->scenePos().x() << ", y: " << getRect()->getHandler()->scenePos().y();
//        qDebug() << "angle to center is " << QLineF(rotateCenter->getPosition(),pos).angle() << " degrees.";
        return true;
    }
    else if(rect->sltRotateCenter){
//        rect->getRotateCenter()->resetTransform();
        rect->getRotateCenter()->setPosition(pos*trans.inverted());
        rect->getRotateCenter()->setPos(pos*trans.inverted());
//        qDebug() << "RotationCenter is selected";
        return true;
    } if(rect->sltTopLeft || rect->sltTopRight || rect->sltBottomLeft || rect->sltBottomRight){
        QList<float> rates = rect->getScaleRate(pos*rotation.inverted());
        if(!rates.isEmpty())
            scalebyOrig(rates.first(),rates.last());

        for(QList<SpNode *>::iterator iNode = pts.begin(); iNode!=pts.end();iNode++)
            (*iNode)->updateScenePos();
//        rect->getTopLeft()->setPosition(pos*trans.inverted());
//        rect->getTopLeft()->setPos(pos*trans.inverted());
//        qDebug() << "rotation pos: " << pos;
//        qDebug() << "rotation pos*trans.inverted" << pos*trans.inverted();
        return true;
    }
    else return false;
}

void SpRotator::rotateByAngleOrig(int ang){
//    QTransform cur = this->transform();
//    QTransform tf;
    rotation.reset();
    rotation.translate(this->getRect()->getRotateCenter()->getPosition().x(),
                       this->getRect()->getRotateCenter()->getPosition().y());
    rotation.rotate(90-ang);
    rotation.translate(-this->getRect()->getRotateCenter()->getPosition().x(),
                       -this->getRect()->getRotateCenter()->getPosition().y());
    this->setTransform(scale*rotation);
}

void SpRotator::scalebyOrig(float rateX, float rateY){
//    QTransform cur = this->transform();
//    QTransform tf;
    scale.reset();
    scale.translate(this->getRect()->getRotateCenter()->getPosition().x(),
                    this->getRect()->getRotateCenter()->getPosition().y());
    scale.scale(rateX,rateY);
    scale.translate(-this->getRect()->getRotateCenter()->getPosition().x(),
                    -this->getRect()->getRotateCenter()->getPosition().y());
    this->setTransform(scale*rotation);
//    this->setTransformations();
}

bool SpRotator::rmPtsFromRotator(QList<SpNode*> pts){
    if(pts.isEmpty()) return false;
    for(QList<SpNode *>::iterator iNode = pts.begin(); iNode!=pts.end();iNode++){
        this->removeFromGroup(*iNode);
    }
    return true;
}




