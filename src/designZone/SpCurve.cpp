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

#include "include/designZone/SpCurve.hpp"

SpCurve::SpCurve(SpNode* pre,SpNode* next){
    lineWid = 1;
    scale = 1;
//    color = QColor(0,176,240);
    color = Qt::red;
    preNode = pre;
    nextNode = next;
    initCurve();
}

SpCurve::SpCurve(SpNode* pre,SpNode* next, int s){
    lineWid = 2;
    scale = 1;
//    color = QColor(0,176,240);
    color = Qt::red;
    preNode = pre;
    nextNode = next;
    scale = s;
    initCurve();
}

SpCurve::SpCurve(SpNode* pre,SpNode* next, QColor c){
    lineWid = 1;
    scale = 1;
    preNode = pre;
    nextNode = next;
    color = c;
    initCurve();
}

SpCurve::SpCurve(SpNode* pre,SpNode* next, QColor c, int s){
    scale = s;
    color = c;
    lineWid = 1;
    scale = 1;
    preNode = pre;
    nextNode = next;
    initCurve();
}

void SpCurve::initCurve(){
    curve = new QPainterPath(preNode->getPosition());
    QPointF P1 = preNode ->getPosition(), P2 = nextNode->getPosition(), A1 = preNode->getLef()->getPosition(), A2 = nextNode->getRig()->getPosition();
    A1.setX(A1.x()+P1.x()); A1.setY(A1.y()+P1.y());
    A2.setX(A2.x()+P2.x()); A2.setY(A2.y()+P2.y());
    curve->cubicTo(A1,A2,P2);
}

QRectF SpCurve::boundingRect() const{
    return curve->boundingRect();
}

void SpCurve::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget){
    Q_UNUSED(option);
    Q_UNUSED(widget);
    painter->setRenderHint(QPainter::Antialiasing);
//    QPen pen(color,lineWid/scale);
    QPen pen(color,lineWid);
    painter->setPen(pen);
    painter->drawPath(*curve);
}

QList<QPointF> SpCurve::getCurvePt(){

    QList<QPointF> list;
    QPointF P1 = preNode ->getPosition(), P2 = nextNode->getPosition(), A1 = preNode->getLef()->getPosition(), A2 = nextNode->getRig()->getPosition();
    A1.setX(A1.x()+P1.x()); A1.setY(A1.y()+P1.y());
    A2.setX(A2.x()+P2.x()); A2.setY(A2.y()+P2.y());

    float step = 0;
    while(step <= 1){
        float a = step;
        float b = 1 - step;
        QPointF pt = P1*b*b*b + 3*A1*b*b*a + 3*A2*b*a*a + P2*a*a*a;
        list.append(pt);
        step = step + 0.05;
    }
    return list;
}


QList<QList<QPointF> > getCurvesPt(QList<SpCurve*> curveList){
    QList<QList<QPointF> > list;
    list.clear();
    for(QList<SpCurve*>::iterator iNode = curveList.begin(); iNode!=curveList.end();iNode++){
        list.append((*iNode)->getCurvePt());
    }
    return list;
}
