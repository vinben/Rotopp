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

#include "include/designZone/SpPoly.hpp"

SpPoly::SpPoly(QList<SpCurve*> *crvs) {
    this->curves = crvs;
    alpha = 128;
    color = (*curves).last()->getColor();
    paths = new QPainterPath();
    QList<QPointF> pts;
    for(QList<SpCurve *>::iterator iNode = curves->begin(); iNode!=curves->end();iNode++){
        paths->addPath(*((*iNode)->getCurve()));
        pts.append((*iNode)->getPreNode()->getPosition());
        pts.append((*iNode)->getNextNode()->getPosition());
    }
    polys = new QPolygonF(pts.toVector());
    paths->addPolygon(*polys);
    paths->closeSubpath();
    paths->setFillRule(Qt::WindingFill);
//    paths->simplified();
}

SpPoly::SpPoly(QList<SpNode*>* pt, QColor c){
    points = new QList<SpNode*>;
    points = pt;
    alpha = 128;
    color = c;
    paths = new QPainterPath();
    polys = new QPolygonF();
//    curves = new QList<SpCurve*>;
    if(points->size()>1){

        QList<QPointF> pts;
        for(QList<SpNode *>::iterator iNode = points->begin(); iNode!=points->end();iNode++){
            pts.append((*iNode)->getPosition());
            if(!(*iNode)->hasNextNode) continue;
            SpNode *endPoint = (*iNode)->getNextNode();
            SpCurve *midCurve = new SpCurve((*iNode),endPoint,color,1);
//            curves->append(midCurve);
            paths->addPath(*(midCurve->getCurve()));
        }
        polys = new QPolygonF(pts.toVector());
        paths->addPolygon(*polys);
        paths->closeSubpath();
        paths->setFillRule(Qt::WindingFill);
    }
}

QRectF SpPoly::boundingRect() const{
    return paths->boundingRect();
//    return polys->boundingRect();
//    return QRectF(0, 0, 10, 10);
}

void SpPoly::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget){
    Q_UNUSED(option);
    Q_UNUSED(widget);
    //    this->updatePoly();
    if(points->size()>1){
        this->updatePolyPts();
        QPen pen = QPen(color);
        pen.setWidth(2);
        painter->setPen(pen);
        color.setAlpha(alpha);
        painter->setBrush(QBrush(color));
        paths->setFillRule(Qt::WindingFill);
        painter->drawPath(paths->simplified());
    }
}

void SpPoly::updatePoly(){
    color = (*curves).last()->getColor();
    paths = new QPainterPath();
    QList<QPointF> pts;
    for(QList<SpCurve *>::iterator iNode = curves->begin(); iNode!=curves->end();iNode++){
        paths->addPath(*((*iNode)->getCurve()));
        pts.append((*iNode)->getPreNode()->getPosition());
        pts.append((*iNode)->getNextNode()->getPosition());
    }
    polys = new QPolygonF(pts.toVector());
    paths->addPolygon(*polys);
    paths->closeSubpath();
//    paths->simplified();
    paths->setFillRule(Qt::WindingFill);
}

void SpPoly::updatePolyPts(){
    if(points->size()<2) return;
    paths = new QPainterPath();
    QList<QPointF> pts;
//    curves->clear();
    if(points->size()>1){
        for(QList<SpNode *>::iterator iNode = points->begin(); iNode!=points->end();iNode++){
            pts.append((*iNode)->getPosition());
            if(!(*iNode)->hasNextNode) continue;
            SpNode *endPoint = (*iNode)->getNextNode();
            SpCurve *midCurve = new SpCurve((*iNode),endPoint,color,1);
//            curves->append(midCurve);
            paths->addPath(*(midCurve->getCurve()));
        }
    }
    polys = new QPolygonF(pts.toVector());
    paths->addPolygon(*polys);
    paths->closeSubpath();
//    paths->simplified();
    paths->setFillRule(Qt::WindingFill);
}
