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

#include "include/designZone/SpHook.hpp"

using namespace std;

SpHook::SpHook(QPointF pos, int typ, qreal s){
    position = pos;
//    this->setPos(pos-QPointF(2,2));
    this->setPos(pos);
    type = typ;
    radius = 5;
    lineWid = 1;
    ptScale = s;
}

SpHook::~SpHook(){

}

QRectF SpHook::boundingRect() const{
    return QRectF(-radius/2, -radius/2, radius, radius);
}
void SpHook::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget){
    Q_UNUSED(option);
    Q_UNUSED(widget);
    painter->setRenderHint(QPainter::Antialiasing);
    QPen pen(Qt::black,lineWid/ptScale);
    painter->setPen(pen);
    painter->setBrush(Qt::blue);
    qreal paitRadius = radius/ptScale;
//    painter->drawEllipse(-paitRadius/2, -paitRadius/2, paitRadius, paitRadius);
    painter->drawEllipse(QPointF(0,0), paitRadius/2, paitRadius/2);
}

QPointF SpHook::getPosition(){
    return position;
}

void SpHook::setPosition(QPointF pos){
    position = pos;
}

qreal SpHook::distance(QPointF b) {
    QPointF diff = this->scenePos() - b;
    return sqrt(diff.x()*diff.x() + diff.y()*diff.y());
}
