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

#include "include/designZone/SpRect.hpp"

SpRect::SpRect(QPointF topleft)
{
    rect = QRectF(topleft,QSize(0,0));
    color = QColor(0,176,240);
}

QRectF SpRect::boundingRect() const{
    return rect;
//    return QRectF(0, 0, 10, 10);
}

void SpRect::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget){
    Q_UNUSED(option);
    Q_UNUSED(widget);
    painter->setRenderHint(QPainter::Antialiasing);
    painter->setPen(color);
    painter->drawRect(rect);
}

void SpRect::setRect(QPointF topleft,QPointF buttright){
    float left = topleft.x() < buttright.x() ? topleft.x() : buttright.x();
    float right = topleft.x() > buttright.x() ? topleft.x() : buttright.x();
    float top = topleft.y() < buttright.y() ? topleft.y() : buttright.y();
    float bottom = topleft.y() > buttright.y() ? topleft.y() : buttright.y();
    rect.setTop(top);
    rect.setBottom(bottom);
    rect.setLeft(left);
    rect.setRight(right);
//    rect.setTopLeft(topleft);
//    rect.setBottomRight(buttright);
    this->update();
}

void SpRect::setColor(QColor c){
    color = c;
}

QRectF SpRect::getRect(){
    return rect;
}

bool SpRect::labelSltPts(QList<SpNode*> points){
    int nPt = 0;
//    sltpoints.clear();
    for(QList<SpNode*>::iterator iNode = points.begin(); iNode!=points.end();iNode++){
            if(rect.contains((*iNode)->getPosition())){
                (*iNode)->slted();
                nPt++;
            }
            else{
                (*iNode)->resetSlted();
            }
        }
    if(nPt>0) return true;
    else return false;
}
