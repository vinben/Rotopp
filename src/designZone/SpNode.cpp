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

#include "include/designZone/SpNode.hpp"
#include "include/designZone/SpCurve.hpp"
using namespace std;

SpNode::SpNode(QPointF pos, QPointF tan_1, QPointF tan_2, qreal s){
    initNode(pos,tan_1,tan_2,s);
}

SpNode::SpNode(SpNode* from){
    QPointF pos = from->getPosition();
    QPointF tan_1 = from->getLef()->getPosition();
    QPointF tan_2 = from->getRig()->getPosition();
    initNode(pos,tan_1,tan_2,from->getScale());
    firstNode = from->isFirstNode();
    color = from->getColor();
    isSlted = from->isSlted;
}

void SpNode::initNode(QPointF pos, QPointF tan_1, QPointF tan_2, qreal s){
    radius = 7;
    lineWid = 1;
    ptScale = s;
    position = pos;
    isSlted = false;
    isVis = true;
    lef = new SpHook(tan_1,1,s);
    rig = new SpHook(tan_2,2,s);
    preNode = NULL;
    nextNode = NULL;
    preFrameNode = NULL;
    nextFrameNode = NULL;
    firstNode = false;
    hasPreNode = false;
    hasNextNode = false;
    hasPreFrameNode = false;
    hasNextFrameNode = false;
    this->addToGroup(lef);
    this->addToGroup(rig);
    this->setPos(position);
    color = Qt::red;
    this->setZValue(0.01);
}

SpNode::~SpNode(){

}

QRectF SpNode::boundingRect() const{

    QPointF startPos = this->lef->getPosition();
    QPointF endPos = this->rig->getPosition();
//    QPointF bound = QPointF(this->lef->getRadius(),this->lef->getRadius());
    return QRectF(startPos, endPos);
}

void SpNode::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget){
    Q_UNUSED(option);
    Q_UNUSED(widget);
    painter->setRenderHint(QPainter::Antialiasing);

    if(isVis){
        QPen pen(Qt::yellow,lineWid/ptScale);
        painter->setBrush(Qt::yellow);
        painter->setPen(pen);
        painter->drawLine(QPointF(0,0), lef->pos());
        painter->drawLine(QPointF(0,0), rig->pos());

        QPen penCenter(Qt::black,lineWid/ptScale);
        painter->setBrush(color);
        painter->setPen(penCenter);
        qreal paitRadius = radius/ptScale;
        painter->drawEllipse(QPointF(0,0), paitRadius/2, paitRadius/2);
    }else{
        QPen penCenter(color,lineWid*1.5);
        painter->setPen(penCenter);
        qreal paitRadius = radius/ptScale;
        painter->drawEllipse(QPointF(0,0), paitRadius/2, paitRadius/2);
    }

}

void SpNode::setPreFrameNode(SpNode * n){
    preFrameNode = n;
    hasPreFrameNode = true;
}
void SpNode::setNextFrameNode(SpNode * n){
    nextFrameNode = n;
    hasNextFrameNode = true;
}
void SpNode::rmPreFrameNode(){
    preFrameNode = NULL;
    hasPreFrameNode = false;
}
void SpNode::rmNextFrameNode(){
    nextFrameNode = NULL;
    hasNextFrameNode = false;
}

void SpNode::setPosition(QPointF pos){
    position = pos;
//    this->setPos(position);
}

void SpNode::setPreNode(SpNode * node){
    hasPreNode = true;
    preNode = node;
}

void SpNode::setNextNode(SpNode * node){
    hasNextNode = true;
    nextNode = node;
}

void SpNode::rmPreNode(){
    hasPreNode = false;
    preNode = NULL;
}

void SpNode::rmNextNode(){
    hasNextNode = false;
    nextNode = NULL;
}

//void SpNode::mouseHoverEvent(QGraphicsSceneMouseEvent * event){
//    color = Qt::yellow;
//}
//void SpNode::mouseLeaveEvent(QGraphicsSceneMouseEvent * event){
//    color = Qt::red;
//}

SpNode* SpNode::closestPoint(QList<SpNode *> nodelist){
    qreal dis = 1000;
    SpNode* checkPoint = NULL;
    for(QList<SpNode *>::iterator iNode = nodelist.begin(); iNode!=nodelist.end();iNode++){
        qreal diff = this->distance((*iNode)->getPosition());
        if(diff<dis){
            dis = diff;
            checkPoint = (*iNode);
        }
    }
    return checkPoint;
}

bool SpNode::nodeNeighbouring(QList<SpNode *> nodelist){
    if(nodelist.size()<2) return false;
    qreal disPrePt = 1000;
    qreal disNextPt = 1000;
    SpNode * cltPt = closestPoint(nodelist);
    //    int indNode = nodelist.indexOf(cltPt);
    if(cltPt->hasPreNode){
        //        disPrePt = distance(cltPt->getPreNode()->getPosition());
        SpCurve* newCurve = new SpCurve(cltPt->getPreNode(),cltPt);
        disPrePt = this->disToCurve(newCurve->getCurvePt());
    }
    if(cltPt->hasNextNode){
        //        disNextPt = distance(cltPt->getNextNode()->getPosition());
        SpCurve* newCurve = new SpCurve(cltPt,cltPt->getNextNode());
        disNextPt = this->disToCurve(newCurve->getCurvePt());
    }
    if(disPrePt < disNextPt){
        this->setPreNode(cltPt->getPreNode());
        this->setNextNode(cltPt);
        cltPt->getPreNode()->setNextNode(this);
        cltPt->setPreNode(this);
        //        nodelist.insert(indNode,this);
    }
    else {
        this->setPreNode(cltPt);
        this->setNextNode(cltPt->getNextNode());
        cltPt->getNextNode()->setPreNode(this);
        cltPt->setNextNode(this);
        //        nodelist.insert(indNode+1,this);
    };
    return true;
}

//bool SpNode::nodeNeighbouringByCurve(QList<QList<QPoint>> curvelist){
//    qreal dis = 1000;
//    for(QList<SpCurve *>::iterator iNode = curvelist.begin(); iNode!=curvelist.end();iNode++){
//        qreal diff = this->disToCurve(*iNode);
//        if(diff<dis){
//            dis = diff;
//        }
//    }
//}

qreal SpNode::disToCurve(QList<QPointF> curve){
    qreal dis = 1000;
    for(QList<QPointF>::iterator iNode = curve.begin(); iNode!=curve.end();iNode++){
        qreal diff = this->distance((*iNode));
        if(diff<dis){
            dis = diff;
        }
    }
    return dis;
}

QList<qreal> SpNode::disToEach(QList<SpNode*> pts){
    QList<qreal> dis;
    if(pts.isEmpty()) return dis;

    for(QList<SpNode*>::iterator iNode = pts.begin(); iNode!=pts.end();iNode++){
        dis.append(this->distance(*iNode));
    }
    return dis;
}

QList<qreal> SpNode::disToEach(QList<QPointF> pts){
    QList<qreal> dis;
    if(pts.isEmpty()) return dis;

    for(int i=0; i<pts.size(); i++){
        dis.append(this->distance(pts[i]));
    }
    return dis;
}

QList<qreal> SpNode::normDis(QList<qreal> dis){
    QList<qreal> normdis;
    if(dis.isEmpty()) return dis;
    qreal max = *std::max_element(dis.begin(), dis.end());
    if(max==0) return dis;
    else{
        for(int i=0; i<dis.size(); i++){
            normdis.append(1-dis[i]/max);
        }
    }
    return normdis;
}

void SpNode::slted(){
    isSlted = true;
    color = QColor(0,176,240);
}
void SpNode::resetSlted(){
    isSlted = false;
    color = Qt::red;
}


void SpNode::updateScenePos(){
    this->setPosition(scenePos());
    this->getLef()->setPosition(this->getLef()->scenePos() - this->getPosition());
    this->getRig()->setPosition(this->getPosition() - this->getLef()->scenePos());
//    this->getLef()->setPos(this->getLef()->scenePos()-this->getPosition());
//    this->getRig()->setPos(this->getRig()->scenePos()-this->getPosition());
}

void SpNode::updatePositionToScene(){
    this->setPos(this->position);
    this->getLef()->setPos(this->getLef()->getPosition());
    this->getRig()->setPos(this->getRig()->getPosition());
}

void SpNode::setScale(qreal s){
    ptScale = s;
    this->getLef()->setScale(s);
    this->getRig()->setScale(s);
}

qreal SpNode::distance(QPointF b) {
    QPointF diff = this->getPosition() - b;
    return sqrt(diff.x()*diff.x() + diff.y()*diff.y());
}

qreal SpNode::distance(SpNode * b) {
    QPointF diff = this->getPosition() - b->getPosition();
    return sqrt(diff.x()*diff.x() + diff.y()*diff.y());
}

bool SpNode::posChangeTo(SpNode* pt){
    if(this->position.x()!= pt->position.x() || this->position.y()!= pt->position.y()) return true;
    if(this->getLef()->getPosition().x() != pt->getLef()->getPosition().x()
            || this->getLef()->getPosition().y() != pt->getLef()->getPosition().y()) return true;
    if(this->getRig()->getPosition().x() != pt->getRig()->getPosition().x()
            || this->getRig()->getPosition().y() != pt->getRig()->getPosition().y()) return true;
    return false;
}

void SpNode::copyFrom(SpNode* from){
    this->setColor(from->getColor());
    this->setPosition(from->getPosition());
    this->getLef()->setPosition(from->getLef()->getPosition());
    this->getRig()->setPosition(from->getRig()->getPosition());
    this->setScale(from->getScale());
    this->isSlted = from->isSlted;
    this->firstNode = from->isFirstNode();
}

QPointF SpNode::interp(QPointF from, QPointF to, float factor){
    return QPointF(from + (to-from)*factor);
}

SpNode* SpNode::interpNode(float factor){

//    QPointF from = this->getPosition();
//    QPointF fromLeft = this->getLef()->getPosition() + from;
    QPointF from = this->scenePos();
    QPointF fromLeft = this->getLef()->scenePos();

//    QPointF to = this->getNextNode()->getPosition();
//    QPointF toRight = to - this->getNextNode()->getRig()->getPosition();

    QPointF to = this->getNextNode()->scenePos();
    QPointF toRight = this->getNextNode()->getRig()->scenePos();

    QPointF from_fromLeft = interp(from,fromLeft,factor);
    QPointF fromLeft_toRight = interp(fromLeft,toRight,factor);
    QPointF toRight_to = interp(toRight,to,factor);

    QPointF newRight = interp(from_fromLeft,fromLeft_toRight,factor);
    QPointF newLeft = interp(fromLeft_toRight,toRight_to,factor);
    QPointF newCenter = interp(newRight,newLeft,0.5);

    return new SpNode(newCenter,newLeft-newCenter,
                      newRight-newCenter,this->getScale());

//    return inNode;
}

QPointF SpNode::leftToScene(){
    return QPointF(this->getLef()->getPosition()+this->getPosition());
}

QPointF SpNode::rightToScene(){
//    return QPointF(this->getPosition()-this->getRig()->getPosition());
    return QPointF(this->getPosition()+this->getRig()->getPosition());
}

void SpNode::centerPositionToScene(){
    this->setPos(this->getPosition());
}

void SpNode::setVis(bool vis){
    isVis = vis;
    if(isVis){
        lef->setVisible(true);
        rig->setVisible(true);
    }else{
        lef->setVisible(false);
        rig->setVisible(false);
    }
}
