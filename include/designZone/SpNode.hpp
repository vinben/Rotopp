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

#ifndef SPNODE_H
#define SPNODE_H

#include <QGraphicsItemGroup>
#include <QPointF>
#include <QRectF>
#include <QPainter>
#include <QPainterPath>
#include <QWidget>
#include <iostream>
#include <QtMath>
#include "SpHook.hpp"
//#include "SpCurve.h"

using namespace std;

class SpNode:public QGraphicsItemGroup {
private:
    SpHook * lef;
    SpHook * rig;
    SpNode * preNode;
    SpNode * nextNode;
    SpNode * preFrameNode;
    SpNode * nextFrameNode;
    bool firstNode;
    QColor color;
    qreal lineWid;
    qreal ptScale;

public:
    SpNode(){}
    SpNode(SpNode*);

    SpNode(QPointF, QPointF, QPointF, qreal s=1);
    ~SpNode();

    void initNode(QPointF, QPointF, QPointF, qreal s=1);

    bool isVis;
    bool isSlted;
    bool hasPreNode;
    bool hasNextNode;
    bool hasPreFrameNode;
    bool hasNextFrameNode;
    qreal radius;
    QPointF position;

//    background painting
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

//    methods for setting basic properties
    qreal getSceneRadius(){return radius/ptScale;}
    void setColor(QColor c){color = c;}
    QColor getColor(){return color;}
    void setLineWid(qreal w){lineWid = w;}
    void setRadius(qreal r){radius=r;}
    QPointF getPosition(){return position;}
    void setPosition(QPointF);

    qreal getRadius(){return radius/ptScale;}

    SpHook * getLef(){return lef;}
    SpHook * getRig(){return rig;}
    SpNode * getPreNode(){return preNode;}
    SpNode * getNextNode(){return nextNode;}
    bool isFirstNode(){return firstNode;}
    void setFirstNode(){firstNode = true;}
    void resetFirstNode(){firstNode = false;}

    void setVis(bool);

//    methods for setting correspondent nodes of pre/next frame
    void setPreFrameNode(SpNode *);
    void setNextFrameNode(SpNode *);
    void rmPreFrameNode();
    void rmNextFrameNode();


//    methods for setting prenode/nextnode of current frame
    void setPreNode(SpNode *);
    void setNextNode(SpNode *);
    void rmPreNode();
    void rmNextNode();

    qreal getScale(){return ptScale;}

    void copyFrom(SpNode *);
    bool nodeNeighbouring(QList<SpNode *>);
    qreal distance(QPointF);
    qreal distance(SpNode*);
    QList<qreal> normDis(QList<qreal>);
    QList<qreal> disToEach(QList<SpNode*>);
    QList<qreal> disToEach(QList<QPointF>);
    qreal disToCurve(QList<QPointF>);
    SpNode* closestPoint(QList<SpNode *>);
    bool posChangeTo(SpNode*);
    void updateScenePos();
    void updatePositionToScene();

    QPointF leftToScene();
    QPointF rightToScene();
    void centerPositionToScene();

    QPointF interp(QPointF,QPointF,float);
    SpNode* interpNode(float);
    void slted();
    void resetSlted();
    void setScale(qreal);
};
#endif
