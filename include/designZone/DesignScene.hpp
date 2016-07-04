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

#ifndef DESIGNSCENE_H
#define DESIGNSCENE_H

#include <QGraphicsItem>
#include <QGraphicsRectItem>
#include <QDebug>
#include <QList>
#include <QtMath>
#include <QPointF>
#include <QRectF>
#include <QPolygonF>
#include <QPainter>
#include <QPainterPath>
#include <QWidget>
#include <iostream>
#include <QGraphicsScene>
#include <QKeyEvent>
#include <QImage>
#include <QGraphicsSceneMouseEvent>
#include <QGuiApplication>
#include "SpNode.hpp"
#include "SpRect.hpp"
#include "SpRectRotate.hpp"
#include "SpNodeRotate.hpp"
#include "SpRotator.hpp"
#include "SpCurve.hpp"
#include "SpPoly.hpp"
#include "include/ImgIcon.hpp"
#include "include/utils.hpp"

using namespace std;

//typedef QList<SpNode*>* NodeList;

class DesignScene : public QGraphicsScene{

    Q_OBJECT

private:
    int valueContrast;
    int valueBrightness;
    int valueGama;
    EditType status;
    QColor curveColor;
    qreal scale;
    SpPoly* poly;
    QImage* img;
    QImage dispImg;
    NodeList* points;
    NodeList sltPts;
    CurveList curves;
    QPointF pos_click;
    QPointF pos_release;
    QPointF pos_move;
    QList<QPointF> origPosDragList;
    QVector<QPointF> turnSpNodeList(NodeList nodes);

signals:
    void pushEditing();
    void pushNoEdit();
    void pushSuggest();
    void pushCursor(QPointF);
    void pushRightMove(QPointF);
    void pushMidMove(QPointF);
    void pushRightClick(QPointF);
    void pushCurves(QList<SpCurve*>);
    void pushStatus();
    bool pushClosedCurve(int);
    void pushGlobalPt(int,int,float);
    void pushGlobalDelect(int,int);
    void pushInitGPLVM();
    void pushDragScene(bool);
    void pushSolveSingleFrame();
    void pushSelectionMode();
    void pushShowPts();

public slots:
    void displayPoly(bool);
    void setRotateMode(bool);
    void setFlagMove(bool);
    void setFlagCreate(bool);
    void setFlagSolverDragPts(bool);
    void setFlagShowPoly(bool);
    void setFlagModPoint(bool);
    void setFlagDelete(bool);
    void setFlagDragPts(bool);
    void setFlagRotation(bool);
    void setFlagSelection(bool);
    void cleanView();

public:
    bool flagMove;
    bool flagCreate;
    bool flagShowPoly;
    bool flagModPoint;
    bool flagDelete;
    bool flagDragPts;
    bool flagSolverDragPts;
    bool flagRotation;
    bool flagSelection;

    bool isPtsVisible;

    bool hasImgProcChange;
    bool hasUpdateImgProcChange;
    bool hasClosedCurve;
    bool hasPoly;
    bool hasSltRect;
    bool hasSltPts;
    bool hasSelectedPt;
    bool hasSelectedLeft;
    bool hasSelectedRight;
    bool hasDragList;
    bool hasAvaiPt;
    bool hasRectRotate;


    int indSltPt;
    SpNode * sltPoint;
    SpRect * sltRect;
    SpRectRotate * rectRotate;
    SpRotator* rotator;
    DesignScene(){}
    DesignScene(QImage*);

    void setPoints(NodeList* p){points = p;}
    QList<SpNode*>* getPoints(){return points;}
    QList<int> getIndSltPts();
    QImage* loadImg(QString);
    void drawBackground(QPainter * painter, const QRectF & rect);
    QPointF getMouseMove(){return pos_move;}
    SpPoly* getPoly(){return poly;}

    void translatePts(QPointF,QList<QPointF>,QList<SpNode*>,bool);
    void translateSltPts(QPointF, SpNode*, QList<QPointF>, QList<SpNode*>);
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
//    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event);
//    void keyPressEvent(QKeyEvent * keyEvent);
//    void keyReleaseEvent(QKeyEvent * keyEvent);

    void setStatus(EditType);

    void cuspPoints();
    void smoothPoints();
    bool copyNodeList(NodeList* curNodes, NodeList* newNodes);
    bool copyNodeList(NodeList* curNodes);
    bool updateNodePos(VecNodeList, bool);
    void initACurve(int n);
    bool updateNodePos();
    void cleanNodes();
    void cleanFlag();
    void resetFlags();
    void resetHasFlags();
    void resetFlagsNotPoly();
    void cleanCurve();
    void updateScene();
    void rerenderPoints(QList<SpNode*>);
    void rerenderScene();
    void CenterPositionToSceneAllPts();

    void addNode(SpNode*);
    void addCurve(SpCurve*);

    void addNodeList(NodeList*);
//    void addCurve(CurveList*);

    void resetDefaultFlags();

    void setCurveColor(QColor);
    bool checkCloseCurve();
    bool checkInPtsArea(QList<SpNode*>,QPointF);
    void hidePoints();
    void showPoints();

    int getSelectedPt();
    int getSelectedPt(SpNode*);
    void resetSelectedPt();

    bool setOrigPosDragList(QList<SpNode*>);
    void resetDragList();

    void setRectRotate(QList<SpNode*>);
    void resetRectRotate();

    void selectAllPts();
    bool setSltPts();
    void resetSltPts();

    void closeShape();

    QImage* getImg(){return img;}
    void setImg(QImage* m){img = m;}

    QImage getAlphaMap(bool);
    QImage getCurveMap(bool,bool,float);

    qreal getScale(){return scale;}

    void setSltRect(QPointF);
    void resetSltRect();

    void addAvaiPoint(SpNode*);

    bool callRotationMode();
    bool resetCallRotationMode();

    bool rszPtsAndLine(qreal);

    bool showPolys();
    bool unshowPolys();

    QPointF getClick(){return pos_click;}
    QPointF getRelease(){return pos_release;}

    bool nodeNeighbouringByCurve(SpNode*, QList<SpCurve*>);
    bool insertGlobalPt(SpNode*);

    void updateDisplayImg();
    int getContrast(){return valueContrast;}
    void setContrast(int c){valueContrast=c;}
    int getBrightness(){return valueBrightness;}
    void setBrightness(int b){valueBrightness=b;}
    int getGamma(){return valueGama;}
    void setGamma(int g){valueGama=g;}

    void setImgProcChange(bool s){hasImgProcChange=s;}
    void setUpdateImgProcChange(bool s){hasUpdateImgProcChange=s;}
};

typedef QList<DesignScene*> DesignSceneList;

#endif
