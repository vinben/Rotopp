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

#ifndef DESIGNVIEW_H
#define DESIGNVIEW_H

#include <QWidget>
#include <QObject>
#include <QScrollBar>
#include <QGraphicsView>
#include <QWheelEvent>
#include <QDebug>
#include "include/designZone/DesignScene.hpp"

using namespace std;

class DesignView : public QGraphicsView
{
    Q_OBJECT
public:
    DesignView(QWidget* parent = NULL);
    bool flagZooming;
    bool flagPtsVis;
    void wheelEvent(QWheelEvent*);
    void zoomin(bool);
//    void zoomin(QPointF);
//    void keyPressEvent(QKeyEvent*);
    void setCurScene(DesignScene*);
    void setCurScene(int);
    DesignScene* getCurScene(){return curScene;}
    QList<DesignScene*>* getScenes(){return &scenes;}
    void setCurveColor(QColor);
    void resetImageProperty();
    void setImagePropertyAllScene(int, int, int);
    void fitSceneToView();
    void hidePointsAllScenes();
    void showPointsAllScenes();

signals:
    void pushScenes(QList<DesignScene*>);
    void pushInitSniperView(QImage*);
    void pushRecheckDesignActions();

public slots:
    void initScenes(QList<QImage*>);
    void changeScene(qreal);
    void zoomin(QPointF);
    void setFlagZooming(bool);
    void updateNodesAllScene(TrackerData3);
    bool insertGlobalPtAllScene(int, int, float);
    bool setClosedCurveAllScene(int);
    bool setGlobalPtDeleteAllScene(int, int);
    bool globalPtsDeleteAllScene();
    bool checkPtsNumConsKeyframes(QList<int>);
    int getNumAllPoints();
    void checkClosedCurveAllScenes();
    void resetRotationAllScenes();
    void copyNodeListFromTo(int,int);
    void updateRightClick(QPointF);
    void dragView(QPointF c);
    void setDragScene(bool);
    void setPtsVisAllScenes();
    void setPtsVisAllScenes(bool);
//    void smoothPts();
//    void cuspPts();

    void setCurFlagSelect(bool checked){this->curScene->setFlagSelection(checked);}
    void setCurFlagMove(bool checked){this->curScene->setFlagMove(checked);}
    void setCurFlagDelete(bool checked){this->curScene->setFlagDelete(checked);}
    void setCurFlagDrag(bool checked){this->curScene->setFlagDragPts(checked);}
    void setCurFlagSolverDrag(bool checked){this->curScene->setFlagSolverDragPts(checked);}
    void setCurFlagCreate(bool checked){this->curScene->setFlagCreate(checked);}
    void setCurFlagModify(bool checked){this->curScene->setFlagModPoint(checked);}
    void setCurFlagPoly(bool checked){this->curScene->displayPoly(checked);}
    void setCurFlagRotation(bool checked){this->curScene->setRotateMode(checked);}

    QList<bool> getCurFlags();
    void cleanCurScene();
private:
    DesignScene* curScene;
    QList<DesignScene*> scenes;
    QPointF pos_rightmove;
    QPointF pos_rightclick;
    bool flagDragScene;
};

#endif // DESIGNVIEW_H
