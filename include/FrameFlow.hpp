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

#ifndef FRAMEFLOW_H
#define FRAMEFLOW_H

#include <QObject>
#include <QWidget>
//#include <QMainWindow>
#include <QListWidget>
//#include "include/mainWindow.hpp"
#include "include/ImgIcon.hpp"
#include "include/designZone/DesignView.hpp"

class FrameFlow : public QListWidget {
    Q_OBJECT
private:
    QList<ImgIcon*> icons;
    int indCurScene;
    int nScene;
public:
    FrameFlow();
    void setSceneConnection(DesignView *);
    void setIconConnection();
    void setDesignActionsConnnection();
    void addImgIcon(QImage*);
    void initFrameFlow(QList<QImage*>);
    QList<ImgIcon*> getIcons(){return icons;}
    int getindCurScene(){return indCurScene;}
    int getNumScene(){return nScene;}
    void labelKeyframeForCurScene();
    void resetImgIconStatus();
    void keyPressEvent(QKeyEvent*);
    void setKeyframeAll();
    EditType getEditTypeCurIcon();
signals:
    void pushImgIcons(QList<ImgIcon*>);
public slots:
    void updateCurScene(qreal);
    void setNextFrame();
    void setPreFrame();
//    void setCurFrame(int);
    QList<int> getIndKeyframes();
    QList<int> getIndEditing();
    void setIndKeyframes(QList<int>);
    void setNextEdit(QList<int>);
    StatusList getStatusList();
    void setAllStatus(StatusList);
    void setAllSuggestOnNotKeyframes();
    void setStatusCurIcon(EditType);
    void zoomin(float);
    void jumpToClosestKeyframe(int, bool);
    int getIndClosestKeyframe(int, bool);
};

#endif // FRAMEFLOW_H
