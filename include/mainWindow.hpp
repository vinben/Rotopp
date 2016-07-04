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

#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QtWidgets>
#include <QFile>
#include <QAction>
#include <QProgressDialog>
#include <QTimer>
#include <vector>
#include <unistd.h>
#include <stdio.h>
#include "include/RotoConfig.h"
#include "include/ImgIcon.hpp"
#include "include/utils.hpp"
#include "include/dialogs/CreateProjDialog.hpp"
#include "include/designZone/SpNode.hpp"
#include "include/designZone/SniperView.hpp"
#include "include/dialogs/SaveProjDialog.hpp"
#include "include/dialogs/LoadProjDialog.hpp"
#include "include/dialogs/LoadNukeProjDialog.hpp"
#include "include/dialogs/ExportProjDialog.hpp"
#include "include/dialogs/ImageProcDialog.hpp"
#include "include/FrameFlow.hpp"
#include "include/RotoStatus.hpp"
#include "include/RotoIO.hpp"
#include "include/RotoCore.hpp"
#include "include/ConfigIO.hpp"

class MainWindow : public QMainWindow
{
    Q_OBJECT

private:

    QTimer* timerAutoSaving;
    QTimer* timerAutoExportLoging;
    QTimer* timerPlay;

    bool isProjActive;
    bool saved;
    bool isPlayingFwd;
    bool isPlayingBck;

    ConfigIO* config;
    ImageList imgs;
    ARotoCurve* rotoCurve;
    KeyFrames* keyframes;
//    QMap<QString,ARotoCurve*> rotoStack;

    RotoStatus* statStack;
    RotoIO* rotoDataIO;
    RotoCore* rotocore;

    DesignSceneList* scenes;

    SniperView* snipView;
    DesignView* view;
    FrameFlow* flowList;

    QTemporaryDir *workingDir;
    QString projectName;
    QString curveName;
    QString projectPath;

    SolverType solverType;

    int frequenceFPS;

    QToolBar *designToolBar;
    QToolBar *rotoToolBar;

    QMenu *fileMenu;
    QMenu *editMenu;
    QMenu *settingMenu;
    QMenu *aboutMenu;

    QPushButton *buttNewProj;
    QPushButton *buttOpenProj;
    QWidget *entrerWidget;
    QWidget *widget;
//    QHBoxLayout *imageLayout;
    QVBoxLayout *mainLayout;
    QStackedLayout *stackedLayout;

    QActionGroup *designViewActionGroup;
    QAction *designViewSelectAction;
    QAction *designViewCreateAction;
    QAction *designViewMoveAction;
    QAction *designViewModifyAction;
    QAction *designViewDragAction;
    QAction *designViewSolverDragAction;
    QAction *designViewRotateAction;
    QAction *designViewDeleteAction;
    QAction *designViewClearAction;
    QAction *designViewColorAction;
    QAction *designViewImageProcAction;

    QAction *designViewPlayFwdAction;
    QAction *designViewPlayBckAction;
    QAction *designViewPlayStopAction;

    QAction *designViewCuspPts;
    QAction *designViewSmoothPts;

    QAction *designViewPolyAction;
    QAction *designViewShowPtsAction;
    QAction *designViewNextFrameAction;
    QAction *designViewPreFrameAction;
    QAction *designViewZoomAction;
    QAction *designViewUndoAction;
    QAction *rotoKeyframeAction;
    QAction *rotoUnKeyframeAction;
    QAction *rotoActiveContour;
    QAction *rotoCopyCureve;

    QActionGroup *rotoSolverTypeGroup;
    QAction *rotoSolverTypeTrackerGPLVMNextEdit;
    QAction *rotoSolverTypeTrackerGPLVM;
    QAction *rotoSolverTypeTracker;
    QAction *rotoSolverTypeLinearInterp;

    QAction *settingFilledCurveMap;

    QAction *newAction;
    QAction *openAction;
    QAction *saveAction;
    QAction *saveAsAction;
    QAction *importAction;
    QAction *importNukeAction;
    QAction *exportAction;
    QAction *exportAlphaMapAction;
    QAction *exportCurveMapAction;
    QAction *closeApplicationAction;

    QAction *aboutAction;

public:
    MainWindow();
    ~MainWindow();

    void initWidget();
    void createEntrerWindow();
    void createFrameFlow(QList<QImage*>);
    void createLayouts();
    void createAction();
    void createToolBar();
    void createMenus();

    void initSignals();
    void initConfig();
    void startTimerAutoSaving();
    void startTimerAutoExportLoging();
    void initTimerPlay();
    void initProgram();
    QString getProjectName(){return projectName;}
    void setProjectName(QString string){this->projectName = string;}
    void updateKeyframe();
    bool runSolving();
    void keyPressEvent(QKeyEvent*);
    void jumpToNextFrame();
    void jumpToPreFrame();
//    void keyReleaseEvent(QKeyEvent*);
//    void wheelEvent(QWheelEvent* event);

signals:
    void loadImgDone(QList<QImage*>);

public slots:
    void reloadImgFrameFlow();
    void linkRotoCurveFromView(QList<DesignScene*>);
    void linkDesignActionsFromImgIcons(QList<ImgIcon*>);
    void linkImgIconFromSceneCurves(QList<DesignScene*>);
    void setRotoSolverTypeTrackerGPLVMNextEdit(bool);
    void setRotoSolverTypeTrackerGPLVM(bool);
    void setRotoSolverTypeTracker(bool);
    void setRotoSolverTypeLinearInterp(bool);
    void updateSolverType(SolverType);
    void resetDesignActions();
    void reCheckDesignActions();
    void initGPLVM();
    void copyCurveFromClosetKeyframe();
    void SolveSingleFrame();
    void setKeyframe();
    void resetKeyframe();
    void cleanCurScene();
    void closeApp();
    void autoSave();
    void autoExportLog();
    void playFwd(bool);
    void playNextFrame();
    void playBck(bool);
    void playPreviousFrame();
    void stopPlaying();
    void jumpToClosestKeyframe(int,bool);
    void backToSelection();
    void backToShowPts();
    void setIsFilledCurveMap(bool);


private slots:

    void undo();
    void about();
    void loadProject();
    bool newProject();
    void importProject();
    bool importNukeProject();
    bool exportProject();
    bool exportAlphaMap();
    bool exportCurveMap();
    void saveProject();
    void saveAsProject();
    void createProject();
    void createNukeProject();
    void openProject();
    void runActiveContour();
    void openColorDialog();
    void changeImageProc();
    void smoothPts();
    void cuspPts();

};
#endif
