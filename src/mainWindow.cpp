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

#include "include/mainWindow.hpp"

DEFINE_double(s, 1e-3, "Scaling factor (lower is less regularised) for the manifold smoothness prior.");
DEFINE_double(a, 1.0, "Scaling factor (lower is less regularised) for the manifold anchor parameter for fixing the keyframes.");
DEFINE_double(r, 10, "Scaling factor for the huber loss function (loosely in pixels). Set to a negative number to disable.");
DEFINE_double(trans_weight, 0.1, "Translation smoothness weighting factor.");
DEFINE_double(rot_weight, 1.0, "Rotation smoothness weighting factor.");
DEFINE_double(scale_weight, 0.1, "Scale smoothness weighting factor.");

//DEFINE_bool(do_linear_interp_RT, false, "Do linear interp with RandT.");
DEFINE_bool(testGradients, false, "Test the optimisation gradients (use with -logtostderr).");
DEFINE_bool(overrideKeys, false, "Override key frames with our estimate.");
DEFINE_bool(use_planar_tracker_weights, true, "Use lin interpolated weights for forwards and backwards planar tracker data.");
//DEFINE_bool(no_rigid_body, false, "Turn off rotation and translation separately.");
//DEFINE_bool(just_interpolate, false, "Turn this on to just return interpolations but no optimisation in the solver.");
DEFINE_bool(skip_initial_optimisation, false, "Skip the initial NULL loss optimisation (sort of pre-initialisation).");
//DEFINE_bool(do_linear_interp, false, "Skip the alignment, manifold and solver and just do linear interp.");

using namespace std;

MainWindow::MainWindow() {
    initProgram();
    createEntrerWindow();
    initWidget();
    startTimerAutoSaving();
    startTimerAutoExportLoging();
    initTimerPlay();
    initSignals();
//    initConfig();
}

MainWindow::~MainWindow() { }

void MainWindow::initProgram(){
    isProjActive = false;
    saved = false;
    solverType = TRACKER_GPLVM_NEXTEDIT;

    scenes = new DesignSceneList();
    rotoCurve = new ARotoCurve();
    keyframes = new KeyFrames();
    view = new DesignView();
    snipView = new SniperView();
    flowList = new FrameFlow();
    statStack = new RotoStatus(view, flowList, keyframes,&solverType);
    rotocore = new RotoCore(view, flowList, &imgs);
    rotoDataIO = new RotoIO(view, flowList, statStack, &imgs, rotocore);

    this->setWindowIcon(QIcon(":/images/poly.png"));
    this->setWindowTitle(tr("Roto++"));
}

void MainWindow::initWidget(){
    widget = new QWidget;
    widget->setWindowIcon(QIcon(":/images/poly.png"));
    createAction();
    createToolBar();
    createMenus();
    createLayouts();
    widget->setLayout(mainLayout);
}

void MainWindow::initConfig(){
    config = new ConfigIO("RotoConfig.rotoInit");
    if(config->isIniFileExist()){

    }
}

//void init

void MainWindow::linkRotoCurveFromView(QList<DesignScene *> ds) {
    this->rotoCurve->clear();
    for (QList<DesignScene *>::iterator iNode = ds.begin(); iNode != ds.end(); iNode++) {
        this->rotoCurve->append((*iNode)->getPoints());
        this->scenes->append(*iNode);
        QObject::connect((*iNode), SIGNAL(pushCursor(QPointF)), snipView, SLOT(setCenter(QPointF)));
        QObject::connect((*iNode), SIGNAL(pushGlobalPt(int, int, float)), view, SLOT(insertGlobalPtAllScene(int, int, float)));
        QObject::connect((*iNode), SIGNAL(pushClosedCurve(int)), view, SLOT(setClosedCurveAllScene(int)));
        QObject::connect((*iNode), SIGNAL(pushGlobalDelect(int, int)), view, SLOT(setGlobalPtDeleteAllScene(int, int)));
        QObject::connect((*iNode), SIGNAL(pushRightClick(QPointF)), view, SLOT(updateRightClick(QPointF)));
        QObject::connect((*iNode), SIGNAL(pushRightMove(QPointF)), view, SLOT(dragView(QPointF)));
        QObject::connect((*iNode), SIGNAL(pushMidMove(QPointF)), view, SLOT(zoomin(QPointF)));
        QObject::connect((*iNode), SIGNAL(pushDragScene(bool)), view, SLOT(setDragScene(bool)));

        QObject::connect((*iNode), SIGNAL(pushInitGPLVM()), this, SLOT(initGPLVM()));
        QObject::connect((*iNode), SIGNAL(pushSolveSingleFrame()), this, SLOT(SolveSingleFrame()));
        QObject::connect((*iNode), SIGNAL(pushSelectionMode()), this, SLOT(backToSelection()));
        QObject::connect((*iNode), SIGNAL(pushShowPts()), this, SLOT(backToShowPts()));

    }
//    qDebug() << "number of frames linked to rotoCurve: " << rotoCurve->size();
}

void MainWindow::linkDesignActionsFromImgIcons(QList<ImgIcon *> is) {
    for (QList<ImgIcon *>::iterator iNode = is.begin(); iNode != is.end(); iNode++) {
        QObject::connect((*iNode), SIGNAL(pushImgIconClicked(qreal)), flowList, SLOT(updateCurScene(qreal)));
    }
}

void MainWindow::linkImgIconFromSceneCurves(QList<DesignScene *> ds) {
    QList<ImgIcon *> icons = flowList->getIcons();
    for (int iScene = 0; iScene < ds.size(); iScene++) {
        QObject::connect(ds[iScene], SIGNAL(pushCurves(QList<SpCurve *>)), icons[iScene], SLOT(updateCurves(QList<SpCurve *>)));
        QObject::connect(ds[iScene], SIGNAL(pushEditing()), icons[iScene], SLOT(setEditingStatus()));
        QObject::connect(ds[iScene], SIGNAL(pushNoEdit()), icons[iScene], SLOT(setNoEditStatus()));
        QObject::connect(ds[iScene], SIGNAL(pushStatus()), statStack, SLOT(pushStatus()));
    }
}

void MainWindow::initSignals() {
    QObject::connect(rotoDataIO, SIGNAL(pushImgs()), this, SLOT(reloadImgFrameFlow()));
    QObject::connect(rotoDataIO, SIGNAL(pushSolverType(SolverType)), this, SLOT(updateSolverType(SolverType)));
    QObject::connect(statStack, SIGNAL(pushSolverType(SolverType)), this, SLOT(updateSolverType(SolverType)));

    QObject::connect(this, SIGNAL(loadImgDone(QList<QImage *>)), view, SLOT(initScenes(QList<QImage *>)));
    QObject::connect(this, SIGNAL(loadImgDone(QList<QImage *>)), rotocore, SLOT(loadImgs(QList<QImage *>)));
    QObject::connect(view, SIGNAL(pushScenes(QList<DesignScene *>)), this, SLOT(linkRotoCurveFromView(QList<DesignScene *>)));
    QObject::connect(view, SIGNAL(pushScenes(QList<DesignScene *>)), this, SLOT(linkImgIconFromSceneCurves(QList<DesignScene *>)));
    QObject::connect(view, SIGNAL(pushInitSniperView(QImage * )), snipView, SLOT(changeImg(QImage * )));
    QObject::connect(view, SIGNAL(pushRecheckDesignActions()), this, SLOT(reCheckDesignActions()));

    QObject::connect(designViewSelectAction, SIGNAL(toggled(bool)), view, SLOT(setCurFlagSelect(bool)));
    QObject::connect(designViewCreateAction, SIGNAL(toggled(bool)), view, SLOT(setCurFlagCreate(bool)));
    QObject::connect(designViewMoveAction, SIGNAL(toggled(bool)), view, SLOT(setCurFlagMove(bool)));
    QObject::connect(designViewDragAction, SIGNAL(toggled(bool)), view, SLOT(setCurFlagDrag(bool)));
    QObject::connect(designViewSolverDragAction, SIGNAL(toggled(bool)), view, SLOT(setCurFlagSolverDrag(bool)));
    QObject::connect(designViewDeleteAction, SIGNAL(toggled(bool)), view, SLOT(setCurFlagDelete(bool)));
    QObject::connect(designViewModifyAction, SIGNAL(toggled(bool)), view, SLOT(setCurFlagModify(bool)));
    QObject::connect(designViewRotateAction, SIGNAL(toggled(bool)), view, SLOT(setCurFlagRotation(bool)));
    QObject::connect(designViewPlayFwdAction, SIGNAL(toggled(bool)), this, SLOT(playFwd(bool)));
    QObject::connect(designViewPlayBckAction, SIGNAL(toggled(bool)), this, SLOT(playBck(bool)));
    QObject::connect(designViewPlayStopAction, SIGNAL(triggered()), this, SLOT(stopPlaying()));
    QObject::connect(designViewPolyAction, SIGNAL(toggled(bool)), view, SLOT(setCurFlagPoly(bool)));
    QObject::connect(designViewShowPtsAction, SIGNAL(toggled(bool)), view, SLOT(setPtsVisAllScenes(bool)));
    QObject::connect(designViewZoomAction, SIGNAL(toggled(bool)), view, SLOT(setFlagZooming(bool)));
    QObject::connect(designViewColorAction, SIGNAL(triggered()), this, SLOT(openColorDialog()));
    QObject::connect(designViewImageProcAction, SIGNAL(triggered()), this, SLOT(changeImageProc()));
    QObject::connect(designViewClearAction, SIGNAL(triggered()), this, SLOT(cleanCurScene()));
    QObject::connect(designViewPreFrameAction, SIGNAL(triggered()), flowList, SLOT(setPreFrame()));
    QObject::connect(designViewPreFrameAction, SIGNAL(triggered()), statStack, SLOT(pushStatus()));
    QObject::connect(designViewNextFrameAction, SIGNAL(triggered()), flowList, SLOT(setNextFrame()));
    QObject::connect(designViewNextFrameAction, SIGNAL(triggered()), statStack, SLOT(pushStatus()));
    QObject::connect(designViewUndoAction, SIGNAL(triggered()), this, SLOT(undo()));
    QObject::connect(designViewCuspPts, SIGNAL(triggered()), this, SLOT(cuspPts()));
    QObject::connect(designViewSmoothPts, SIGNAL(triggered()), this, SLOT(smoothPts()));

    QObject::connect(rotoKeyframeAction, SIGNAL(triggered()), this, SLOT(setKeyframe()));
    QObject::connect(rotoUnKeyframeAction, SIGNAL(triggered()), this, SLOT(resetKeyframe()));
    QObject::connect(rotoCopyCureve, SIGNAL(triggered()), this, SLOT(copyCurveFromClosetKeyframe()));
    QObject::connect(rotoActiveContour, SIGNAL(triggered()), this, SLOT(runActiveContour()));

    QObject::connect(rotoSolverTypeTrackerGPLVMNextEdit, SIGNAL(toggled(bool)), this,
                     SLOT(setRotoSolverTypeTrackerGPLVMNextEdit(bool)));
//    QObject::connect(rotoSolverTypeTrackerGPLVM, SIGNAL(toggled(bool)), this,
//                     SLOT(setRotoSolverTypeTrackerGPLVM(bool)));
    QObject::connect(rotoSolverTypeTracker, SIGNAL(toggled(bool)), this, SLOT(setRotoSolverTypeTracker(bool)));
    QObject::connect(rotoSolverTypeLinearInterp, SIGNAL(toggled(bool)), this,
                     SLOT(setRotoSolverTypeLinearInterp(bool)));
    QObject::connect(settingFilledCurveMap, SIGNAL(toggled(bool)), this,
                     SLOT(setIsFilledCurveMap(bool)));
}

void MainWindow::startTimerAutoSaving() {
    timerAutoSaving = new QTimer();
    timerAutoSaving->start(20000);
    QObject::connect(timerAutoSaving, SIGNAL(timeout()), this, SLOT(autoSave()));
}

void MainWindow::startTimerAutoExportLoging() {
    timerAutoExportLoging = new QTimer();
    timerAutoExportLoging->start(10000);
    QObject::connect(timerAutoExportLoging, SIGNAL(timeout()), this, SLOT(autoExportLog()));
}

void MainWindow::initTimerPlay(){
    timerPlay = new QTimer();
    QObject::connect(timerPlay, SIGNAL(timeout()), this, SLOT(playNextFrame()));
    QObject::connect(timerPlay, SIGNAL(timeout()), this, SLOT(playPreviousFrame()));
}


void MainWindow::keyPressEvent(QKeyEvent *keyEvent) {
    if (isProjActive && !imgs.isEmpty()) {
        this->setFocus();
        if(keyEvent->key() == Qt::Key_Delete || keyEvent->key() == Qt::Key_Space){
            if(designViewPolyAction->isChecked()){
                designViewPolyAction->toggled(false);
                designViewPolyAction->setChecked(false);
            }
            if(designViewRotateAction->isChecked())
                backToSelection();

            view->globalPtsDeleteAllScene();
//            qDebug() << "delete selected points: size:" << inds.size();
        }
        else if(keyEvent->modifiers() == Qt::KeyboardModifier::AltModifier
                && (keyEvent->key() == Qt::Key_Left
                    || keyEvent->key() == Qt::Key_Z)){
            jumpToClosestKeyframe(flowList->getindCurScene(), false);
//            qDebug() << "jump to previous keyframe";
        }
        else if(keyEvent->modifiers() == Qt::KeyboardModifier::AltModifier
                && (keyEvent->key() == Qt::Key_Right
                    || keyEvent->key() == Qt::Key_X)){
            jumpToClosestKeyframe(flowList->getindCurScene(), true);
//            qDebug() << "jump to next keyframe";
        }
        else if(keyEvent->modifiers() == Qt::KeyboardModifier::ControlModifier
                && keyEvent->key() == Qt::Key_A){
                view->getCurScene()->selectAllPts();
        }
        else if(keyEvent->modifiers() == Qt::KeyboardModifier::ShiftModifier
                && keyEvent->key() == Qt::Key_H){
//            qDebug() << "Hide solver mode";
            if(rotoSolverTypeGroup->isEnabled()) rotoSolverTypeGroup->setEnabled(false);
            else rotoSolverTypeGroup->setEnabled(true);
        }
        else if((keyEvent->key() == Qt::Key_Enter
                 || keyEvent->key() == Qt::Key_Return)
                && view->getCurScene()->getPoints()->size() > 1){
            view->setClosedCurveAllScene(view->getCurScene()->getPoints()->size());
        }
        else if (keyEvent->key() == Qt::Key_Z || keyEvent->key() == Qt::Key_Left) {
            flowList->setPreFrame();
        }
        else if (keyEvent->key() == Qt::Key_X || keyEvent->key() == Qt::Key_Right) {
            flowList->setNextFrame();
        }
        else if (keyEvent->key() == Qt::Key_Equal || keyEvent->key() == Qt::Key_2 ) {
            if(!designViewZoomAction->isChecked()){
                designViewZoomAction->setChecked(true);
                designViewZoomAction->toggled(true);
            }
            view->zoomin(true);
            if(designViewZoomAction->isChecked()){
                designViewZoomAction->setChecked(false);
                designViewZoomAction->toggled(false);
            }
        }
        else if (keyEvent->key() == Qt::Key_Minus || keyEvent->key() == Qt::Key_1) {
            if(!designViewZoomAction->isChecked()){
                designViewZoomAction->setChecked(true);
                designViewZoomAction->toggled(true);
            }
            view->zoomin(false);

            if(designViewZoomAction->isChecked()){
                designViewZoomAction->setChecked(false);
                designViewZoomAction->toggled(false);
            }
        }
        else if(keyEvent->key() == Qt::Key_F){
            if(isProjActive && !view->getScenes()->isEmpty())
                view->fitSceneToView();
        }
    }
}

void MainWindow::cleanCurScene() {
    if(designViewPolyAction->isChecked()) {
        designViewPolyAction->toggled(false);
        designViewPolyAction->setChecked(false);
    }
    view->cleanCurScene();
    reCheckDesignActions();
}

void MainWindow::updateKeyframe() {
    QList<int> indKeys = flowList->getIndKeyframes();
    if (!indKeys.isEmpty()) {
        keyframes->clear();
        for (QList<int>::iterator iNode = indKeys.begin(); iNode != indKeys.end(); iNode++)
            (*keyframes)[*iNode] = (*rotoCurve)[*iNode];
    }
}

void MainWindow::initGPLVM() {
    QList<int> indKeys = flowList->getIndKeyframes();
    int indLastFrame = flowList->getNumScene() - 1;
    if (!indKeys.contains(0) || !indKeys.contains(indLastFrame))
        QMessageBox::information(this, tr("Error"), "To initialize the manifold both the first frame and the last "
                "frame have to be labeled as keyframes.");
//    else rotocore->buildGPLVM_MotionEst();
    else rotocore->buildGPLVM_GrabDrag();
}

void MainWindow::SolveSingleFrame() {
    DesignScene *scene = view->getCurScene();
    QList<int> indKeys = flowList->getIndKeyframes();
    int indLastFrame = flowList->getNumScene() - 1;
    int indCurScene = flowList->getindCurScene();
    if (!indKeys.contains(0) || !indKeys.contains(indLastFrame)) {
        scene->updateNodePos();
        QMessageBox::information(this, tr("Error"), "To initialize the manifold both the first frame and the last "
                "frame have to be labeled as keyframes.");
    }
    else if (scene->getPoints()->isEmpty())
        QMessageBox::information(this, tr("Error"), "This scene is empty.");
    else {
        rotocore->SolveTrackAndGPLVMSingleFrame(indCurScene);
    }
}

void MainWindow::setKeyframe() {
    view->getCurScene()->checkCloseCurve();
    if (view->getCurScene()->getPoints()->isEmpty())
        QMessageBox::information(this, tr("Error"), "The new labeled keyframe is empty.");
    else if(!view->getCurScene()->hasClosedCurve){
        QMessageBox::information(this, tr("Error"), "Please close the curve before setting it as keyframe.");
    }
    else if(!view->checkPtsNumConsKeyframes(flowList->getIndKeyframes())){
        QMessageBox::information(this, tr("Error"), "Number of points are not consistant across keyframes. Suggest to"
                " use Clone A Curve tool in current frame to solve this issue.");
    }
    else{

        view->resetRotationAllScenes();
        EditType typeCurFrame = flowList->getEditTypeCurIcon();
        flowList->labelKeyframeForCurScene();

        QList<int> editingList = flowList->getIndEditing();
        if (!editingList.isEmpty()) {
            QMessageBox::StandardButton reply;
            reply = QMessageBox::question(this, "Warning",
                                          "Some frames may be under editing. The shape in such frames will be overwritten."
                                                  " Please press NO if you are not sure about it.",
                                          QMessageBox::Yes | QMessageBox::No);
            if (reply == QMessageBox::No) {
                flowList->setStatusCurIcon(typeCurFrame);
                return;
            }
        }



        this->updateKeyframe();
        statStack->pushStatus();

        if (!this->runSolving()) {
            QMessageBox::information(this, tr("Error"), "The suggestion is not available because both the first frame"
                    " and the last frame are not keyframes.");
            flowList->setStatusCurIcon(typeCurFrame);
            return;
        }
        statStack->pushStatus();
    }

    //    qDebug() << "number of keyframes: " << keyframes->count();
    //    NodeList nodes = (*rotoCurve)[indScene];
    //    qDebug() << "first point in this keyframe, x: " << (*nodes).first()->getPosition().x() << ", y: " << (*nodes).first()->getPosition().y();
}

void MainWindow::resetKeyframe() {
    EditType typeCurFrame = flowList->getEditTypeCurIcon();
    if (typeCurFrame != KEYFRAME) {
        QMessageBox::information(this, tr("Error"), "The current frame is not a keyframe.");
        return;
    }

    QList<int> indKeys = flowList->getIndKeyframes();
    int indCurFrame = flowList->getindCurScene();
    if (indKeys.size() == 1 && indKeys.contains(indCurFrame)) {
        QMessageBox::information(this, tr("Error"), "This is the only keyframe which cannot be removed.");
        return;
    }

    flowList->setStatusCurIcon(NOEDIT);
    view->resetRotationAllScenes();

    QList<int> editingList = flowList->getIndEditing();
    if (!editingList.isEmpty()) {
        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(this, "Warning",
                                      "Some frames may be under editing. The shape in such frames will be overwritten."
                                              " Please press NO if you are not sure about it.",
                                      QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::No) {
            flowList->setStatusCurIcon(KEYFRAME);
            return;
        }
    }

    this->updateKeyframe();
    statStack->pushStatus();

    if (!this->runSolving()) {
        QMessageBox::information(this, tr("Error"),
                                 "If the current keyframe is removed, the suggestion is not available "
                                         "because both the first frame and the last frame are not keyframes.");
        flowList->setStatusCurIcon(KEYFRAME);
        return;
    }
    statStack->pushStatus();
}

bool MainWindow::runSolving() {
    QList<int> indKeys = flowList->getIndKeyframes();
    int indLastFrame = flowList->getNumScene() - 1;
    if (indKeys.contains(0) && indKeys.contains(indLastFrame)) {
        qDebug() << "Performing both side tracking.";
        TrackerData3 *fwdTrack = new TrackerData3();
        TrackerData3 *bckTrack = new TrackerData3();
        if (!rotocore->track(fwdTrack, FORWARD, solverType)) qDebug() << "Both sides tracker bad!";
        if (!rotocore->track(bckTrack, BACKWARD, solverType)) qDebug() << "Both sides tracker bad!";
        qDebug() << "start to convert data.";
        std::vector<PlanarTrackingData> track;
        track.push_back(rotocore->QtTrackerDataToPlanarTrackingData(fwdTrack, true));
        track.push_back(rotocore->QtTrackerDataToPlanarTrackingData(bckTrack, true));
        qDebug() << "start to build solver.";
        rotocore->SolveTrackAndGPLVM(track, solverType);
    } else if (indKeys.contains(0) && !indKeys.contains(indLastFrame)) {
        qDebug() << "Performing forward tracking.";
        TrackerData3 *trackRes = new TrackerData3();
        if (!rotocore->track(trackRes, FORWARD, solverType)) qDebug() << "Forward tracker bad!";
        else rotocore->updateTrackingResults(trackRes);
    } else if (!indKeys.contains(0) && indKeys.contains(indLastFrame)) {
        qDebug() << "Performing backward tracking.";
        TrackerData3 *trackRes = new TrackerData3();
        if (!rotocore->track(trackRes, BACKWARD, solverType)) qDebug() << "Backward tracker bad!";
        else rotocore->updateTrackingResults(trackRes);
    } else return false;
    return true;
}

void MainWindow::undo() {

    if(designViewPolyAction->isChecked()){
        designViewPolyAction->setChecked(false);
        designViewPolyAction->toggled(false);
    }

    if(designViewRotateAction->isChecked()){
        designViewRotateAction->setChecked(false);
        designViewSelectAction->setChecked(true);
        designViewSelectAction->toggled(true);
    }

    statStack->popStatus();
    reCheckDesignActions();
}

void MainWindow::about() {
    QMessageBox::information(this, tr("About"), "Roto++  Ver. " + QString::number(Roto_VERSION_MAJOR) + "." +
            QString::number(Roto_VERSION_MINOR) + "." + QString::number(Roto_VERSION_PATCH) + "      ");
}

void MainWindow::runActiveContour() {
    int indScene = flowList->getindCurScene();
    if (indScene > -1) {
        rotocore->runActiveContour(indScene);
    }
}

void MainWindow::resetDesignActions() {
    designViewSelectAction->setChecked(false);
    designViewCreateAction->setChecked(false);
    designViewMoveAction->setChecked(false);
    designViewDragAction->setChecked(false);
    designViewSolverDragAction->setChecked(false);
    designViewPolyAction->setChecked(false);
    designViewDeleteAction->setChecked(false);
    designViewModifyAction->setChecked(false);
    designViewRotateAction->setChecked(false);
    designViewZoomAction->setChecked(false);
    designViewShowPtsAction->setChecked(true);
}

void MainWindow::reCheckDesignActions() {
    if (designViewSelectAction->isChecked()) designViewSelectAction->toggled(true);
    else if (designViewCreateAction->isChecked()) designViewCreateAction->toggled(true);
    else if (designViewMoveAction->isChecked()) designViewMoveAction->toggled(true);
    else if (designViewDragAction->isChecked()) designViewDragAction->toggled(true);
    else if (designViewSolverDragAction->isChecked()) designViewSolverDragAction->toggled(true);
    else if (designViewDeleteAction->isChecked()) designViewDeleteAction->toggled(true);
    else if (designViewModifyAction->isChecked()) designViewModifyAction->toggled(true);
    if (designViewZoomAction->isChecked()) designViewZoomAction->toggled(true);
    if (designViewPolyAction->isChecked()) designViewPolyAction->toggled(true);
    if (designViewShowPtsAction->isChecked()){
//        designViewShowPtsAction->setChecked(true);
        designViewShowPtsAction->toggled(true);
    }
    else if(!designViewShowPtsAction->isChecked()){
//        designViewShowPtsAction->setChecked(false);
        designViewShowPtsAction->toggled(false);
    }
    designViewRotateAction->setChecked(false);
}

/*************************
 *        creation       *
 *                       *
 ************************/

void MainWindow::createEntrerWindow() {

    //    setWindowTitle(tr("Roto++"));
    entrerWidget = new QWidget();
    //    entrerWidget->setAutoFillBackground(true);

    buttNewProj = new QPushButton(tr("New Project"));
    buttOpenProj = new QPushButton(tr("Open Project"));

    QLabel *entrerLabel = new QLabel(tr("Welcome to Roto++"));
    entrerLabel->setAlignment(Qt::AlignCenter);

    QHBoxLayout *entrerButtonLayout = new QHBoxLayout();
    QVBoxLayout *entrerLayout = new QVBoxLayout();

    //    entrerButtonLayout->addStretch();
    entrerButtonLayout->addStretch();
    entrerButtonLayout->addWidget(buttNewProj);
    entrerButtonLayout->addStretch();
    entrerButtonLayout->addWidget(buttOpenProj);
    entrerButtonLayout->addStretch();

    entrerLayout->addStretch();
    entrerLayout->addWidget(entrerLabel);
    entrerLayout->addStretch();
    entrerLayout->addLayout(entrerButtonLayout);
    entrerLayout->addStretch();

    QObject::connect(buttNewProj, SIGNAL(clicked()), this, SLOT(createProject()));
    QObject::connect(buttOpenProj, SIGNAL(clicked()), this, SLOT(openProject()));

    entrerWidget->setLayout(entrerLayout);
    setCentralWidget(entrerWidget);
    entrerWidget->show();
}

void MainWindow::reloadImgFrameFlow() {
    this->createFrameFlow(imgs);
    if (!flowList->getIcons().isEmpty()) {
        flowList->getIcons().first()->pushImgIconClicked(0);
        flowList->setCurrentRow(0);
    }
}

void MainWindow::createFrameFlow(QList<QImage *>) {

    /**** image list widget ****/

    flowList->initFrameFlow(imgs);
    flowList->setSceneConnection(view);
//    flowList->setIconConnection();
//    this->linkDesignActionsFromImgIcons(flowList->getIcons());
    emit loadImgDone(imgs);

    mainLayout->addWidget(flowList);
    designToolBar->setEnabled(true);
    rotoToolBar->setEnabled(true);

    qDebug() << "createFrameFlow is done.";
    //    playToolBar->setEnabled(true);
}

void MainWindow::createLayouts() {
    stackedLayout = new QStackedLayout();
    stackedLayout->setStackingMode(QStackedLayout::StackAll);
//    qDebug() << "check_1.5.4.1";
    stackedLayout->addWidget(view);
    stackedLayout->addWidget(snipView);
//    qDebug() << "check_1.5.4.2";
    view->show();
    snipView->show();
//    qDebug() << "check_1.5.4.3";
    mainLayout = new QVBoxLayout();
//    qDebug() << "check_1.5.4.4";
    mainLayout->addLayout(stackedLayout);
}

void MainWindow::createAction() {
    /**** menu Action ****/

    newAction = new QAction(tr("New Project"), this);
    newAction->setShortcut(tr("Ctrl+N"));
    connect(newAction, SIGNAL(triggered()), this, SLOT(newProject()));

    openAction = new QAction(tr("&Open"), this);
    openAction->setShortcut(tr("Ctrl+O"));
    connect(openAction, SIGNAL(triggered()), this, SLOT(loadProject()));

    saveAction = new QAction(tr("Save"), this);
    saveAction->setShortcut(tr("Ctrl+S"));
    connect(saveAction, SIGNAL(triggered()), this, SLOT(saveProject()));

    saveAsAction = new QAction(tr("Save As"), this);
    saveAsAction->setShortcut(tr("Ctrl+Shift+S"));
    connect(saveAsAction, SIGNAL(triggered()), this, SLOT(saveAsProject()));


    importAction = new QAction(tr("Import Points"), this);
    importAction->setShortcut(tr("Ctrl+I"));
    importAction->setEnabled(false);
    connect(importAction, SIGNAL(triggered()), this, SLOT(importProject()));

    importNukeAction = new QAction(tr("Import Nuke Project"), this);
    importNukeAction->setShortcut(tr("Ctrl+Shift+I"));
    connect(importNukeAction, SIGNAL(triggered()), this, SLOT(createNukeProject()));

    exportAction = new QAction(tr("Export Points"), this);
    exportAction->setShortcut(tr("Ctrl+E"));
    connect(exportAction, SIGNAL(triggered()), this, SLOT(exportProject()));

    exportAlphaMapAction = new QAction(tr("Export Alpha Map"), this);
    exportAlphaMapAction->setShortcut(tr("Ctrl+Shift+E"));
    connect(exportAlphaMapAction, SIGNAL(triggered()), this, SLOT(exportAlphaMap()));

    exportCurveMapAction = new QAction(tr("Export Curve Map"), this);
    exportCurveMapAction->setShortcut(tr("Ctrl+Shift+C"));
    connect(exportCurveMapAction, SIGNAL(triggered()), this, SLOT(exportCurveMap()));

    closeApplicationAction = new QAction(tr("Close App"), this);
    closeApplicationAction->setShortcut(QKeySequence::Close);
    connect(closeApplicationAction, SIGNAL(triggered()), this, SLOT(closeApp()));

    aboutAction = new QAction(tr("About"), this);
    connect(aboutAction, SIGNAL(triggered()), this, SLOT(about()));

    /**** designView Action ****/
    designViewActionGroup = new QActionGroup(this);

    designViewSelectAction = new QAction(tr("Selection Mode (Q)"), this);
    designViewSelectAction->setIcon(QIcon(":/images/select.png"));
    designViewSelectAction->setStatusTip(tr("Selection Mode (Q)"));
    designViewSelectAction->setShortcut(tr("Q"));
    designViewSelectAction->setCheckable(true);

    designViewCreateAction = new QAction(tr("Bezier (V)"), this);
    designViewCreateAction->setIcon(QIcon(":/images/add.png"));
    designViewCreateAction->setStatusTip(tr("Bezier (V)"));
    designViewCreateAction->setShortcut(tr("V"));
    designViewCreateAction->setCheckable(true);

    designViewMoveAction = new QAction(tr("Move A Point (A)"), this);
    designViewMoveAction->setIcon(QIcon(":/images/move.png"));
    designViewMoveAction->setStatusTip(tr("Move A Point (A)"));
    designViewMoveAction->setShortcut(tr("A"));
    designViewMoveAction->setCheckable(true);
    designViewMoveAction->setEnabled(false);

    designViewDragAction = new QAction(tr("Drag Multiple Points or Whole Shape (T)"), this);
    designViewDragAction->setIcon(QIcon(":/images/drag.png"));
    designViewDragAction->setStatusTip(tr("Drag Multiple Points or Whole Shape (T)"));
    designViewDragAction->setShortcut(tr("T"));
    designViewDragAction->setCheckable(true);

    designViewSolverDragAction = new QAction(tr("Assisted Grab and Move (G)"), this);
    designViewSolverDragAction->setIcon(QIcon(":/images/rotoDrag.png"));
    designViewSolverDragAction->setStatusTip(tr("Assisted Grab and Move (G)"));
    designViewSolverDragAction->setShortcut(tr("G"));
    designViewSolverDragAction->setCheckable(true);


    designViewPolyAction = new QAction(tr("Show Polygon of Shape (M)"), this);
    designViewPolyAction->setIcon(QIcon(":/images/poly.png"));
    designViewPolyAction->setStatusTip(tr("Show Polygon of Shape (M)"));
    designViewPolyAction->setShortcut(tr("M"));
    designViewPolyAction->setCheckable(true);

    designViewShowPtsAction = new QAction(tr("Show the points (Alt+H)"), this);
    designViewShowPtsAction->setIcon(QIcon(":/images/eye.png"));
    designViewShowPtsAction->setStatusTip(tr("Show the points (Alt+H)"));
    designViewShowPtsAction->setShortcut(tr("Alt+H"));
    designViewShowPtsAction->setCheckable(true);
    designViewShowPtsAction->setChecked(true);

    designViewZoomAction = new QAction(tr("Zooming (1,2)"), this);
    designViewZoomAction->setIcon(QIcon(":/images/zoom.png"));
    designViewZoomAction->setStatusTip(tr("Zooming (1,2)"));
//    designViewZoomAction->setShortcut(tr("Z"));
    designViewZoomAction->setCheckable(true);
//    designViewZoomAction->setChecked(true); // zooming is turned on by default.


    designViewDeleteAction = new QAction(tr("Delete A Point (D)"), this);
    designViewDeleteAction->setIcon(QIcon(":/images/delete.png"));
    designViewDeleteAction->setStatusTip(tr("Delete A Point (D)"));
    designViewDeleteAction->setShortcut(tr("D"));
    designViewDeleteAction->setCheckable(true);

    designViewModifyAction = new QAction(tr("Insert A Point (E)"), this);
    designViewModifyAction->setIcon(QIcon(":/images/insert.png"));
    designViewModifyAction->setStatusTip(tr("Insert A Point (E)"));
    designViewModifyAction->setShortcut(tr("E"));
    designViewModifyAction->setCheckable(true);

    designViewClearAction = new QAction(tr("Clean"), this);
    designViewClearAction->setIcon(QIcon(":/images/clear.png"));
    designViewClearAction->setStatusTip(tr("Clean"));

    designViewColorAction = new QAction(tr("Change Color"), this);
    designViewColorAction->setIcon(QIcon(":/images/color.png"));
    designViewColorAction->setStatusTip(tr("Change Color"));
    designViewColorAction->setShortcut(tr("I"));

    designViewImageProcAction = new QAction(tr("Change Brightness/Contrast/Gamma"), this);
    designViewImageProcAction->setIcon(QIcon(":/images/imgProc.png"));
    designViewImageProcAction->setStatusTip(tr("Change Brightness/Contrast/Gamma"));

    //    designViewColorButton = new QPushButton(tr("Color"));
    //    designViewColorButton->setFixedSize(35,24);

    designViewRotateAction = new QAction(tr("Rotation/Scale Mode (R)"), this);
    designViewRotateAction->setIcon(QIcon(":/images/rotate.png"));
    designViewRotateAction->setStatusTip(tr("Rotation/Scale Mode (R)"));
    designViewRotateAction->setShortcut(tr("R"));
    designViewRotateAction->setCheckable(true);

    designViewPlayFwdAction = new QAction(tr("Play Forward (L)"), this);
    designViewPlayFwdAction->setIcon(QIcon(":/images/fwd.png"));
    designViewPlayFwdAction->setStatusTip(tr("Play Forward (L)"));
    designViewPlayFwdAction->setShortcut(tr("L"));
    designViewPlayFwdAction->setCheckable(true);

    designViewPlayBckAction = new QAction(tr("Play Backward (J)"), this);
    designViewPlayBckAction->setIcon(QIcon(":/images/bck.png"));
    designViewPlayBckAction->setStatusTip(tr("Play Backward (J)"));
    designViewPlayBckAction->setShortcut(tr("J"));
    designViewPlayBckAction->setCheckable(true);

    designViewPlayStopAction = new QAction(tr("Stop Playing (K)"), this);
    designViewPlayStopAction->setIcon(QIcon(":/images/stop.png"));
    designViewPlayStopAction->setStatusTip(tr("Stop Playing (K)"));
    designViewPlayStopAction->setShortcut(tr("K"));
//    designViewPlayStopAction->setCheckable(true);

    rotoKeyframeAction = new QAction(tr("Set Keyframe (H)"), this);
    rotoKeyframeAction->setIcon(QIcon(":/images/key.png"));
    rotoKeyframeAction->setStatusTip(tr("Set Keyframe (H)"));
    rotoKeyframeAction->setShortcut(tr("H"));

    rotoUnKeyframeAction = new QAction(tr("Reset Keyframe (U)"), this);
    rotoUnKeyframeAction->setIcon(QIcon(":/images/unkey.png"));
    rotoUnKeyframeAction->setStatusTip(tr("Reset Keyframe (U)"));
    rotoUnKeyframeAction->setShortcut(tr("U"));


    rotoActiveContour = new QAction(tr("Active Contour Tool beta (W)"), this);
    rotoActiveContour->setIcon(QIcon(":/images/activeContour.png"));
    rotoActiveContour->setStatusTip(tr("Active Contour Tool"));
    rotoActiveContour->setShortcut(tr("W"));

    rotoCopyCureve = new QAction(tr("Copy Shape from Closest Keyframe (Ctrl+V)"), this);
    rotoCopyCureve->setIcon(QIcon(":/images/copy.png"));
    rotoCopyCureve->setStatusTip(tr("Copy Shape from Closest Keyframe (Ctrl+V)"));
    rotoCopyCureve->setShortcut(tr("Ctrl+V"));

    designViewPreFrameAction = new QAction(tr("To Previous Frame (Z)"), this);
    designViewPreFrameAction->setIcon(QIcon(":/images/left.png"));
    designViewPreFrameAction->setStatusTip(tr("To Previous Frame (Z)"));
    //    designViewPreFrameAction->setShortcut(Qt::LeftArrow);

    designViewNextFrameAction = new QAction(tr("To Next Frame (X)"), this);
    designViewNextFrameAction->setIcon(QIcon(":/images/right.png"));
    designViewNextFrameAction->setStatusTip(tr("To Next Frame (X)"));
    //    designViewNextFrameAction->setShortcut(QKeySequence(Qt::RightArrow));



    designViewCuspPts = new QAction(tr("Cusp The Points (Shift+Z)"), this);
    designViewCuspPts->setStatusTip(tr("Cusp the points (Shift+Z)"));
    designViewCuspPts->setShortcut(tr("Shift+Z"));

    designViewSmoothPts = new QAction(tr("Smooth the points (Shift+A)"), this);
    designViewSmoothPts->setStatusTip(tr("Smooth the points (Shift+A)"));
    designViewSmoothPts->setShortcut(tr("Shift+A"));


    designViewUndoAction = new QAction(tr("Undo (Ctrl+Z)"), this);
    designViewUndoAction->setIcon(QIcon(":/images/undo.png"));
    designViewUndoAction->setStatusTip(tr("Undo (Ctrl+Z)"));
    designViewUndoAction->setShortcut(tr("Ctrl+Z"));

    designViewActionGroup->addAction(designViewSelectAction);
    designViewActionGroup->addAction(designViewRotateAction);
    designViewActionGroup->addAction(designViewCreateAction);
    designViewActionGroup->addAction(designViewDeleteAction);
    designViewActionGroup->addAction(designViewModifyAction);

    designViewActionGroup->addAction(designViewPlayFwdAction);
    designViewActionGroup->addAction(designViewPlayBckAction);

    designViewActionGroup->addAction(designViewMoveAction);
    designViewActionGroup->addAction(designViewDragAction);
    designViewActionGroup->addAction(designViewSolverDragAction);

    designViewActionGroup->setExclusive(true);

    rotoSolverTypeGroup = new QActionGroup(this);


    rotoSolverTypeTrackerGPLVMNextEdit = new QAction(tr("System Mode One\n Manifold+Tracker"), this);
    rotoSolverTypeTrackerGPLVMNextEdit->setIcon(QIcon(":/images/1.png"));
    rotoSolverTypeTrackerGPLVMNextEdit->setStatusTip(tr("System Mode One\n Manifold+Tracker"));
    rotoSolverTypeTrackerGPLVMNextEdit->setCheckable(true);
    rotoSolverTypeTrackerGPLVMNextEdit->setChecked(true);

    rotoSolverTypeTracker = new QAction(tr("System Mode Two\n Plannar Tracker"), this);
    rotoSolverTypeTracker->setIcon(QIcon(":/images/2.png"));
    rotoSolverTypeTracker->setStatusTip(tr("System Mode Two\n Plannar Tracker"));
    rotoSolverTypeTracker->setCheckable(true);

    rotoSolverTypeLinearInterp = new QAction(tr("System Mode Three\n Linear Interpolation"), this);
    rotoSolverTypeLinearInterp->setIcon(QIcon(":/images/3.png"));
    rotoSolverTypeLinearInterp->setStatusTip(tr("System Mode Three\n Linear Interpolation"));
    rotoSolverTypeLinearInterp->setCheckable(true);

    rotoSolverTypeGroup->addAction(rotoSolverTypeTrackerGPLVMNextEdit);
//    rotoSolverTypeGroup->addAction(rotoSolverTypeTrackerGPLVM);
    rotoSolverTypeGroup->addAction(rotoSolverTypeTracker);
    rotoSolverTypeGroup->addAction(rotoSolverTypeLinearInterp);
//    rotoSolverTypeGroup->setVisible(true);

#ifdef ENABLE_SOLVER_MODE
    rotoSolverTypeGroup->setEnabled(true);
#else
    rotoSolverTypeGroup->setEnabled(false);
#endif

    settingFilledCurveMap = new QAction(tr("Filled Curve Map"), this);
    settingFilledCurveMap->setStatusTip(tr("Filled Curve Map"));
    settingFilledCurveMap->setCheckable(true);

    /**** menu click Action ****/

    addAction(designViewPreFrameAction);
    addAction(designViewNextFrameAction);
    addAction(designViewUndoAction);
    setContextMenuPolicy(Qt::ActionsContextMenu);
}

void MainWindow::createToolBar() {
    /**** design ToolBar ****/

    designToolBar = addToolBar(tr("Design Tools"));
    designToolBar->addAction(designViewSelectAction);
    designToolBar->addAction(designViewRotateAction);
    designToolBar->addAction(designViewModifyAction);
    designToolBar->addAction(designViewDeleteAction);
    designToolBar->addAction(designViewCreateAction);
//    designToolBar->addAction(designViewMoveAction);
//    designToolBar->addAction(designViewDragAction);
//    designToolBar->addAction(designViewSolverDragAction);

    designToolBar->addSeparator();

    designToolBar->addAction(designViewPlayFwdAction);
    designToolBar->addAction(designViewPlayStopAction);
    designToolBar->addAction(designViewPlayBckAction);

    designToolBar->addSeparator();

    designToolBar->addAction(designViewColorAction);
    designToolBar->addAction(designViewImageProcAction);
//    designToolBar->addAction(designViewShowPtsAction);
    designToolBar->addAction(designViewPolyAction);
    designToolBar->addAction(designViewUndoAction);
    designToolBar->addAction(designViewZoomAction);
    designToolBar->addAction(designViewPreFrameAction);
    designToolBar->addAction(designViewNextFrameAction);
    designToolBar->addAction(designViewClearAction);
    //    designToolBar->addWidget(designViewColorButton);

    this->addToolBar(Qt::LeftToolBarArea, designToolBar);
    designToolBar->setEnabled(false);
    designToolBar->hide();

    /**** Roto ToolBar ****/

    rotoToolBar = addToolBar(tr("Roto Tools"));
    rotoToolBar->addAction(rotoKeyframeAction);
    rotoToolBar->addAction(rotoUnKeyframeAction);
    rotoToolBar->addAction(rotoCopyCureve);
    rotoToolBar->addAction(designViewSolverDragAction);
    rotoToolBar->addAction(rotoActiveContour);

    rotoToolBar->addSeparator();

    rotoToolBar->addAction(rotoSolverTypeTrackerGPLVMNextEdit);
//    rotoToolBar->addAction(rotoSolverTypeTrackerGPLVM);
    rotoToolBar->addAction(rotoSolverTypeTracker);
    rotoToolBar->addAction(rotoSolverTypeLinearInterp);

    this->addToolBar(Qt::RightToolBarArea, rotoToolBar);
    rotoToolBar->setEnabled(false);
    rotoToolBar->hide();

    //    qDebug() << "icon size of toolbar, x:" << designToolBar->iconSize().width() << " , y:" << designToolBar->iconSize().height();
}

void MainWindow::createMenus() {
    fileMenu = menuBar()->addMenu(tr("File"));
    fileMenu->addAction(newAction);
    fileMenu->addAction(openAction);
    fileMenu->addSeparator();
    fileMenu->addAction(saveAction);
    fileMenu->addAction(saveAsAction);
    fileMenu->addAction(importAction);
    fileMenu->addAction(importNukeAction);
    fileMenu->addAction(exportAction);
    fileMenu->addAction(exportAlphaMapAction);
    fileMenu->addAction(exportCurveMapAction);
    fileMenu->addSeparator();
    fileMenu->addAction(closeApplicationAction);

    editMenu = menuBar()->addMenu(tr("Edit"));
    editMenu->addAction(designViewShowPtsAction);
    editMenu->addSeparator();
    editMenu->addAction(designViewSmoothPts);
    editMenu->addAction(designViewCuspPts);
    editMenu->addSeparator();
    editMenu->addAction(designViewUndoAction);
    editMenu->addAction(designViewPreFrameAction);
    editMenu->addAction(designViewNextFrameAction);

    settingMenu = menuBar()->addMenu(tr("Setting"));
    settingMenu -> addAction(settingFilledCurveMap);

    aboutMenu = menuBar()->addMenu(tr("About"));
    aboutMenu->addAction(aboutAction);
    this->setContextMenuPolicy(Qt::NoContextMenu);
}


/*************************
 *       functions       *
 *                       *
 ************************/

void MainWindow::smoothPts(){
    view->getCurScene()->smoothPoints();
    statStack->pushStatus();
}

void MainWindow::cuspPts(){
    view->getCurScene()->cuspPoints();
    statStack->pushStatus();
}

void MainWindow::backToSelection(){
    if(!isProjActive) return;
    designViewSelectAction->toggled(true);
    designViewSelectAction->setChecked(true);
}

void MainWindow::backToShowPts(){
    if(!isProjActive) return;
    designViewShowPtsAction->toggled(true);
    designViewShowPtsAction->setChecked(true);
}

void MainWindow::jumpToClosestKeyframe(int indScene, bool fwd){
    QList<int> keys = flowList->getIndKeyframes();
    int indLast = flowList->getIcons().size()-1;
    if(keys.isEmpty()){
        if(fwd && indLast > -1 && indLast != flowList->getindCurScene()){
            flowList->getIcons().at(indLast)->pushImgIconClicked(indLast);
            flowList->setCurrentRow(indLast);
        }else if(!fwd && flowList->getindCurScene() != 0){
            flowList->getIcons().at(0)->pushImgIconClicked(0);
            flowList->setCurrentRow(0);
        }
    }
    int ind = -1;
    if(fwd){
        ind = rotocore->getIndClosestKeyframe(indScene,fwd);
        if(ind < 0 && ind != flowList->getindCurScene()){
            flowList->getIcons().at(indLast)->pushImgIconClicked(indLast);
            flowList->setCurrentRow(indLast);
        }else if(ind != flowList->getindCurScene()){
            flowList->getIcons().at(ind)->pushImgIconClicked(ind);
            flowList->setCurrentRow(ind);
        }
    }else{
        ind = rotocore->getIndClosestKeyframe(indScene,fwd);
        if(ind < 0 && ind != flowList->getindCurScene()){
            flowList->getIcons().at(0)->pushImgIconClicked(0);
            flowList->setCurrentRow(0);
        }else if(ind != flowList->getindCurScene()){
            flowList->getIcons().at(ind)->pushImgIconClicked(ind);
            flowList->setCurrentRow(ind);
        }
    }
}

void MainWindow::updateSolverType(SolverType type){
    if(type == TRACKER_GPLVM_NEXTEDIT) {
        setRotoSolverTypeTrackerGPLVMNextEdit(true);
        rotoSolverTypeTrackerGPLVMNextEdit->setChecked(true);
    }
//    else if (type == TRACKER_GPLVM) {
//        setRotoSolverTypeTrackerGPLVM(true);
//        rotoSolverTypeTrackerGPLVM->setChecked(true);
//    }
    else if (type == TRACKER) {
        setRotoSolverTypeTracker(true);
        rotoSolverTypeTracker->setChecked(true);
    }
    else if (type == LINEAR_INTERP) {
        setRotoSolverTypeLinearInterp(true);
        rotoSolverTypeLinearInterp->setChecked(true);
    }
}

void MainWindow::setRotoSolverTypeTrackerGPLVMNextEdit(bool checked) {
    if (checked) {
        solverType = TRACKER_GPLVM_NEXTEDIT;
        designViewSolverDragAction->setEnabled(true);
//        statStack->pushStatus();
    }
//    qDebug() << "TRACKER_GPLVM_NEXTEDIT is selected.";
}

void MainWindow::setRotoSolverTypeTrackerGPLVM(bool checked) {
    if (checked) {
        solverType = TRACKER_GPLVM;
        designViewSolverDragAction->setEnabled(true);
//        statStack->pushStatus();
    }
//    qDebug() << "TRACKER_GPLVM is selected.";
}

void MainWindow::setRotoSolverTypeTracker(bool checked) {
    if (checked) {
        solverType = TRACKER;
        designViewSolverDragAction->setEnabled(false);
//        statStack->pushStatus();
    }
//    qDebug() << "TRACKER is selected.";
}

void MainWindow::setRotoSolverTypeLinearInterp(bool checked) {
    if (checked) {
        solverType = LINEAR_INTERP;
        designViewSolverDragAction->setEnabled(false);
//        statStack->pushStatus();
    }
//    qDebug() << "LINEAR_INTERP is selected.";
}

void MainWindow::setIsFilledCurveMap(bool checked){
    isFiled_curveMap = checked;
}


void MainWindow::copyCurveFromClosetKeyframe() {
    QList<int> indKeys = flowList->getIndKeyframes();
    int indCurFrame = flowList->getindCurScene();
    if (indKeys.isEmpty()) QMessageBox::information(this, tr("Error"), "No keyframe is labelled.");
    else if (indKeys.contains(indCurFrame)) QMessageBox::information(this, tr("Error"), "Current frame is a keyframe");
    else {
        int ind = rotocore->getIndClosestKeyframe(indCurFrame);
        if (ind > -1 && ind != indCurFrame) {
            view->copyNodeListFromTo(ind, indCurFrame);
            view->getCurScene()->checkCloseCurve();
            this->reCheckDesignActions();
            view->getCurScene()->pushEditing();
            statStack->pushStatus();
        }
    }
}

void MainWindow::openColorDialog() {
    QColorDialog *colorDialog = new QColorDialog(this);
    if (QDialog::Accepted == colorDialog->exec()) {
        view->setCurveColor(colorDialog->currentColor());
    }
}

void MainWindow::changeImageProc(){
    if(isProjActive){
        ImageProcDialog *imPro = new ImageProcDialog(view);
        imPro->exec();
    }
}

void MainWindow::loadProject() {
    LoadProjDialog *loadDiaglog = new LoadProjDialog();
    int operation = loadDiaglog->exec();
    if (operation == QDialog::Accepted
        && !loadDiaglog->getProjFile().isNull()
        && !loadDiaglog->getNameProj().isNull()
        && !loadDiaglog->getNameCurve().isNull()
        && !loadDiaglog->getNFrame().isNull()
        && !loadDiaglog->getNKeyframe().isNull()) {

        if (rotoDataIO->loadRoto(loadDiaglog->getProjFile())) {

            this->projectName = loadDiaglog->getNameProj();
            this->curveName = loadDiaglog->getNameCurve();

            QString tmpPath = QFileInfo(loadDiaglog->getProjFile()).dir().path();
            tmpPath.chop(projectName.size());
            this->projectPath = tmpPath;

            qDebug() << "path of project file: " << projectPath;
            qDebug() << "Project name: " << projectName;
            qDebug() << "Curve name: " << curveName;

            this->updateKeyframe();
            view->checkClosedCurveAllScenes();
            statStack->pushStatus();
            saved = true;
            isProjActive = true;
        } else {
            QMessageBox::information(this, tr("Error"), "Unproper project file loaded.");
            this->loadProject();
        }
    }
}

bool MainWindow::newProject() {
    if (isProjActive && saved) {
        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(this, "Warning",
                                      "Are you saving the current project '" + projectName
                                      + "' and curve '" + curveName
                                      + "' to " + projectPath,
                                      QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::Yes)
            rotoDataIO->saveRoto(projectName, curveName, projectPath, solverType, false);
    }

    CreateProjDialog *newDial = new CreateProjDialog();
    int operation = newDial->exec();
    if (operation == QDialog::Accepted
        && !newDial->getFiles().isNull()
        && !newDial->getName().isNull()
        && !newDial->getCurveName().isNull()
        && !newDial->getProjPath().isNull()) {
        qDebug() << "new project";

        if (rotoDataIO->checkRotoFileExist(newDial->getName(), newDial->getCurveName(), newDial->getProjPath())) {
            QMessageBox::StandardButton reply;
            reply = QMessageBox::question(this, "File Exists",
                                          "The project file is existed. Do you want to overwrite it?",
                                          QMessageBox::Yes | QMessageBox::No);
            if (reply == QMessageBox::No) {
                this->newProject();
                return true;
            }
        }

        workingDir = new QTemporaryDir();
        this->setProjectName(newDial->getName());
        QProcess *process = new QProcess(this);
        process->setWorkingDirectory(workingDir->path());

        if (newDial->hasSrcVideo() && !newDial->hasSrcImgs()) {
            frequenceFPS = newDial->getFPSFrequence();
            qDebug() << "Frequence " << frequenceFPS;
            QString cmd = "ffmpeg -i " + newDial->getFiles() + " -r " + QString::number(frequenceFPS) + " -f image2 " +
                          workingDir->path() + "/image_" + this->getProjectName() + "%_04d.jpg";
            qDebug() << cmd;
            process->execute(cmd);
            qDebug() << "External ffmpeg command ends..." << endl;
            process->waitForFinished();
        } else if (!newDial->hasSrcVideo() && newDial->hasSrcImgs()) {
            QDir dirImgs(newDial->getFiles());
            dirImgs.setNameFilters(QStringList() << "*.png" << "*.jpg" << "*.bmp" << "*.tiff");
            QStringList fileList = dirImgs.entryList(QDir::Files | QDir::NoDotAndDotDot);

            QProgressDialog *bar = new QProgressDialog(this);
            bar->setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
            bar->setLabelText("Loading Images...");
            bar->setRange(0, fileList.size());
            bar->setModal(true);
            bar->setCancelButton(0);
            bar->setValue(0);
            bar->show();

            for (int iImg = 0; iImg < fileList.size(); iImg++) {
                QImage *im = new QImage(dirImgs.path() + "/" + fileList[iImg]);
                QString number = QString("%1").arg(iImg + 1, 4, 10, QChar('0'));
                im->save(workingDir->path() + "/image_" + getProjectName() + "_" + number + ".jpg");
                qDebug() << "Preparing image: " << iImg;
                bar->setValue(iImg);
            }
            qDebug() << dirImgs.path().toUtf8().constData();
            bar->close();
        }

        if (!imgs.isEmpty()) imgs.clear();
        //        if(!matImgs.isEmpty()) matImgs.clear();
        qDebug() << "Dir path is " << workingDir->path();
        QDir *dir = new QDir(workingDir->path());
        dir->setNameFilters(QStringList() << "*.png" << "*.jpg" << "*.bmp" << "*.tiff");
        QStringList fileList = dir->entryList(QDir::Files | QDir::NoDotAndDotDot);

        QProgressDialog *bar = new QProgressDialog(this);
        bar->setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
        bar->setLabelText("Preparing Project...");
        bar->setRange(0, fileList.size());
        bar->setModal(true);
        bar->setCancelButton(0);
        bar->setValue(0);

        for (int iImg = 0; iImg < fileList.size(); iImg++) {
            QString number = QString("%1").arg(iImg + 1, 4, 10, QChar('0'));
            QImage *img = new QImage(dir->path() + "/image_" + getProjectName() + "_" + number + ".jpg");
            imgs.append(img);
            bar->setValue(iImg);
        }

        this->curveName = newDial->getCurveName();
        this->projectPath = newDial->getProjPath();
        this->projectName = newDial->getName();

        QDir alphaMapPath = rotoDataIO->getAlphaMapPath(projectPath + "/" + projectName + "/*");
//        qDebug() << alphaMapPath;
        if(alphaMapPath.exists())
            rotoDataIO->deleteAlphaMapFiles(alphaMapPath.path(),projectName,curveName);

        rotoDataIO->saveRoto(projectName, curveName, projectPath, solverType, false);
        this->createFrameFlow(imgs);
        isProjActive = true;
        saved = true;
        if (!flowList->getIcons().isEmpty()) {
            flowList->getIcons().first()->pushImgIconClicked(0);
            flowList->setCurrentRow(0);
        }
        bar->close();
        return true;
    }
    else if (operation == QDialog::Accepted
             && (newDial->getFiles().isNull()
                 || newDial->getName().isNull()
                 || newDial->getCurveName().isNull()
                 || newDial->getProjPath().isNull())) {
        QMessageBox::information(this, tr("Error"), "Names and Paths are required to fill.");
        this->newProject();
        return true;
    }
    return true;
}

void MainWindow::importProject() {

}

bool MainWindow::importNukeProject() {

    if (isProjActive && saved) {
        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(this, "Warning",
                                      "Are you saving the current project '" + projectName
                                      + "' and curve '" + curveName
                                      + "' to " + projectPath,
                                      QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::Yes)
            rotoDataIO->saveRoto(projectName, curveName, projectPath, solverType, false);
    }

    LoadNukeProjDialog *newDial = new LoadNukeProjDialog();
    int operation = newDial->exec();
    if (operation == QDialog::Accepted
        && !newDial->getFiles().isNull()
        && !newDial->getName().isNull()
        && !newDial->getCurveName().isNull()
        && !newDial->getProjPath().isNull()
        && !newDial->getNukeFileName().isNull()) {
        qDebug() << "new project";

        if (rotoDataIO->checkRotoFileExist(newDial->getName(), newDial->getCurveName(), newDial->getProjPath())) {
            QMessageBox::StandardButton reply;
            reply = QMessageBox::question(this, "File Exists",
                                          "The project file is existed. Do you want to overwrite it?",
                                          QMessageBox::Yes | QMessageBox::No);
            if (reply == QMessageBox::No) {
                this->importNukeProject();
                rotoDataIO->loadNukeExport(newDial->getNukeFileName());
                return true;
            }
        }

        workingDir = new QTemporaryDir();
        this->setProjectName(newDial->getName());
        QProcess *process = new QProcess(this);
        process->setWorkingDirectory(workingDir->path());

        if (!newDial->hasSrcVideo() && newDial->hasSrcImgs()) {
            QDir dirImgs(newDial->getFiles());
            dirImgs.setNameFilters(QStringList() << "*.png" << "*.jpg" << "*.bmp" << "*.tiff");
            QStringList fileList = dirImgs.entryList(QDir::Files | QDir::NoDotAndDotDot);

            QProgressDialog *bar = new QProgressDialog(this);
            bar->setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
            bar->setLabelText("Loading Images...");
            bar->setRange(0, fileList.size());
            bar->setModal(true);
            bar->setCancelButton(0);
            bar->setValue(0);
            bar->show();

            for (int iImg = 0; iImg < fileList.size(); iImg++) {
                QImage *im = new QImage(dirImgs.path() + "/" + fileList[iImg]);
                QString number = QString("%1").arg(iImg + 1, 4, 10, QChar('0'));
                im->save(workingDir->path() + "/image_" + getProjectName() + "_" + number + ".jpg");
                qDebug() << "Preparing image: " << iImg;
                bar->setValue(iImg);
            }
            qDebug() << dirImgs.path().toUtf8().constData();
            bar->close();
        }

        if (!imgs.isEmpty()) imgs.clear();
        //        if(!matImgs.isEmpty()) matImgs.clear();
        qDebug() << "Dir path is " << workingDir->path();
        QDir *dir = new QDir(workingDir->path());
        dir->setNameFilters(QStringList() << "*.png" << "*.jpg" << "*.bmp" << "*.tiff");
        QStringList fileList = dir->entryList(QDir::Files | QDir::NoDotAndDotDot);

        QProgressDialog *bar = new QProgressDialog(this);
        bar->setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
        bar->setLabelText("Preparing Project...");
        bar->setRange(0, fileList.size());
        bar->setModal(true);
        bar->setCancelButton(0);
        bar->setValue(0);

        for (int iImg = 0; iImg < fileList.size(); iImg++) {
            QString number = QString("%1").arg(iImg + 1, 4, 10, QChar('0'));
            QImage *img = new QImage(dir->path() + "/image_" + getProjectName() + "_" + number + ".jpg");
            imgs.append(img);
            bar->setValue(iImg);
        }

        this->curveName = newDial->getCurveName();
        this->projectPath = newDial->getProjPath();
        this->projectName = newDial->getName();

        QDir alphaMapPath = rotoDataIO->getAlphaMapPath(projectPath + "/" + projectName + "/*");
//        qDebug() << alphaMapPath;
        if(alphaMapPath.exists())
            rotoDataIO->deleteAlphaMapFiles(alphaMapPath.path(),projectName,curveName);

        rotoDataIO->saveRoto(projectName, curveName, projectPath, solverType, false);
        this->createFrameFlow(imgs);
        isProjActive = true;
        saved = true;
        if (!flowList->getIcons().isEmpty()) {
            flowList->getIcons().first()->pushImgIconClicked(0);
            flowList->setCurrentRow(0);
        }
        bar->close();
        rotoDataIO->loadNukeExport(newDial->getNukeFileName());
        return true;
    }
    else if (operation == QDialog::Accepted
             && (newDial->getFiles().isNull()
                 || newDial->getName().isNull()
                 || newDial->getCurveName().isNull()
                 || newDial->getProjPath().isNull()
                 || newDial->getNukeFileName().isNull())) {
        QMessageBox::information(this, tr("Error"), "Names and Paths are required to fill.");
        this->importNukeProject();
        rotoDataIO->loadNukeExport(newDial->getNukeFileName());
        return true;
    }
    return true;
}

bool MainWindow::exportAlphaMap(){
    if (isProjActive && saved){
        rotoDataIO->saveAlphaMap(projectName, curveName, projectPath,true);
        QMessageBox::information(this, tr("Information"), "The Alpha Maps have been exported successfully.");
        return true;
    }
    return false;
}

bool MainWindow::exportCurveMap(){
    if (isProjActive && saved){
        rotoDataIO->saveCurveMap(projectName, curveName, projectPath);
        QMessageBox::information(this, tr("Information"), "The Curve Maps have been exported successfully.");
        return true;
    }
    return false;
}

bool MainWindow::exportProject() {
    if (isProjActive && saved){
        ExportProjDialog *expDial = new ExportProjDialog(curveName+"_pts",projectPath+projectName);
        int operation = expDial->exec();
        if (operation == QDialog::Accepted
                && !expDial->getExpName().isNull()
                && !expDial->getExpPath().isNull()){

            if (rotoDataIO->checkRotoExportFileExist(expDial->getExpName(),expDial->getExpPath())){
                QMessageBox::StandardButton reply;
                reply = QMessageBox::question(this, "File Exists",
                                              "The Export file is existed. Do you want to overwrite it?",
                                              QMessageBox::Yes | QMessageBox::No);
                if (reply == QMessageBox::No) {
                    this->exportProject();
                    return true;
                }
            }

            if(!rotoDataIO->exportRoto(expDial->getExpPath() + "/" + expDial->getExpName() + ".rotoExport", expDial->isTrans())){
                QMessageBox::information(this, tr("Error"), "File exporting fails.");
                this->exportProject();
                return true;
            }

        }
        else if (operation == QDialog::Accepted
                 && (expDial->getExpName().isNull()
                     || expDial->getExpPath().isNull())) {
            QMessageBox::information(this, tr("Error"), "Names and Paths are required to fill.");
            this->exportProject();
            return true;
        }
    } else return false;
    return true;
}

//Just copy the content of tmp folder and all the drawed picture in a defined path
void MainWindow::saveProject() {
    if (isProjActive) {
        if (saved) rotoDataIO->saveRoto(projectName, curveName, projectPath, solverType, false);
        else saveAsProject();
        QMessageBox::information(this, tr("Information"), "The current project has been saved successfully.");
    }
}

void MainWindow::saveAsProject() {
    if (isProjActive) {
        QString tmpPath = projectPath;
        if (tmpPath.isEmpty()) tmpPath = workingDir->path();
        SaveProjDialog *saveasDiaglog = new SaveProjDialog(projectName, curveName, tmpPath);
        int operation = saveasDiaglog->exec();
        if (operation == QDialog::Accepted
            && saveasDiaglog->getNameProj() != NULL
            && saveasDiaglog->getNameCurve() != NULL
            && saveasDiaglog->getProjPath() != NULL) {

            //            qDebug() << "project name: " << saveasDiaglog->getNameProj() << endl
            //                     << "curve name: " << saveasDiaglog->getNameCurve() << endl
            //                     << "workspace path: " << saveasDiaglog->getProjPath();

            projectName = saveasDiaglog->getNameProj();
            curveName = saveasDiaglog->getNameCurve();
            projectPath = saveasDiaglog->getProjPath();

            rotoDataIO->saveRoto(projectName, curveName, projectPath, solverType, false);
            saved = true;
        }
        else if (operation == QDialog::Accepted
                 && (saveasDiaglog->getNameProj().isEmpty()
                     || saveasDiaglog->getNameCurve().isEmpty()
                     || saveasDiaglog->getProjPath().isEmpty())) {
            QMessageBox::information(this, tr("Error"), "Names and Project Path are required to fill.");
            this->saveAsProject();
        }
    }
}

void MainWindow::closeApp() {
    if (isProjActive && saved) {
        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(this, "Warning",
                                      "Are you saving the current project '" + projectName
                                      + "' and curve '" + curveName
                                      + "'\n have been saved to " + projectPath,
                                      QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::Yes)
            rotoDataIO->saveRoto(projectName, curveName, projectPath, solverType, false);
    }
    this->close();
}

void MainWindow::autoSave() {
    if (isProjActive && saved)
        rotoDataIO->autoSave(solverType);
}

void MainWindow::autoExportLog() {
    if (isProjActive && saved)
        rotoDataIO->autoExportLog();
}

void MainWindow::playFwd(bool checked){
    if(checked){
        isPlayingFwd = true;
        isPlayingBck = false;
        timerPlay->start(100);
//        view->hidePointsAllScenes();
    }else isPlayingFwd = false;
}

void MainWindow::jumpToNextFrame(){

}

void MainWindow::jumpToPreFrame(){

}

void MainWindow::playNextFrame(){
    if(!isPlayingFwd || !isProjActive) return;
    int indCurScene = -1;
    int nScene = -1;
    indCurScene = flowList->getindCurScene();
    nScene = imgs.size();
    if(indCurScene<0 || nScene<0) return;
    if(indCurScene==nScene-1){
        flowList->getIcons().at(0)->pushImgIconClicked(0);
        flowList->setCurrentRow(0);
    }
    else {
        flowList->getIcons().at(indCurScene+1)->pushImgIconClicked(indCurScene+1);
        flowList->setCurrentRow(indCurScene+1);
    }
}

void MainWindow::playBck(bool checked){
    if(checked){
        isPlayingFwd = false;
        isPlayingBck = true;
        timerPlay->start(100);
//        view->hidePointsAllScenes();
    }else isPlayingBck = false;
}

void MainWindow::playPreviousFrame(){
    if(!isPlayingBck || !isProjActive) return;
    int indCurScene = -1;
    int nScene = -1;
    indCurScene = flowList->getindCurScene();
    nScene = imgs.size();
    if(indCurScene<0 || nScene<0) return;
    if(indCurScene == 0){
        flowList->getIcons().at(nScene-1)->pushImgIconClicked(nScene-1);
        flowList->setCurrentRow(nScene-1);
    }
    else {
        flowList->getIcons().at(indCurScene-1)->pushImgIconClicked(indCurScene-1);
        flowList->setCurrentRow(indCurScene-1);
    }
}

void MainWindow::stopPlaying(){
    isPlayingFwd = false;
    isPlayingBck = false;
    timerPlay->stop();
//    view->showPointsAllScenes();
    designViewSelectAction->toggled(true);
    designViewSelectAction->setChecked(true);
}

void MainWindow::createProject() {
    newProject();

    entrerWidget->hide();
    setCentralWidget(widget);
    designToolBar->show();
    rotoToolBar->show();
    widget->show();
}

void MainWindow::createNukeProject() {
    importNukeProject();

    entrerWidget->hide();
    setCentralWidget(widget);
    designToolBar->show();
    rotoToolBar->show();
    widget->show();
}

void MainWindow::openProject() {
    loadProject();

    entrerWidget->hide();
    setCentralWidget(widget);
    designToolBar->show();
    rotoToolBar->show();
    widget->show();
}


