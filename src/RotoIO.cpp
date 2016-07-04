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

#include "include/RotoIO.hpp"

RotoIO::RotoIO(DesignView* v,FrameFlow* fl, RotoStatus* rs, ImageList* im, RotoCore* rc)
    :view(v),flowList(fl),statStack(rs),imgs(im),rotocore(rc){
    config = new ConfigIO("RotoConfig.rotoInit");
    QObject::connect(statStack,SIGNAL(pushLog()),this,SLOT(saveLog()));
}

void RotoIO::saveAlphaMap(QString projName, QString curveName, QString projPath, bool isWhiteAlpha){
    QList<DesignScene*>* scenes = view->getScenes();
    if(scenes->isEmpty()) return;

    if(!QDir(projPath+"/"+projName).exists())
        QDir(projPath).mkdir(projName);

    if(!QDir(projPath+"/"+projName+"/alphaMap").exists())
        QDir(projPath+"/"+projName).mkdir("alphaMap");

    QDir imgPath = getAlphaMapPath(projPath + "/" + projName + "/*");
    imgPath.setNameFilters(QStringList() << "alphaMap_" + projName + "_" + curveName + "*.*");
    imgPath.setFilter(QDir::Files);
    foreach(QString dirFile, imgPath.entryList())
        imgPath.remove(dirFile);

    for(int iImg = 0; iImg < scenes->size(); iImg++){
        QString number = QString("%1").arg(iImg+1, 4, 10, QChar('0'));
        scenes->at(iImg)->getAlphaMap(isWhiteAlpha).save(projPath + "/" + projName + "/alphaMap/alphaMap_"+projName+ "_" + curveName + "_" + number +".png");
    }
}

void RotoIO::saveCurveMap(QString projName, QString curveName, QString projPath){
    QList<DesignScene*>* scenes = view->getScenes();
    if(scenes->isEmpty()) return;

    if(!QDir(projPath+"/"+projName).exists())
        QDir(projPath).mkdir(projName);

    if(!QDir(projPath+"/"+projName+"/curveMap").exists())
        QDir(projPath+"/"+projName).mkdir("curveMap");

    QDir imgPath = getAlphaMapPath(projPath + "/" + projName + "/*");
    imgPath.setNameFilters(QStringList() << "curveMap_" + projName + "_" + curveName + "*.*");
    imgPath.setFilter(QDir::Files);
    foreach(QString dirFile, imgPath.entryList())
        imgPath.remove(dirFile);

    config->readConfigs();

    for(int iImg = 0; iImg < scenes->size(); iImg++){
        QString number = QString("%1").arg(iImg+1, 4, 10, QChar('0'));
        scenes->at(iImg)->getCurveMap(config->isFilledCurveMap(),config->isShowPtFilledCurveMap(),config->getLineWidthCurveMap()).save(projPath + "/" + projName + "/curveMap/curveMap_"+projName+ "_" + curveName + "_" + number +".png");
    }
}

void RotoIO::saveRoto(QString projName, QString curveName, QString projPath, SolverType type, bool flagAutoSave){

    if(!QDir(projPath+"/"+projName).exists())
        QDir(projPath).mkdir(projName);

    if(!QDir(projPath+"/"+projName+"/image").exists())
        QDir(projPath+"/"+projName).mkdir("image");

    int nScene = flowList->getNumScene();
    DesignSceneList* sceneList = view->getScenes();
    StatusList states = flowList->getStatusList();
    int nKeyframe = flowList->getIndKeyframes().size();
    int nPoints = view->getNumAllPoints();

    QString savefile;

    if(flagAutoSave) savefile = projPath + "/" + projName + "/" + curveName + ".rotoAutoSave";
    else savefile = projPath + "/" + projName + "/" + curveName + ".roto";

    QFile file(savefile);

    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);
    out << projName << "\n";
    out << curveName << "\n";
    out << QDateTime::currentDateTime().toString() << "\n";
    out << type << " "
        << imgs->size() << " "
        << nKeyframe << " "
        << imgs->first()->size().width() << " "
        << imgs->first()->size().height() << " "
        << nPoints << "\n";

//    qDebug() << "naming saving done!";

    for(int iScene = 0; iScene < nScene; iScene++){
        NodeList* points = (*sceneList)[iScene]->getPoints();
        int indFirstNode = 0;
        int isClosedCurve = 0;
        if(points->size()>0){
            for(int iNode = 0; iNode < points->size();iNode++)
                if((*points)[iNode]->isFirstNode()) indFirstNode = iNode;
            if((*points)[indFirstNode]->hasPreNode) isClosedCurve = 1;
        }

        out << states[iScene] << " " << points->size() << " " << indFirstNode << " " << isClosedCurve << "\n";

        if(points->size()>0){
            for(int iNode = 0; iNode < points->size();iNode++){
                QPointF center = (*points)[iNode]->getPosition();
                QPointF left = (*points)[iNode]->getLef()->getPosition();
                QPointF right = (*points)[iNode]->getRig()->getPosition();
                out << center.x() << " " << center.y() << " "
                    << left.x() << " " << left.y() << " "
                    << right.x() << " " << right.y() << "\n";
                //                qDebug() << center.x() << " " << center.y() << " "
                //                    << left.x() << " " << left.y() << " "
                //                    << right.x() << " " << right.y();
            }
        }
    }

    if(!checkImgFiles(projPath,projName)){
        deleteRotoFiles(projPath + "/" + projName); // clean roto files in the project folder
        qDebug() << "start to save images.";
        QDir imgPath = getImgPath(projPath + "/" + projName + "/*");
        imgPath.setNameFilters(QStringList() << "*.*");
        imgPath.setFilter(QDir::Files);
        foreach(QString dirFile, imgPath.entryList())
            imgPath.remove(dirFile);

        for(int iImg = 0; iImg < imgs->size(); iImg++){
            QString number = QString("%1").arg(iImg+1, 4, 10, QChar('0'));
            (*imgs)[iImg]->save(projPath + "/" + projName + "/image/image_"+projName+ "_" + number +".jpg");
        }
    }

    nameOfProject = projName;
    nameOfCurve = curveName;
    projectPath = projPath;
}

bool RotoIO::loadRoto(QString filePath){

    QFile file(filePath);
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    QTextStream in(&file);
    QString nameProj = in.readLine();
    QString nameCurve = in.readLine();
    QString saveTime = in.readLine();
    QString tmp = in.readLine();
    if(!checkStringList(tmp,6)) return false;
    QStringList frameList = tmp.split(" ");
    int type = frameList.at(0).toInt();
    int nFrame = frameList.at(1).toInt();
    int nKeyframe = frameList.at(2).toInt();
    QSize imgSize(frameList.at(3).toInt(),frameList.at(4).toInt());
    int nPoints = frameList.at(5).toInt();

    //******** load images ********//

    if(!checkImgPath(filePath,nFrame)) return false;
    imgs->clear();
    QDir imgPath = getImgPath(filePath);
    for(int iImg = 0; iImg < nFrame; iImg++){
        QString number = QString("%1").arg(iImg+1, 4, 10, QChar('0'));
        if(!QFile(imgPath.path() + "/image_" + nameProj+ "_" + number +".jpg").exists()) return false;

        imgs->append(new QImage(imgPath.path() + "/image_" + nameProj+ "_" + number +".jpg"));
        qDebug() << "Loading image: " << iImg;
    }

    emit pushImgs();

    int keyframeCheck = 0;

    if(nPoints > 0){

        DesignSceneList* scenes = view->getScenes();

        for(int iFrame = 0; iFrame < nFrame; iFrame++){
            QString tmpInfo = in.readLine();

            if(!checkStringList(tmpInfo,4)) return false;
            QStringList dataList = tmpInfo.split(" ");

            int frameStatus = dataList.at(0).toInt();
            int nPoint = dataList.at(1).toInt();
            int indFirstPt = dataList.at(2).toInt();
            int isclosedCurve = dataList.at(3).toInt();

            if(nPoint == 0) continue;

            NodeList* nodelist = scenes->at(iFrame)->getPoints();
            ImgIconList iconlist = flowList->getIcons();
            scenes->at(iFrame)->cleanView();
            for(int iPt = 0; iPt < nPoint; iPt++){
                QString tmpPt = in.readLine();
                if(!checkStringList(tmpPt,6)) return false;
                QStringList ptList = tmpPt.split(" ");
                QPointF center(ptList.at(0).toFloat(), ptList.at(1).toFloat());
                QPointF left(ptList.at(2).toFloat(), ptList.at(3).toFloat());
                QPointF right(ptList.at(4).toFloat(), ptList.at(5).toFloat());
                SpNode* newPt = new SpNode(center,left,right,scenes->at(iFrame)->getScale());
                if(!nodelist->isEmpty()){
                    nodelist->last()->setNextNode(newPt);
                    newPt->setPreNode(nodelist->last());
                }
                scenes->at(iFrame)->addNode(newPt);
            }

            nodelist->at(indFirstPt)->setFirstNode();

            if(isclosedCurve==1 && indFirstPt==0){
                nodelist->last()->setNextNode(nodelist->first());
                nodelist->first()->setPreNode(nodelist->last());
                nodelist->first()->setFirstNode();
            }

            scenes->at(iFrame)->checkCloseCurve();
            scenes->at(iFrame)->updateScene();

            if(frameStatus==0) iconlist.at(iFrame)->setStatus(EditType::NOEDIT);
            else if (frameStatus==1){
                keyframeCheck++;
                iconlist.at(iFrame)->setStatus(EditType::KEYFRAME);
            }else if (frameStatus==2) iconlist.at(iFrame)->setStatus(EditType::SUGGESTED);
            else iconlist.at(iFrame)->setStatus(EditType::EDITING);

        }

        if(keyframeCheck!=nKeyframe) return false;
    }

    nameOfProject = nameProj;
    nameOfCurve = nameCurve;
    QString tmpPath = QFileInfo(filePath).dir().path();
    tmpPath.chop(nameOfProject.size());
    projectPath = tmpPath;

    emit pushSolverType(static_cast<SolverType>(type));

    return true;
}

bool RotoIO::loadNukeExport(QString filePath){
    QFile file(filePath);
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    QTextStream in(&file);
    QString nukeCurveName = in.readLine();
    QString tmp = in.readLine();
    QStringList frameList = tmp.split(" ");
    frameList.removeAll("");
    int nFrame = frameList.size();
    qDebug() << "image size:" << imgs->size();
    qDebug() << "nuke image size:" << nFrame;
    if(nFrame != imgs->size()) return false;
    QList<QList< float > > tmpList;

    while (!in.atEnd()) {
        QList<float> row;
        QStringList tmp = in.readLine().split(" ");
        tmp.removeAll("");
        for(int iPt = 1; iPt < tmp.size(); iPt++){
            row.append(tmp.at(iPt).toFloat());
//            qDebug() << tmp.at(iPt).toFloat();
        }

        if(row.size() != nFrame) return false;
        tmpList.append(row);
    }

    Eigen::MatrixXd res(tmpList.at(0).size(),tmpList.size());

    for(int iRow = 0; iRow < tmpList.size(); iRow++){
        for(int iCol = 0; iCol < tmpList.at(0).size(); iCol++){
            res(iCol,iRow) = tmpList.at(iRow).at(iCol);
        }
    }

    int imgHeight = imgs->at(0)->size().height();

    for(int iRow = 0; iRow < res.rows(); iRow++){
        for(int iCol = 1; iCol < res.cols(); iCol=iCol+2){
            res(iRow,iCol) = imgHeight - res(iRow,iCol);
        }
    }

    Eigen::MatrixXd regRes(tmpList.at(0).size(),tmpList.size());

    for(int iCol = 0; iCol < res.cols(); iCol=iCol+6) {
        regRes.col(iCol) = res.col(iCol);
        regRes.col(iCol + 1) = res.col(iCol + 1);
        regRes.col(iCol + 2) = res.col(iCol + 4);
        regRes.col(iCol + 3) = res.col(iCol + 5);
        regRes.col(iCol + 4) = res.col(iCol + 2);
        regRes.col(iCol + 5) = res.col(iCol + 3);
    }

    rotocore->updateTrackingResults(regRes);
    flowList->setKeyframeAll();
    qDebug() << "loaded Nuke project, points: " << tmpList.size()/6 << ", frames:" << tmpList.at(0).size();
    return true;
}

void RotoIO::saveLog(){
    if(nameOfProject!=NULL && nameOfCurve!=NULL && projectPath!=NULL && !imgs->isEmpty()){
        if(QDir(projectPath+"/"+nameOfProject).exists()){
            QFile file(projectPath+"/"+nameOfProject+"/"+nameOfCurve+".rotoLog");
            if(!file.exists()){
                file.open(QIODevice::WriteOnly | QIODevice::Text);
                QTextStream out(&file);
                out << nameOfProject << "\n";
                out << nameOfCurve << "\n";
                out << QDateTime::currentDateTime().toString() << "\n";
                out << imgs->size() << " "
                    << imgs->first()->size().width() << " "
                    << imgs->first()->size().height() << "\n";
                file.close();
            }
            file.open(QIODevice::Append | QIODevice::Text);
            QTextStream out(&file);
            out << statStack->getLastTimeStamp() << "\n";
            out << statStack->getLastSolverType();

            StatusList statusList = statStack->getLastEditTypes();
            for(int i = 0; i < statusList.size(); i++)
                out << " " << statusList[i];
            out << "\n";
            out << statStack->getLastIndCurScene();
            FlagList list = statStack->getLastFlags();
            for(int i = 0; i < list.size(); i++)
                out << " " << list[i];
            out << "\n";
            out << statStack->getLastMouseClick().x() << " "
                << statStack->getLastMouseClick().y() << " "
                << statStack->getLastMouseRelease().x() << " "
                << statStack->getLastMouseRelease().y() << "\n";
        }
    }
//    qDebug() << "saveLog is called.";
}

void RotoIO::autoSave(SolverType type){
    if(nameOfProject!=NULL && nameOfCurve!=NULL && projectPath!=NULL){
        saveRoto(nameOfProject, nameOfCurve, projectPath, type, true);
    }
}

void RotoIO::autoExportLog(){
    if(nameOfProject==NULL || nameOfCurve==NULL
            || projectPath==NULL || imgs->isEmpty())
        return;

    Eigen::MatrixXd pts = rotocore->normAllPoints();
    if(pts.rows()==0 || pts.cols()==0) return;

    if(QDir(projectPath+"/"+nameOfProject).exists()){
        QFile file(projectPath+"/"+nameOfProject+"/"+nameOfCurve+".rotoAutoExport");
        if(!file.exists()){
            file.open(QIODevice::WriteOnly | QIODevice::Text);
            QTextStream out(&file);
            out << nameOfProject << "\n";
            out << nameOfCurve << "\n";
            out << QDateTime::currentDateTime().toString() << "\n";
            out << pts.rows() << " " << pts.cols() << " " << 0 << "\n";
            file.close();
        }
        file.open(QIODevice::Append | QIODevice::Text);
        QTextStream out(&file);
        out << QDateTime::currentDateTime().toString() << "\n";
        EigenMatrixToFile(pts,out);
    }

}

bool RotoIO::exportRoto(QString filename, bool isTrans){
    Eigen::MatrixXd pts = rotocore->normAllPoints();
    if(pts.rows()==0 || pts.cols()==0) return false;

    QFile file(filename);
    if(!file.open(QFile::WriteOnly | QFile::Text)) return false;
    QTextStream out(&file);
    out << nameOfProject << "\n";
    out << nameOfCurve << "\n";
    out << pts.rows() << " " << pts.cols() << " " << isTrans << " "
        << view->getScenes()->first()->hasClosedCurve << "\n";
    if(!isTrans) EigenMatrixToFile(pts,out);
    else EigenMatrixToFile(pts.transpose(),out);
    return true;
}

bool RotoIO::checkRotoFileExist(QString projName, QString curveName, QString projPath){
    QString savefile = projPath + "/" + projName + "/" + curveName + ".roto";
    QFile file(savefile);
    return file.exists();
}

bool RotoIO::checkRotoExportFileExist(QString expName, QString projPath){
    QString savefile = projPath + "/" + expName + ".rotoExport";
    QFile file(savefile);
//    qDebug() << savefile;
    return file.exists();
}

bool RotoIO::checkImgPath(QString filePath, int n){
    QDir imgPath = getImgPath(filePath);
    if(!imgPath.exists()) return false;
    imgPath.setNameFilters(QStringList() << "*.png" << "*.jpg" << "*.bmp" << "*.tiff");
    QStringList fileList = imgPath.entryList(QDir::Files|QDir::NoDotAndDotDot);
    if(fileList.size()!=n) return false;
    return true;
}

bool RotoIO::checkImgFiles(QString path, QString projName){
    if(!checkImgPath(path + "/" + projName + "/*", imgs->size())) return false;
    for(int iImg = 0; iImg < imgs->size(); iImg++){
        QString number = QString("%1").arg(iImg+1, 4, 10, QChar('0'));
        if(!QFile(path + "/" + projName + "/image/image_" + projName+ "_" + number + ".jpg").exists()) return false;
    }
    return true;
}

bool RotoIO::checkAlphaMapPath(QString filePath, QString projName, QString curveName, int n){
    QDir imgPath = getAlphaMapPath(filePath);
    if(!imgPath.exists()) return false;
    imgPath.setNameFilters(QStringList() << "alphaMap_" + projName + "_" + curveName + "_*.jpg");
    QStringList fileList = imgPath.entryList(QDir::Files|QDir::NoDotAndDotDot);
    if(fileList.size()!=n) return false;
    return true;
}

bool RotoIO::checkAlphaMapFiles(QString path, QString projName, QString curveName){
    if(!checkAlphaMapPath(path + "/" + projName + "/*", projName, curveName, imgs->size())) return false;
    for(int iImg = 0; iImg < imgs->size(); iImg++){
        QString number = QString("%1").arg(iImg+1, 4, 10, QChar('0'));
        if(!QFile(path + "/" + projName + "/alphaMap/alphaMap_" + projName+ "_" + curveName + "_" + number + ".jpg").exists()) return false;
    }
    return true;
}

QDir RotoIO::getImgPath(QString filePath){
    return QDir(QFileInfo(filePath).dir().path()+"/image");
}

QDir RotoIO::getAlphaMapPath(QString filePath){
    return QDir(QFileInfo(filePath).dir().path()+"/alphaMap");
}

QDir RotoIO::getCurveMapPath(QString filePath){
    return QDir(QFileInfo(filePath).dir().path()+"/curveMap");
}

bool RotoIO::checkStringList(QString tmp,int n){
    if(tmp.isNull()) return false;
    else return checkStringList(tmp.split(" "),n);
}

bool RotoIO::checkStringList(QStringList list, int n){
    if(list.size()!=n) return false;
    else return true;
}

void RotoIO::EigenMatrixToFile(const Eigen::MatrixXd & M, QTextStream& out){
//    QFile file( filename );
//    if(!file.open(QFile::WriteOnly | QFile::Text)) return;
//    QTextStream out(&file);
    for(unsigned int iRow = 0; iRow < M.rows(); iRow++) {
        QStringList row;
        for(unsigned int iCol = 0; iCol < M.cols(); iCol++)
            row << QString::number(M(iRow,iCol));
        out << (row.join(" ") + "\n");
    }
}

void RotoIO::deleteRotoFiles(QString filePath){
//    qDebug() << filePath;
    QDir imgPath(filePath);
    if(!imgPath.exists()) return;
    imgPath.setNameFilters(QStringList() << "*.rotoLog" << "*.rotoAutoSave" << "*.rotoAutoExport");
    QStringList fileList = imgPath.entryList(QDir::Files|QDir::NoDotAndDotDot);
    foreach(QString dirFile, fileList){
//        qDebug() << dirFile;
        imgPath.remove(dirFile);
    }
}

void RotoIO::deleteAlphaMapFiles(QString filePath, QString projName, QString curveName){
//    qDebug() << filePath;
    QDir imgPath(filePath);
    if(!imgPath.exists()) return;
    imgPath.setNameFilters(QStringList() << "alphaMap_" + projName + "_" + curveName + "_*.jpg");
//    qDebug() << imgPath;
    QStringList fileList = imgPath.entryList(QDir::Files|QDir::NoDotAndDotDot);
    foreach(QString dirFile, fileList){
//        qDebug() << dirFile;
        imgPath.remove(dirFile);
    }
}



















