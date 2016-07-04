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

#include "include/RotoStatus.hpp"

RotoStatus::RotoStatus(DesignView* v, FrameFlow* fl,KeyFrames* kf, SolverType* st, int limit)
    :view(v),flowList(fl),keyframes(kf),solverType(st){
    if(limit<20) stackLimit = 20;
    else stackLimit = limit;
}

bool RotoStatus::pushStatus(){
//    qDebug() << "status is being stored.";
    DesignSceneList* scenes = view->getScenes();
    ARotoCurve rotoStatus;
    for(QList<DesignScene *>::iterator iNode = scenes->begin(); iNode!=scenes->end();iNode++){
        NodeList* newNodes = new NodeList();
        NodeList* curNodes = (*iNode)->getPoints();
        if(!curNodes->isEmpty() && curNodes->size()>0){
            SpNode* firstNode = new SpNode();
            for(QList<SpNode *>::iterator iNode = curNodes->begin(); iNode!=curNodes->end();iNode++){
                if((*iNode)->isFirstNode()) firstNode = *iNode;
            }
            SpNode* tmpCurNode = firstNode;
            newNodes->append(new SpNode(firstNode));
            while(tmpCurNode->hasNextNode){
                if(tmpCurNode->getNextNode()->isFirstNode()){
                    newNodes->last()->setNextNode(newNodes->first());
                    newNodes->first()->setPreNode(newNodes->last());
                    break;
                }
                else{
                    SpNode* tmpNewNode = new SpNode(tmpCurNode->getNextNode());
                    newNodes->last()->setNextNode(tmpNewNode);
                    tmpNewNode->setPreNode(newNodes->last());
                    newNodes->append(tmpNewNode);
                    tmpCurNode = tmpCurNode->getNextNode();
                }
            }
        }
        rotoStatus.append(newNodes);
    }

//    if(!rotoStack.isEmpty() && !checkStatusChenge(rotoStatus)) {
//        qDebug() << "rotostack size:" << rotoStack.size();
//        qDebug() << "points not change, not stock status";
//        return false;
//    }
//    qDebug() << "points changes, stock status";
//    qDebug() << "pushstatus check 4.";
    curSceneStack.append(flowList->getindCurScene());
    rotoStack.append(rotoStatus);
    frameStack.append(flowList->getStatusList());
    solverTypeStack.append(*solverType);

    timeStampStack.append(QDateTime::currentDateTimeUtc().toString());
    flagsStack.append(view->getCurFlags());
    mouseClickStack.append(view->getCurScene()->getClick());
    mouseReleaseStack.append(view->getCurScene()->getRelease());

//    qDebug() << QDateTime::currentDateTimeUtc().toString();
//    qDebug() << QTime::currentTime().toString();

    if(stackLimit>0){
        if(curSceneStack.size()>stackLimit) curSceneStack.removeFirst();
        if(rotoStack.size()>stackLimit) rotoStack.removeFirst();
        if(frameStack.size()>stackLimit) frameStack.removeFirst();
        if(timeStampStack.size()>stackLimit) timeStampStack.removeFirst();
        if(flagsStack.size()>stackLimit) flagsStack.removeFirst();
        if(mouseClickStack.size()>stackLimit) mouseClickStack.removeFirst();
        if(mouseReleaseStack.size()>stackLimit) mouseReleaseStack.removeFirst();
        if(solverTypeStack.size()>stackLimit) solverTypeStack.removeFirst();
    }
    emit pushLog();
    return true;
}

bool RotoStatus::popStatus(){
    if(rotoStack.size()>0 && frameStack.size()>0){

        view->resetRotationAllScenes();
        ARotoCurve rotostate = rotoStack.last();
        StatusList framestate = frameStack.last();
        int curScenestate = curSceneStack.last();
        SolverType type = solverTypeStack.last();
        emit pushSolverType(type);

        DesignSceneList* scenes = view->getScenes();
        for(int iScene = 0; iScene < scenes->size();iScene++){
            (*scenes)[iScene]->cleanView();
            (*scenes)[iScene]->addNodeList(rotostate[iScene]);
            (*scenes)[iScene]->updateScene();
            (*scenes)[iScene]->isPtsVisible = true;
        }

        flowList->setAllStatus(framestate);
        flowList->updateCurScene(curScenestate);
        flowList->setCurrentRow(curScenestate);
        view->setCurScene(curScenestate);
        view->getCurScene()->checkCloseCurve();
//            qDebug() << "these is closed curve.";

        rotoStack.removeLast();
        frameStack.removeLast();
        curSceneStack.removeLast();
        solverTypeStack.removeLast();

        timeStampStack.removeLast();
        flagsStack.removeLast();
        mouseClickStack.removeLast();
        mouseReleaseStack.removeLast();
    }

    return true;
}

void RotoStatus::printCurStatus(){
    qDebug() << "number of stats: " << this->rotoStack.size();
    //    qDebug() << "number of keyframes: " << this->keyframeStack.size();
    qDebug() << "number of frames for the latest stat: " << frameStack.last().size();
}


bool RotoStatus::checkStatusChenge(ARotoCurve cur){
    qDebug() << "checking status change";
    if(rotoStack.isEmpty() || frameStack.isEmpty() || curSceneStack.isEmpty() || solverTypeStack.isEmpty()
            || timeStampStack.isEmpty() || flagsStack.isEmpty() || mouseClickStack.isEmpty() || mouseReleaseStack.isEmpty())
        return true;
    ARotoCurve pre = rotoStack.last();
    if(checkARotoCurveChange(pre,cur)) return true;



    return false;
}

bool RotoStatus::checkARotoCurveChange(ARotoCurve pre, ARotoCurve cur){
    qDebug() << "checking curves change";
    if(pre.size() != cur.size()) return true;
    else if(cur.isEmpty()) return false;
    for(int iScene = 0; iScene < pre.size(); iScene++){
        NodeList* prelist = pre.at(iScene);
        NodeList* curlist = cur.at(iScene);
        if(!prelist->isEmpty() && !curlist->isEmpty()
                && prelist->size() != curlist->size())
            return true;
        qDebug() << "checking curves change on scene " << iScene;
        if(prelist->isEmpty()) continue;
        for(int iNode = 0; iNode < curlist->size(); iNode++){
            if(prelist->at(iNode)->posChangeTo(curlist->at(iNode))) return true;
        }
    }
    return false;
}























