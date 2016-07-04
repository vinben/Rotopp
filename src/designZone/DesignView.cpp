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

#include "include/designZone/DesignView.hpp"

DesignView::DesignView(QWidget* parent) : QGraphicsView(parent)
{
    flagZooming = false;
    flagDragScene = false;
    flagPtsVis = true;
    curScene = new DesignScene();
    this->centerOn(QPoint(0,0));
    this->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    this->setEnabled(true);
    this->setMinimumSize(QSize(1000, 600));
    this->setInteractive(true);
    this->setRenderHints(QPainter::Antialiasing|QPainter::HighQualityAntialiasing|QPainter::NonCosmeticDefaultPen|QPainter::TextAntialiasing);
    this->setCacheMode(QGraphicsView::CacheNone);
    this->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    this->setMouseTracking(true);
}

void DesignView::updateNodesAllScene(TrackerData3 track){
    if(track.nodes.size()==scenes.size()){
//        qDebug() << "updateNodesAllScene starts";
        for(int iScene = 0; iScene < scenes.size(); iScene++){
            NodeList* tmplist = scenes[iScene]->getPoints();
            int indkey = track.mostCloseIndKeys(iScene,!track.flagTrackDirection);
            if(indkey > -1 && indkey<=scenes.size()){
//                qDebug() << "the most cloest keyframe: " << indkey;
                if(tmplist->size() != scenes[indkey]->getPoints()->size()){
                    scenes[iScene]->cleanNodes();
                    scenes[iScene]->copyNodeList(scenes[indkey]->getPoints());
                    scenes[iScene]->updateNodePos(track.nodes.at(iScene),false);
//                    scenes[iScene]->setStatus(EditType::SUGGESTED);
//                    qDebug() << "updateNodesAllScene creating points running";
                }
                else {
                    scenes[iScene]->updateNodePos(track.nodes.at(iScene),false);
//                    scenes[iScene]->setStatus(EditType::SUGGESTED);
//                    qDebug() << "updateNodesAllScene updating points running";
                }
            }else if(scenes[iScene]->getPoints()->isEmpty()){
                scenes[iScene]->cleanNodes();
                scenes[iScene]->initACurve(track.nodes.at(iScene).size());
                scenes[iScene]->updateNodePos(track.nodes.at(iScene),false);
            }
        }
    }
}

bool DesignView::insertGlobalPtAllScene(int pointsSize, int indPreNode, float factor){
    if(scenes.isEmpty()) return false;
    for(int iScene = 0; iScene < scenes.size(); iScene++){
        if(scenes[iScene]->getPoints()->size() == pointsSize){
            NodeList* nodelist = scenes[iScene]->getPoints();
            SpNode* preNode = (*nodelist)[indPreNode];
            SpNode* inNode = preNode->interpNode(factor);

            inNode->setPreNode(preNode);
            inNode->setNextNode(preNode->getNextNode());
            preNode->setNextNode(inNode);
            inNode->getNextNode()->setPreNode(inNode);

            int ind = nodelist->indexOf(inNode->getNextNode());
            if(ind==0) nodelist->append(inNode);
            else nodelist->insert(ind,inNode);

            scenes[iScene]->addItem(inNode);
            scenes[iScene]->updateScene();
        }
    }
    return true;
}

bool DesignView::checkPtsNumConsKeyframes(QList<int> indKeys) {
    if(indKeys.isEmpty()) return true;
    int ptSize = curScene->getPoints()->size();
    for(int iKey = 0; iKey < indKeys.size(); iKey++){
        int ptSceneSize = scenes.at(indKeys.at(iKey))->getPoints()->size();
        if(ptSceneSize!=ptSize) return false;
    }
    return true;
}

bool DesignView::globalPtsDeleteAllScene(){
    int ptSize = curScene->getPoints()->size();
    QList<int> ptSlt = curScene->getIndSltPts();
    if(ptSlt.isEmpty() || ptSize == 0 || ptSize < ptSlt.size()) return false;
    for(int iSltPt = 0; iSltPt < ptSlt.size(); iSltPt++){
        QList<int> ptStepSlt = curScene->getIndSltPts();
        int ptStepSize = curScene->getPoints()->size();
        if(ptStepSlt.isEmpty() || ptStepSize == 0 || ptStepSize < ptStepSlt.size()) return false;
        setGlobalPtDeleteAllScene(ptStepSize,ptStepSlt.last());
    }
    curScene->pushEditing();
    return true;
}

bool DesignView::setGlobalPtDeleteAllScene(int pointsSize, int indSltPt){
    if(scenes.isEmpty()) return false;
    for(int iScene = 0; iScene < scenes.size(); iScene++){
        if(scenes[iScene]->getPoints()->size() == pointsSize){

            NodeList* nodelist = scenes[iScene]->getPoints();

            if(!(*nodelist)[indSltPt]->hasPreNode
                    && (*nodelist)[indSltPt]->hasNextNode){

                (*nodelist)[indSltPt]->getNextNode()->rmPreNode();
                if((*nodelist)[indSltPt]->isFirstNode())
                    (*nodelist)[indSltPt]->getNextNode()->setFirstNode();
            }
            else if((*nodelist)[indSltPt]->hasPreNode
                     && !(*nodelist)[indSltPt]->hasNextNode){

                (*nodelist)[indSltPt]->getPreNode()->rmNextNode();
            }
            else if((*nodelist)[indSltPt]->hasPreNode
                     && (*nodelist)[indSltPt]->hasNextNode){

                SpNode* prePt = (*nodelist)[indSltPt]->getPreNode();
                SpNode* nextPt = (*nodelist)[indSltPt]->getNextNode();
                prePt->setNextNode(nextPt);
                nextPt->setPreNode(prePt);

                if((*nodelist)[indSltPt]->isFirstNode())
                    nextPt->setFirstNode();
            }

            scenes[iScene]->removeItem((*nodelist)[indSltPt]);
            (*nodelist).removeAll((*nodelist)[indSltPt]);
            scenes[iScene]->checkCloseCurve();
            scenes[iScene]->updateScene();

//            NodeList* nodelist = scenes[iScene]->getPoints();
//            SpNode* preNode = (*nodelist)[indPreNode];
//            SpNode* inNode = preNode->interpNode(factor);

//            inNode->setPreNode(preNode);
//            inNode->setNextNode(preNode->getNextNode());
//            preNode->setNextNode(inNode);
//            inNode->getNextNode()->setPreNode(inNode);

//            int ind = nodelist->indexOf(inNode->getNextNode());
//            if(ind==0) nodelist->append(inNode);
//            else nodelist->insert(ind,inNode);

//            scenes[iScene]->addItem(inNode);
//            scenes[iScene]->updateScene();
        }
    }
    return true;
}

bool DesignView::setClosedCurveAllScene(int pointsSize){
    qDebug() << "setClosedCurveAllScene is called.";
    if(scenes.isEmpty()) return false;
    for(int iScene = 0; iScene < scenes.size(); iScene++){
        if(scenes[iScene]->getPoints()->size() == pointsSize){
            NodeList* nodelist = scenes[iScene]->getPoints();
            SpNode* startNode = new SpNode();
            for(int iPt = 0; iPt < pointsSize; iPt++){
                if(nodelist->at(iPt)->isFirstNode()){
                    startNode = (*nodelist)[iPt];
                    break;
                }
            }
            if(!startNode->hasPreNode){
                startNode->setPreNode(nodelist->last());
                nodelist->last()->setNextNode(startNode);
            }
        }
        scenes[iScene]->updateScene();
        scenes[iScene]->hasClosedCurve = true;
    }
    return true;
}

void DesignView::changeScene(qreal ind){
    scenes[ind]->displayPoly(false);
    scenes[ind]->resetDefaultFlags();
    this->setCurScene(scenes[ind]);
    this->getCurScene()->checkCloseCurve();
    this->getCurScene()->updateDisplayImg();
    emit pushRecheckDesignActions();
    emit pushInitSniperView(scenes[ind]->getImg());
}

void DesignView::zoomin(bool in){
    qreal zoomFactor = 0;
    qreal zoomInFactor = 1.2;
    qreal zoomOutFactor = 1 / zoomInFactor;
    QPointF oldPos = getCurScene()->getMouseMove();
    this->setTransformationAnchor(QGraphicsView::NoAnchor);
    this->setResizeAnchor(QGraphicsView::NoAnchor);

    if(flagZooming && !flagDragScene){
        if(in) zoomFactor = zoomInFactor;
        else zoomFactor = zoomOutFactor;
        this->scale(zoomFactor,zoomFactor);
        QPointF newPos = getCurScene()->getMouseMove();
        this->translate((newPos-oldPos).x(),(newPos-oldPos).y());
        this->getCurScene()->rszPtsAndLine(transform().m11());
        //        qDebug() << "scale factor: " << this->transform().m11() << " " << this->transform().m12();
    }

}

void DesignView::zoomin(QPointF p){
    qreal zoomFactor = 0;
    qreal zoomInFactor = 1.1;
    qreal zoomOutFactor = 1 / zoomInFactor;
    QPointF oldPos = getCurScene()->getMouseMove();
    this->setTransformationAnchor(QGraphicsView::NoAnchor);
    this->setResizeAnchor(QGraphicsView::NoAnchor);
    QSize viewSize = this->size();
//    qDebug() << "view size: " << rect.topLeft().x() << rect.topLeft().y() << ", other side: " << rect.bottomRight().x() << rect.bottomRight().y();
//    qDebug() << "view size: " << viewSize.width() << viewSize.height();
//    qDebug() << "rightMove point:" << p.x() << p.y();
    QPointF diff = p - pos_rightclick;
    if((abs(diff.x())/viewSize.width() > 0.01 && abs(diff.x()/viewSize.width()) < 0.012)
            || (abs(diff.y()/viewSize.height()) > 0.01 && abs(diff.y()/viewSize.height()) < 0.012)){
        qDebug() << "rightMove point:" << p.x() << p.y();
        qDebug() << "rightclick point:" << pos_rightclick.x() << pos_rightclick.y();
        if(diff.x()>0 || diff.y()<0) zoomFactor = 1 + abs(diff.x())/viewSize.width();
        else zoomFactor = 1 - abs(diff.x())/viewSize.width();
        this->scale(zoomFactor,zoomFactor);
        QPointF newPos = getCurScene()->getMouseMove();
        this->translate((newPos-oldPos).x(),(newPos-oldPos).y());
        this->getCurScene()->rszPtsAndLine(transform().m11());
        pos_rightclick = p;
//        qDebug() << "drag zooming in view";


//        if(diff.x()>0 || diff.y()<0) zoomin(true);
//        else zoomin(false);

    }

    //        qDebug() << "scale factor: " << this->transform().m11() << " " << this->transform().m12();

}

void DesignView::wheelEvent(QWheelEvent* event){
    qreal zoomFactor = 0;
    qreal zoomInFactor = 1.2;
    qreal zoomOutFactor = 1 / zoomInFactor;
    QPointF oldPos = this->mapToScene(event->pos());

    this->setTransformationAnchor(QGraphicsView::NoAnchor);
    this->setResizeAnchor(QGraphicsView::NoAnchor);

    //    if(event->modifiers() & Qt::AltModifier )
    if(flagZooming && !flagDragScene){
        if(event->delta() > 0) zoomFactor = zoomInFactor;
        else zoomFactor = zoomOutFactor;
        this->scale(zoomFactor,zoomFactor);
        QPointF newPos = this->mapToScene(event->pos());
        this->translate((newPos-oldPos).x(),(newPos-oldPos).y());
        this->getCurScene()->rszPtsAndLine(transform().m11());
        //        qDebug() << "scale factor: " << this->transform().m11() << " " << this->transform().m12();
    }
    else QGraphicsView::wheelEvent(event);
}

void DesignView::setCurScene(DesignScene* s){
    curScene = s;
    //    this->centerOn(QPoint(curScene->getImg()->width()/2,curScene->getImg()->height()/2));
    this->setScene(curScene);
}

void DesignView::setCurScene(int s){
    if(s>-1 && s<scenes.size()){
        curScene = scenes[s];
        //        this->centerOn(QPoint(curScene->getImg()->width()/2,curScene->getImg()->height()/2));
        this->setScene(curScene);
    }

}

void DesignView::copyNodeListFromTo(int from, int to){
    scenes.at(to)->displayPoly(false);
    scenes.at(to)->cleanView();
    scenes.at(to)->addNodeList(scenes.at(from)->getPoints());
    scenes.at(to)->updateScene();
}

void DesignView::initScenes(QList<QImage*> im){
    scenes.clear();
    for(int iImg = 0; iImg<im.size();iImg++){
        scenes.append(new DesignScene(im[iImg]));
    }
    this->update();
    qDebug() << "scenes initialization done!";
    emit pushScenes(scenes);
}

void DesignView::setFlagZooming(bool checked){
    if(checked) flagZooming = true;
    else flagZooming = false;
}

void DesignView::setCurveColor(QColor c){
    for(QList<DesignScene *>::iterator iNode = scenes.begin(); iNode!=scenes.end();iNode++){
        (*iNode)->setCurveColor(c);
//        if((*iNode)->hasPoly)
        (*iNode)->getPoly()->setColor(c);
    }
}

QList<bool> DesignView::getCurFlags(){
    QList<bool> flags;
    flags.append(curScene->flagCreate);
    flags.append(curScene->flagDelete);
    flags.append(curScene->flagDragPts);
    flags.append(curScene->flagModPoint);
    flags.append(curScene->flagMove);
    flags.append(curScene->flagRotation);
    flags.append(curScene->flagSelection);
    flags.append(curScene->flagShowPoly);
    return flags;
}

int DesignView::getNumAllPoints(){
    int num = 0;
    for(QList<DesignScene *>::iterator iNode = scenes.begin(); iNode!=scenes.end();iNode++){
        num = num + (*iNode)->getPoints()->size();
    }
    return num;
}

void DesignView::checkClosedCurveAllScenes(){
    for(QList<DesignScene *>::iterator iNode = scenes.begin(); iNode!=scenes.end();iNode++){
        (*iNode)->checkCloseCurve();
    }
}

void DesignView::resetRotationAllScenes(){
    for(QList<DesignScene *>::iterator iNode = scenes.begin(); iNode!=scenes.end();iNode++)
        (*iNode)->resetCallRotationMode();
}

void DesignView::cleanCurScene(){
    curScene->resetCallRotationMode();
    curScene->cleanView();
}

void DesignView::updateRightClick(QPointF c){
//    pos_rightclick = mapFromScene(c);
    pos_rightclick = c;
//    qDebug() << "current right move position: " << pos_rightmove.x() << pos_rightmove.y();
//    qDebug() << "current right click position: " << pos_rightclick.x() << pos_rightclick.y();
}

void DesignView::dragView(QPointF c){
//    pos_rightmove = mapFromScene(c);
    pos_rightmove = c;
    float hValue = horizontalScrollBar()->value() - (pos_rightmove.x() - pos_rightclick.x());
    float vValue = verticalScrollBar()->value() - (pos_rightmove.y() - pos_rightclick.y());
    if(hValue >= horizontalScrollBar()->minimum() && hValue <= horizontalScrollBar()->maximum())
        horizontalScrollBar()->setValue(hValue);
    if(vValue >= verticalScrollBar()->minimum() && vValue <= verticalScrollBar()->maximum())
        verticalScrollBar()->setValue(vValue);
    pos_rightclick = pos_rightmove;
}

void DesignView::setDragScene(bool t){
    flagDragScene = t;
    if(flagDragScene) setCursor(Qt::ClosedHandCursor);
    else setCursor(Qt::ArrowCursor);
}

void DesignView::resetImageProperty(){
    for(QList<DesignScene *>::iterator iNode = scenes.begin(); iNode!=scenes.end();iNode++){
        (*iNode)->setBrightness(DEFAULT_BRIGHTNESS);
        (*iNode)->setContrast(DEFAULT_CONTRAST);
        (*iNode)->setGamma(DEFAULT_GAMMA);
        (*iNode)->setImgProcChange(false);
        (*iNode)->setUpdateImgProcChange(false);
    }
//    getCurScene()->updateDisplayImg();
    getCurScene()->update();
}

void DesignView::setImagePropertyAllScene(int b, int c, int g){
    for(QList<DesignScene *>::iterator iNode = scenes.begin(); iNode!=scenes.end();iNode++){
        (*iNode)->setBrightness(b);
        (*iNode)->setContrast(c);
        (*iNode)->setGamma(g);
        if(b!=DEFAULT_BRIGHTNESS || c!=DEFAULT_CONTRAST || g!=DEFAULT_GAMMA){
            (*iNode)->setImgProcChange(true);
            (*iNode)->setUpdateImgProcChange(false);
        }
    }
    getCurScene()->updateDisplayImg();
    getCurScene()->update();
}

void DesignView::fitSceneToView(){
    QSize viewSize = this->size();
    QSize imgSize = this->getCurScene()->getImg()->size();
    float rateWid = float(viewSize.width())/float(imgSize.width());
    float rateHeig = float(viewSize.height())/float(imgSize.height());
    this->resetTransform();
    this->centerOn(0,0);
    float zoomFactor = rateWid < rateHeig ? rateWid : rateHeig;
    this->scale(zoomFactor - 0.1, zoomFactor - 0.1);
//    qDebug() << "rate of width and height:" << rateWid << rateHeig;
}

void DesignView::hidePointsAllScenes(){
    for(QList<DesignScene *>::iterator iNode = scenes.begin(); iNode!=scenes.end();iNode++)
            (*iNode)->hidePoints();
}

void DesignView::showPointsAllScenes(){
    for(QList<DesignScene *>::iterator iNode = scenes.begin(); iNode!=scenes.end();iNode++)
        (*iNode)->showPoints();
}

void DesignView::setPtsVisAllScenes(){
    int count = 0;
    for(QList<DesignScene *>::iterator iNode = scenes.begin(); iNode!=scenes.end();iNode++){
        if(flagPtsVis){
            (*iNode)->hidePoints();
            count++;
        }
        else if(!flagPtsVis){
            (*iNode)->showPoints();
            count++;
        }
    }
    if(count>0){
        flagPtsVis = !flagPtsVis;
    }
}

void DesignView::setPtsVisAllScenes(bool checked){
    for(QList<DesignScene *>::iterator iNode = scenes.begin(); iNode!=scenes.end();iNode++){
        if(checked){
            (*iNode)->showPoints();
        }
        else if(!checked){
            (*iNode)->hidePoints();
        }
    }

    if(checked) flagPtsVis = true;
    else flagPtsVis = false;
}
//
//void DesignView::smoothPts(){
//
//}
//
//void DesignView::cuspPts(){
//
//}









