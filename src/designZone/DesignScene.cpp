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

#include "include/designZone/DesignScene.hpp"
#include "include/RotoConfig.h"

using namespace std;

DesignScene::DesignScene(QImage* im){
    status = EditType::NOEDIT;
    pos_move = QPointF(0,0);
    curveColor = Qt::red;
    points = new QList<SpNode*>;

    poly = new SpPoly(points,curveColor);

    scale = 1;
    valueContrast = DEFAULT_CONTRAST;
    valueBrightness = DEFAULT_BRIGHTNESS;
    valueGama = DEFAULT_GAMMA;
    img = im;
    this->setSceneRect(QRect(-img->width(),-img->height(),2*img->width(), 2*img->height()));

    flagMove = false;
    flagCreate = false;
    flagShowPoly = false;
    flagModPoint = false;
    flagDragPts = false;
    flagDelete = false;
    flagRotation = false;
    flagSelection = true;

    isPtsVisible = true;

    hasImgProcChange = false;
    hasUpdateImgProcChange = false;

    hasClosedCurve = false;
    hasSltPts = false;
    hasSelectedPt = false;
    hasSelectedLeft = false;
    hasSelectedRight = false;
    hasAvaiPt = false;
    hasDragList = false;
    hasSltRect = false;
    hasRectRotate = false;

    this->resetDragList();
    this->resetSelectedPt();
}

QImage DesignScene::getAlphaMap(bool isWhiteAlpha){

    QImage res = QImage(img->size(), QImage::Format_RGB32);
    QColor cBackground;
    QColor cAlpha;
    if(isWhiteAlpha) {
        cBackground = Qt::black;
        cAlpha = Qt::white;
    }
    else{
        cAlpha = Qt::black;
        cBackground = Qt::white;
    }
    res.fill(cBackground);
    if(points->size()>1){
        SpPoly shape(points,cAlpha);
        QPainterPath* paths = shape.getPaths();
        QPainter painter;
        painter.begin(&res);
        QPen pen = QPen(cAlpha);
        pen.setWidth(0);
        painter.setPen(pen);
        painter.setBrush(QBrush(cAlpha));
        paths->translate(QPointF(img->size().width()/2,img->size().height()/2));
        paths->setFillRule(Qt::WindingFill);
        painter.drawPath(paths->simplified());
        painter.end();
    }
    return res;
}

QImage DesignScene::getCurveMap(bool isFilled, bool isPrintPt, float penWidth){

    QImage res = img->copy();

    if(points->size()>1) {
        QColor alpha = curveColor;
        SpPoly shape(points, alpha);
        QPainterPath *paths = shape.getPaths();
        QPainter painter;
        painter.begin(&res);
        if (isFilled) alpha.setAlpha(128);
        QPen pen = QPen(alpha);
        pen.setWidth(penWidth);

        painter.setPen(pen);

        if (isFilled) {
            painter.setBrush(QBrush(alpha));
            paths->setFillRule(Qt::WindingFill);
        }

        paths->translate(QPointF(img->size().width() / 2, img->size().height() / 2));
        painter.drawPath(paths->simplified());

        if (isPrintPt) {
        QPen ptPen(QColor(255, 192, 0));
        painter.setPen(ptPen);
        painter.setBrush(QBrush(QColor(255, 192, 0)));
        paths->setFillRule(Qt::WindingFill);
        for (int iPt = 0; iPt < points->size(); iPt++) {
            QRectF rect(-penWidth * 2, -penWidth * 2, penWidth * 2, penWidth * 2);
            rect.moveTo(points->at(iPt)->position +
                        QPointF(img->size().width() / 2 - penWidth, img->size().height() / 2 - penWidth));
            painter.drawRect(rect);
        }
        }

        painter.end();
    }
    return res;
}

QImage* DesignScene::loadImg(QString str){
    return new QImage(str);
}

QList<int> DesignScene::getIndSltPts(){
    QList<int> indSlt;
    for(int iPt = 0; iPt < points->size(); iPt++){
        if(points->at(iPt)->isSlted) indSlt.append(iPt);
    }
    return indSlt;
}

void DesignScene::drawBackground(QPainter * painter, const QRectF & rect){
    Q_UNUSED(rect);
    //    qDebug() << "image's width: " << background.width();
    painter->setBrush(Qt::black);
    painter->drawRect(QRectF(-20*img->width(), -20*img->height(), 40*img->width(), 40*img->height()));

    if(valueBrightness!=DEFAULT_BRIGHTNESS || valueContrast!=DEFAULT_CONTRAST || valueGama!=DEFAULT_GAMMA)
        painter->drawImage(QRectF(-img->width()/2, -img->height()/2, img->width(), img->height()), dispImg);
    else
        painter->drawImage(QRectF(-img->width()/2, -img->height()/2, img->width(), img->height()), *img);
}

void DesignScene::mousePressEvent(QGraphicsSceneMouseEvent *event){
    if(!event->buttons()) return;
    pos_click = event->scenePos();
    if(event->buttons() == Qt::LeftButton){
        //    qDebug() << "single click on x: " << pos_click.x() << ", y: " << pos_click.y() << endl;
        indSltPt = getSelectedPt();
    }
    else if((event->modifiers()==Qt::KeyboardModifier::AltModifier && event->buttons() == Qt::MidButton)
            ||(event->modifiers()==Qt::KeyboardModifier::ControlModifier && event->buttons() == Qt::MidButton)){
        emit pushRightClick(pos_click);
//        qDebug() << "detect mid";
    }
    else if(event->buttons() == Qt::MidButton){
        emit pushDragScene(true);
        emit pushRightClick(pos_click);
    }
    if((flagSelection||flagMove) && event->buttons() == Qt::LeftButton){
//        qDebug() << "indSltPt: " << indSltPt << "hasSltPts" << hasSltPts;
        if(indSltPt==-1 && hasSltPts){
            if(!checkInPtsArea(sltPts,pos_click)) {
                resetDragList();
                resetSltPts();
            }else setOrigPosDragList(sltPts);
        }
        if(!hasSltRect && indSltPt<0 && (!hasSltPts || !checkInPtsArea(sltPts,pos_click))){
                setSltRect(pos_click);
                this->addItem(sltRect);
                qDebug() << "add a selection rect";
        }
        else if(indSltPt>-1 && hasSltPts){
            setOrigPosDragList(sltPts);
        }
    }

    else if (flagCreate && event->buttons() == Qt::LeftButton) {
        // Points creation mode
        if(hasSelectedPt && points->size()>1){
            // in case of selecting an existed point.
            SpNode *newPoint = (*points)[indSltPt];
            newPoint->setVis(true);
            if(newPoint->isFirstNode() || !newPoint->hasPreNode){
                newPoint->setPreNode(points->last());
                points->last()->setNextNode(newPoint);
                emit pushClosedCurve(points->size());
            }
            if(newPoint->isFirstNode()) hasClosedCurve = true;
//            emit pushStatus();
            emit pushEditing();
        }
        else if(!hasClosedCurve && !hasSelectedPt){
            // in case of clicking at total empty area and there is no closed curve.
            SpNode *newPoint = new SpNode(pos_click,QPointF(0,0),QPointF(0,0),scale);
            if(!points->isEmpty()){

//                if(SMART_BEZIER == 1){

#ifdef ENABLE_SMART_BEZIER
                    QVector2D norVec(pos_click - points->last()->getLef()->scenePos());
                    QVector2D lengthTanVec(points->last()->getLef()->getPosition());
                    newPoint->getLef()->setPosition(norVec.normalized().toPointF()*lengthTanVec.length());
                    newPoint->getLef()->setPos(norVec.normalized().toPointF()*lengthTanVec.length());
                    newPoint->getRig()->setPosition(-norVec.normalized().toPointF()*lengthTanVec.length());
                    newPoint->getRig()->setPos(-norVec.normalized().toPointF()*lengthTanVec.length());
#endif

                newPoint->setPreNode(points->last());
                if(points->last()->hasNextNode) points->last()->getNextNode()->rmPreNode();
                points->last()->setNextNode(newPoint);
            }else newPoint->setFirstNode();
            points->append(newPoint);
            this-> addItem(newPoint);
            indSltPt = getSelectedPt(newPoint);
            emit pushStatus();
            emit pushEditing();
            qDebug() << "new point's position, x: " << newPoint->getPosition().x() << ", y: "<< newPoint->getPosition().y() ;
        } else if(hasClosedCurve && !hasSelectedPt){
            emit pushSelectionMode();
        }
        else resetSelectedPt();
    }
    else if (flagModPoint && event->buttons() == Qt::LeftButton){
        // Points adding and tangent points modification mode
//        qDebug() << event->modifiers();
        if( !hasSelectedPt && points->size()>1){
            SpNode* addedPt = new SpNode(pos_click,QPointF(0,0),QPointF(0,0),scale);
            this->nodeNeighbouringByCurve(addedPt,curves);
            int indNextNode = points->indexOf(addedPt->getNextNode());
            if(indNextNode>0)
                points->insert(indNextNode,addedPt);
            else points->append(addedPt);
            if(!insertGlobalPt(addedPt)) qDebug() << "Global inserting point fails.";
            indSltPt = getSelectedPt(addedPt);
            this-> addItem(addedPt);
            emit pushStatus();
            emit pushEditing();
        }
        else if(hasSelectedPt){
            sltPoint->setVis(true);
        }
    }
    else if (flagDelete && hasSelectedPt && event->buttons() == Qt::LeftButton){

        if(!(*points)[indSltPt]->hasPreNode
                && (*points)[indSltPt]->hasNextNode){

            (*points)[indSltPt]->getNextNode()->rmPreNode();
            if((*points)[indSltPt]->isFirstNode())
                (*points)[indSltPt]->getNextNode()->setFirstNode();
        }
        else if((*points)[indSltPt]->hasPreNode
                && !(*points)[indSltPt]->hasNextNode){

            (*points)[indSltPt]->getPreNode()->rmNextNode();
        }
        else if((*points)[indSltPt]->hasPreNode
                && (*points)[indSltPt]->hasNextNode){

            SpNode* prePt = (*points)[indSltPt]->getPreNode();
            SpNode* nextPt = (*points)[indSltPt]->getNextNode();
            prePt->setNextNode(nextPt);
            nextPt->setPreNode(prePt);

            if((*points)[indSltPt]->isFirstNode())
                nextPt->setFirstNode();
        }

        this->removeItem((*points)[indSltPt]);
        (*points).removeAll((*points)[indSltPt]);
        if(points->size()<2) checkCloseCurve();
        emit pushGlobalDelect(points->size()+1,indSltPt);
        emit pushStatus();
        emit pushEditing();
        qDebug() << "deleted point " << indSltPt;
    }
    else if(flagDragPts && event->buttons() == Qt::LeftButton){
        if(hasSltPts) {
            if(checkInPtsArea(sltPts,pos_click)){
                setOrigPosDragList(sltPts);
                emit pushStatus();
            }
            else emit pushSelectionMode();
        }
        else {
            if(checkInPtsArea(*points,pos_click)){
                setOrigPosDragList(*points);
                emit pushStatus();
            }
            else emit pushSelectionMode();
        }
    }
    else if(flagSolverDragPts && event->buttons() == Qt::LeftButton){

        if(hasSltPts) {
            if(checkInPtsArea(sltPts,pos_click) || (hasSelectedPt && sltPts.contains(sltPoint))){
                setOrigPosDragList(sltPts);
                emit pushStatus();
                emit pushEditing();
                emit pushInitGPLVM();
            }
            else if(!hasSelectedPt) {
                resetDragList();
                resetSltPts();
                emit pushSelectionMode();
            }
        }
        else {
            if(checkInPtsArea(*points,pos_click)){
                setOrigPosDragList(*points);
                emit pushStatus();
                emit pushEditing();
                emit pushInitGPLVM();
            }
            else if(!hasSelectedPt) emit pushSelectionMode();
        }
    }
    else if(flagRotation && hasRectRotate && event->buttons() == Qt::LeftButton){
        if(!rotator->getRect()->labelHandler(pos_click))
            emit pushSelectionMode();
        //        emit pushStatus();
    }
    updateScene();
    //    qDebug() << "Number of points: " << points->size();
    //    qDebug() << "Number of curves: " << curves.size();
}

void DesignScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    pos_release = event ->scenePos();
    emit pushDragScene(false);

    if(flagSelection||flagMove){
        resetSltRect();
        if(hasSelectedPt){
            if(hasSltPts){
//                emit pushStatus();
                resetDragList();
                resetSelectedPt();
            }
            else if(!hasSelectedLeft && !hasSelectedRight){
                (*points)[indSltPt] -> setPos(pos_release);
            }
            emit pushStatus();
        }

        resetSelectedPt();
    }
    else if(flagCreate && points->size()>0){
        if(!isPtsVisible) sltPoint->setVis(false);
        if(points->size()>1){
            SpNode *p2 = points->last();
            SpNode *p1 = p2->getPreNode();
            SpCurve *newCurve = new SpCurve(p1, p2, curveColor,scale);
            curves.append(newCurve);
            this->addItem(newCurve);
            if(!hasClosedCurve || hasSelectedPt) emit pushStatus();
            resetSelectedPt();
        }
        //        qDebug() << "curve's position, x: " << newCurve->getPosition().x() << ", y: "<< newCurve->getPosition().y();
    }
    else if(flagSolverDragPts){
        if(!hasSelectedPt) pushSolveSingleFrame();
    }
    else if(flagRotation && hasRectRotate){
        resetSelectedPt();
        rotator->getRect()->resetLabels();
        //        resetCallRotationMode();
    }
    else if(flagModPoint){
        if(!isPtsVisible) sltPoint->setVis(false);
//        qDebug() << "flagModPoint mouse release";
        emit pushStatus();
    }
    updateScene();
    //    qDebug() << "number of points in this scene: " << points->size();
    //    rerenderScene();
}

void DesignScene::mouseMoveEvent(QGraphicsSceneMouseEvent *event){

    pos_move = event->scenePos();
    emit pushCursor(pos_move);
    //    qDebug() << "mouse pos record, x: " << pos_move.x() << " y: " << pos_move.y();

    if((event->modifiers() == Qt::KeyboardModifier::AltModifier
        || event->modifiers() == Qt::KeyboardModifier::ControlModifier)
            && event->buttons() == Qt::MidButton){
    }
    else if(event->buttons() == Qt::MidButton){
        emit pushRightMove(pos_move);
    }
    else if((flagCreate || flagModPoint) && hasSelectedPt && !hasSelectedLeft && !hasSelectedRight && event->buttons()==Qt::LeftButton){
        sltPoint->getLef()->setPosition(pos_move - pos_click);
        sltPoint->getLef()->setPos(sltPoint->getLef()->getPosition());
        sltPoint->getRig()->setPosition(pos_click - pos_move);
        sltPoint->getRig()->setPos(sltPoint->getRig()->getPosition());
    }

    else if((flagSelection || flagMove) && event->buttons()==Qt::LeftButton){
        if(hasSltRect){
            sltRect->setRect(pos_click, event->scenePos());
            if(sltRect->labelSltPts(*points)) setSltPts();
            else resetSltPts();
        }
        else if(hasSelectedPt) {
            if(hasSltPts && sltPts.size()>1 && sltPts.contains(sltPoint)){
                translateSltPts(pos_move-pos_click,(*points)[indSltPt],origPosDragList,sltPts);
            }
            else if(hasSelectedLeft && isPtsVisible){
                sltPoint->getLef()->setPosition(pos_move - sltPoint->getPosition());
                sltPoint->getLef()->setPos(sltPoint->getLef()->getPosition());
                sltPoint->getRig()->setPosition(sltPoint->getPosition() - pos_move);
                sltPoint->getRig()->setPos(sltPoint->getRig()->getPosition());
            }
            else if(hasSelectedRight && isPtsVisible){
                sltPoint->getRig()->setPosition(pos_move - sltPoint->getPosition());
                sltPoint->getRig()->setPos(sltPoint->getRig()->getPosition());
                sltPoint->getLef()->setPosition(sltPoint->getPosition() - pos_move);
                sltPoint->getLef()->setPos(sltPoint->getLef()->getPosition());
            }
            else{
                sltPoint->setPosition(event->scenePos());
                sltPoint->setPos(event->scenePos());
            }
            emit pushEditing();
        }
        else if(!hasSelectedPt && hasSltPts && hasDragList){
            if(checkInPtsArea(sltPts,pos_move)){
                translatePts(pos_move - pos_click,origPosDragList,sltPts,true);
                emit pushEditing();
            }
        }
    }
    else if(flagDragPts && hasDragList && event->buttons()==Qt::LeftButton){
        if(hasSltPts) translatePts(pos_move - pos_click,origPosDragList,sltPts,true);
        else translatePts(pos_move - pos_click,origPosDragList,*points,true);
        updateScene();
    }
    else if(flagSolverDragPts && hasDragList && event->buttons()==Qt::LeftButton){

        if(hasSelectedPt && hasSltPts && sltPts.size()>1 && sltPts.contains(sltPoint)){
                translateSltPts(pos_move-pos_click,(*points)[indSltPt],origPosDragList,sltPts);
        }

        if(qtDistance(pos_click,pos_move) > 10){
            if(hasSltPts) {
                setOrigPosDragList(sltPts);
                translatePts(pos_move - pos_click,origPosDragList,sltPts,true);
            }
            else {
                setOrigPosDragList(*points);
                translatePts(pos_move - pos_click,origPosDragList,*points,true);
            }
            qDebug() << "assisted mode drag: " << qtDistance(pos_click,pos_move);
            pos_click = pos_move;
        }
        else{
            emit pushSolveSingleFrame();
        }
    }
    else if(flagRotation && hasRectRotate && event->buttons()==Qt::LeftButton){
        rotator->handlerMoveTo(event->scenePos());
        emit pushEditing();
    }
    if(event->buttons()==Qt::LeftButton) updateScene();
}

bool DesignScene::copyNodeList(NodeList* curNodes, NodeList* newNodes){

    if(!curNodes->isEmpty()){
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
    } else return false;
    return true;
}

bool DesignScene::copyNodeList(NodeList* curNodes){
    if(!curNodes->isEmpty()){
        SpNode* firstNode = new SpNode();
        for(QList<SpNode *>::iterator iNode = curNodes->begin(); iNode!=curNodes->end();iNode++){
            if((*iNode)->isFirstNode()) firstNode = *iNode;
        }
        SpNode* tmpCurNode = firstNode;
        if(!points->isEmpty()) points->clear();
        this->addNode(new SpNode(firstNode));
        while(tmpCurNode->hasNextNode){
            if(tmpCurNode->getNextNode()->isFirstNode()){
                points->last()->setNextNode(points->first());
                points->first()->setPreNode(points->last());
                break;
            }
            else{
                SpNode* tmpNewNode = new SpNode(tmpCurNode->getNextNode());
                points->last()->setNextNode(tmpNewNode);
                tmpNewNode->setPreNode(points->last());
                this->addNode(tmpNewNode);
                tmpCurNode = tmpCurNode->getNextNode();
            }
        }
    } else return false;
    this->updateScene();
    return true;
}

bool DesignScene::updateNodePos(VecNodeList nodes, bool selected){
    if(nodes.size()!=points->size()) return false;
    if(!selected){
        for(int iNode = 0; iNode < points->size(); iNode++){
            (*points)[iNode]->setPosition(nodes.at(iNode).at(0));
            (*points)[iNode]->getLef()->setPosition(nodes.at(iNode).at(1));
            (*points)[iNode]->getRig()->setPosition(nodes.at(iNode).at(2));
            (*points)[iNode]->updatePositionToScene();
        }
    }else{
        for(int iNode = 0; iNode < points->size(); iNode++){
            if(!(*points)[iNode]->isSlted) continue;
            (*points)[iNode]->setPosition(nodes.at(iNode).at(0));
            (*points)[iNode]->getLef()->setPosition(nodes.at(iNode).at(1));
            (*points)[iNode]->getRig()->setPosition(nodes.at(iNode).at(2));
            (*points)[iNode]->updatePositionToScene();
        }
    }
    updateScene();
    return true;
}

void DesignScene::initACurve(int n){
    if(n < 1) return;
    cleanNodes();

    for(int iPt = 0; iPt < n; iPt++){
        SpNode* newPt = new SpNode(QPointF(0,0),QPointF(0,0),QPointF(0,0),scale);
        if(points->size()>0){
            points->last()->setNextNode(newPt);
            newPt->setPreNode(points->last());
        }else{
            newPt->setFirstNode();
        }
        points->append(newPt);
        this->addItem(newPt);
    }

    if(n>1){
        points->last()->setNextNode(points->first());
        points->first()->setPreNode(points->last());
    }

}

bool DesignScene::updateNodePos(){
    if(points->isEmpty()) return false;
    for(int iNode = 0; iNode < points->size(); iNode++){
        points->at(iNode)->setPos(points->at(iNode)->getPosition());
    }
    updateScene();
    return true;
}

void DesignScene::updateDisplayImg(){
    if(hasImgProcChange && !hasUpdateImgProcChange
            && (valueContrast != DEFAULT_CONTRAST || valueBrightness != DEFAULT_BRIGHTNESS || valueGama != DEFAULT_GAMMA)){
        dispImg = changeGamma(changeBrightness(changeContrast(*img,valueContrast),valueBrightness),valueGama);
        hasUpdateImgProcChange = true;
    }
}

void DesignScene::setRectRotate(QList<SpNode*> pts){
    rectRotate = new SpRectRotate(pts);
    hasRectRotate = true;
}

void DesignScene::resetRectRotate(){
    rectRotate = NULL;
    hasRectRotate = false;
}

void DesignScene::translatePts(QPointF tansVec, QList<QPointF> origPos, QList<SpNode*> nodelist, bool updateScenePos){
    for(int iPt = 0; iPt < nodelist.size(); iPt++){
        nodelist[iPt]->setPosition(origPos[iPt]+tansVec);
        if(updateScenePos) nodelist[iPt]->setPos(origPos[iPt]+tansVec);
    }
}

void DesignScene::translateSltPts(QPointF tansVec, SpNode* keyMovePt, QList<QPointF> origPos, QList<SpNode*> nodelist){
    QList<qreal> normdis = keyMovePt->normDis(keyMovePt->disToEach(origPos));

    for(int iPt = 0; iPt < nodelist.size(); iPt++){
        nodelist[iPt]->setPosition(normdis[iPt]*tansVec + origPos[iPt]);
        nodelist[iPt]->setPos(normdis[iPt]*tansVec + origPos[iPt]);
    }
}

bool DesignScene::setOrigPosDragList(QList<SpNode*> nodelist){
    if(nodelist.isEmpty()) return false;
    origPosDragList.clear();
    for(int iPt = 0; iPt < nodelist.size(); iPt++){
        origPosDragList.append(nodelist[iPt]->getPosition());
    }
    hasDragList = true;
    return true;
}

void DesignScene::addNode(SpNode* p){
    points->append(p);
    this->addItem(p);
}

void DesignScene::cuspPoints(){
    if(points->isEmpty()) return;
    for(QList<SpNode *>::iterator iNode = points->begin(); iNode!=points->end();iNode++){
        if(!(*iNode)->isSlted) continue;
        (*iNode)->getLef()->setPosition(QPointF(0,0));
        (*iNode)->getLef()->setPos(QPointF(0,0));
        (*iNode)->getRig()->setPosition(QPointF(0,0));
        (*iNode)->getRig()->setPos(QPointF(0,0));
    }
    update();
    updateScene();
}

void DesignScene::smoothPoints(){
    if(points->isEmpty()) return;
    for(QList<SpNode *>::iterator iNode = points->begin(); iNode!=points->end();iNode++){
        if(!(*iNode)->isSlted) continue;

        QPointF preLef(0,0);
        QPointF nextRig(0,0);
        qreal len = 0;
        qreal preLefLen = 0;
        qreal nextRigLen = 0;

        if((*iNode)->hasPreNode) {
            preLef = (*iNode)->getPreNode()->getLef()->scenePos();
            QVector2D lef((*iNode)->getPreNode()->getLef()->getPosition());
            preLefLen = lef.length();
        }
        else preLef = (*iNode)->scenePos();

        if((*iNode)->hasNextNode) {
            nextRig = (*iNode)->getNextNode()->getRig()->scenePos();
            QVector2D rig((*iNode)->getNextNode()->getRig()->getPosition());
            nextRigLen = rig.length();
        }
        else nextRig = (*iNode)->scenePos();

        QVector2D norVec(nextRig-preLef);

        if(preLefLen == 0 && nextRigLen == 0) len = DEFUALT_TANGENT_LENGTH;
        else len = (preLefLen + nextRigLen)/2;

#ifdef ENABLE_INCREASING_SMOOTH
        len = len * DEFUALT_INCREASING_RATIO;
#endif

        (*iNode)->getLef()->setPosition(norVec.normalized().toPointF()*len);
        (*iNode)->getLef()->setPos(norVec.normalized().toPointF()*len);
        (*iNode)->getRig()->setPosition(-norVec.normalized().toPointF()*len);
        (*iNode)->getRig()->setPos(-norVec.normalized().toPointF()*len);
    }
    update();
    updateScene();
}

void DesignScene::addNodeList(NodeList* nodelist){
    if(nodelist->size()>0){
        SpNode* tmpCurNode = nodelist->first();
        this->addNode(new SpNode(nodelist->first()));

        while(tmpCurNode->hasNextNode){
            if(tmpCurNode->getNextNode()->isFirstNode()){
                points->last()->setNextNode(points->first());
                points->first()->setPreNode(points->last());
                break;
            }
            else{
                SpNode* tmpNewNode = new SpNode(tmpCurNode->getNextNode());
                points->last()->setNextNode(tmpNewNode);
                tmpNewNode->setPreNode(points->last());
                this->addNode(tmpNewNode);
                tmpCurNode = tmpCurNode->getNextNode();
            }
        }
    }
}

void DesignScene::addCurve(SpCurve* c){
    curves.append(c);
    this->addItem(c);
}

void DesignScene::cleanFlag(){
    this->resetFlags();
    this->resetHasFlags();
}

void DesignScene::resetFlags(){
    flagMove=false;
    flagCreate=false;
    flagShowPoly=false;
    flagModPoint=false;
    flagDelete=false;
    flagDragPts=false;
    flagSolverDragPts = false;
    flagRotation=false;
    flagSelection=false;
}

void DesignScene::resetFlagsNotPoly(){
    flagMove=false;
    flagCreate=false;
    flagModPoint=false;
    flagDelete=false;
    flagDragPts=false;
    flagSolverDragPts = false;
    flagRotation=false;
    flagSelection=false;
}

void DesignScene::resetHasFlags(){
    hasPoly=false;
    hasSltRect=false;
    hasSltPts=false;
    hasSelectedPt=false;
    hasSelectedLeft=false;
    hasSelectedRight=false;
    hasDragList=false;
    hasAvaiPt=false;
    hasRectRotate=false;
}

void DesignScene::cleanNodes(){
    if(points->size()>0){
        for(QList<SpNode *>::iterator iNode = points->begin(); iNode!=points->end();iNode++){
            this->removeItem(*iNode);
        }
        points->clear();
    }
}

void DesignScene::cleanView(){
    this->clear();
    this->points->clear();
    this->curves.clear();
    this->resetDefaultFlags();
    this->updateScene();
    emit pushNoEdit();
}

void DesignScene::CenterPositionToSceneAllPts(){
    if(!points->isEmpty()){
        for(QList<SpNode *>::iterator iNode = points->begin(); iNode!=points->end();iNode++){
            (*iNode)->centerPositionToScene();
        }
    }
}

void DesignScene::cleanCurve(){
    if(curves.size()>0){
        for(QList<SpCurve *>::iterator iNode = curves.begin(); iNode!=curves.end();iNode++){
            this->removeItem(*iNode);
        }
        curves.clear();
    }

}

void DesignScene::rerenderPoints(QList<SpNode*> nodes){
    for(QList<SpNode *>::iterator iNode = nodes.begin(); iNode!=nodes.end();iNode++){
        (*iNode)->getLef()->setPos((*iNode)->getLef()->scenePos()-(*iNode)->getPosition());
        (*iNode)->getRig()->setPos((*iNode)->getRig()->scenePos()-(*iNode)->getPosition());
        (*iNode)->resetTransform();
    }
}

QVector<QPointF> DesignScene::turnSpNodeList(QList<SpNode*> nodes){
    QVector<QPointF> inpts;
    QListIterator<SpNode*> iter(nodes);
    while(iter.hasNext()){
        inpts.append(iter.next()->position);
    }
    return inpts;
}

void DesignScene::updateScene(){
    cleanCurve();
    if(points->size()>1){
        for(QList<SpNode *>::iterator iNode = points->begin(); iNode!=points->end();iNode++){
            if(!(*iNode)->hasNextNode) continue;
            SpNode *endPoint = (*iNode)->getNextNode();
            SpCurve *midCurve = new SpCurve((*iNode),endPoint,curveColor,scale);
            this ->addItem(midCurve);
            curves.append(midCurve);
        }
    }
    emit pushCurves(curves);
}

void DesignScene::rerenderScene(){
    this->clear();
    for(QList<SpNode *>::iterator iNode = points->begin(); iNode!=points->end();iNode++){
        if(points->size()>0) this ->addItem(*iNode);
        if(points->size()>1){
            if(!(*iNode)->hasNextNode) continue;
            SpNode *endPoint = (*iNode)->getNextNode();
            SpCurve *midCurve = new SpCurve((*iNode),endPoint,curveColor,scale);
            this ->addItem(midCurve);
            curves.append(midCurve);
        }
    }
    emit pushCurves(curves);
}

int DesignScene::getSelectedPt(){
    resetSelectedPt();
    //    qDebug() << "get in select";
    for(int i = 0; i < points->size(); i++) {

        if((*points)[i]->getLef()->distance(pos_click) <= (*points)[i]->getLef()->getSceneRadius()){
            sltPoint = (*points)[i];
            hasSelectedPt = true;
            hasSelectedLeft = true;
            hasSelectedRight = false;
            indSltPt = i;
//            qDebug() << "left tangent point is selected.";
//            qDebug() << (*points)[i]->getLef()->contains(pos_click-(*points)[i]->scenePos());
            return i;
        }else if((*points)[i]->getRig()->distance(pos_click) <= (*points)[i]->getRig()->getSceneRadius()){
            sltPoint = (*points)[i];
            hasSelectedPt = true;
            hasSelectedLeft = false;
            hasSelectedRight = true;
            indSltPt = i;
//            qDebug() << "right tangent point is selected.";
            return i;
        }else if((*points)[i]->distance(pos_click) <= (*points)[i]->getSceneRadius()) {
            sltPoint = (*points)[i];
            hasSelectedPt = true;
            hasSelectedLeft = false;
            hasSelectedRight = false;
            indSltPt = i;
            return i;
        }
    }
    resetSelectedPt();
    return -1;
}

int DesignScene::getSelectedPt(SpNode* node){
    resetSelectedPt();
    hasSelectedPt = true;
    sltPoint = node;
    int ind = points->indexOf(node);
    if(ind > -1) return ind;
    else return -1;
}

void DesignScene::resetSelectedPt(){
    hasSelectedPt = false;
    hasSelectedLeft = false;
    hasSelectedRight = false;
    indSltPt = -1;
    sltPoint = NULL;
}

void DesignScene::selectAllPts(){
    if(points->isEmpty()) return;
    for(QList<SpNode*>::iterator iNode = points->begin(); iNode!=points->end();iNode++){
        (*iNode)->slted();
    }
    setSltPts();
    update();
}

bool DesignScene::setSltPts(){
    sltPts.clear();
    for(QList<SpNode*>::iterator iNode = points->begin(); iNode!=points->end();iNode++){
        if((*iNode)->isSlted) sltPts.append(*iNode);
    }
    if(sltPts.isEmpty()){
        hasSltPts = false;
        return false;
    }
    else{
        hasSltPts = true;
        return true;
    }

}

void DesignScene::resetSltPts(){
    if(hasSltPts) {
        sltPts.clear();
        for(QList<SpNode*>::iterator iNode = points->begin(); iNode!=points->end();iNode++){
            (*iNode)->resetSlted();
            (*iNode)->update();
        }
        hasSltPts = false;
    }
}

void DesignScene::closeShape(){
    if(hasClosedCurve || points->size() < 2) return;
    points->first()->setPreNode(points->last());
    points->last()->setNextNode(points->first());
    hasClosedCurve = true;
}

void DesignScene::resetDragList(){
    if(hasDragList){
        origPosDragList.clear();
        hasDragList = false;
    }

}

void DesignScene::addAvaiPoint(SpNode* node){
    points->append(node);
    hasAvaiPt = true;
}

void DesignScene::setSltRect(QPointF topleft){
    sltRect = new SpRect(topleft);
    hasSltRect = true;
}
void DesignScene::resetSltRect(){
    if(hasSltRect) {
        this->removeItem(sltRect);
        sltRect = NULL;
        hasSltRect = false;
    }
}

bool DesignScene::callRotationMode(){
    flagRotation = true;
    if(!hasRectRotate&&!points->isEmpty()){
        setRectRotate(*points);
        rotator = new SpRotator(rectRotate,*points);
        this->addItem(rotator);
        return true;
    }
    return false;
}

bool DesignScene::resetCallRotationMode(){
    if(hasRectRotate){
        rotator->rmPtsFromRotator(rotator->getPoints());
        this->removeItem(rotator);
        resetRectRotate();
        this->destroyItemGroup(rotator);
        rerenderPoints(*points);
        flagRotation = false;
    }
    return true;
}

void DesignScene::setRotateMode(bool checked){
    if(checked&&!hasRectRotate&&!points->isEmpty()){
        setRectRotate(*points);
        rotator = new SpRotator(rectRotate,*points);
        this->addItem(rotator);
        flagRotation = true;
    }
    else if(hasRectRotate){
        rotator->rmPtsFromRotator(rotator->getPoints());
        this->removeItem(rotator);
        resetRectRotate();
        this->destroyItemGroup(rotator);
        rerenderPoints(*points);
        flagRotation = false;
    }
}

bool DesignScene::rszPtsAndLine(qreal s){
    if(s<0.8) scale = 0.8;
    else if(s>5) scale = 5;
    else scale = s;

    for(QList<SpNode *>::iterator iNode = points->begin(); iNode!=points->end();iNode++){
        (*iNode)->setScale(scale);
    }
    return true;
}

bool DesignScene::nodeNeighbouringByCurve(SpNode* node, QList<SpCurve*> curvesList){
    if(curvesList.isEmpty()) return false;
    qreal dis = 10000;
    SpCurve* cur = NULL;
    for(QList<SpCurve *>::iterator iNode = curvesList.begin(); iNode!=curvesList.end();iNode++){
        qreal diff = node->disToCurve((*iNode)->getCurvePt());
        if(diff>dis) continue;
        else{
            dis = diff;
            cur = *iNode;
        }
    }
    //    qDebug() << "distance to closest curve is: " << dis;
    //    qDebug() << "this curve has elements: " << cur->getCurve()->elementCount();

    QPointF newLeft = (cur->getPreNode()->getLef()->getPosition() + cur->getNextNode()->getLef()->getPosition())/2;
    QPointF newRight = (cur->getPreNode()->getRig()->getPosition() + cur->getNextNode()->getRig()->getPosition())/2;

    node->getLef()->setPosition(newLeft);
    node->getLef()->setPos(newLeft);
    node->getRig()->setPosition(newRight);
    node->getRig()->setPos(newRight);

    node->setPreNode(cur->getPreNode());
    cur->getPreNode()->setNextNode(node);
    node->setNextNode(cur->getNextNode());
    cur->getNextNode()->setPreNode(node);

    return true;
}

bool DesignScene::insertGlobalPt(SpNode* node){
    float disToPreNode = node->distance(node->getPreNode());
    float disToNextNode = node->distance(node->getNextNode());
    float factor = disToPreNode/(disToPreNode+disToNextNode);
    if(!node->hasPreNode) return false;
    int indPreNode = points->indexOf(node->getPreNode());
    emit pushGlobalPt(points->size()-1,indPreNode,factor);
    return true;
}

void DesignScene::resetDefaultFlags(){
    this->resetSelectedPt();
    this->resetDragList();
    this->resetCallRotationMode();
    this->resetSltPts();
    this->resetSltRect();

    flagMove=false;
    flagCreate=false;
    flagShowPoly=false;
    flagModPoint=false;
    flagDelete=false;
    flagDragPts=false;
    flagSolverDragPts=false;
    flagRotation=false;
    flagSelection=false;

    //    hasPoly=false;
    hasSltRect=false;
    hasSltPts=false;
    hasSelectedPt=false;
    hasSelectedLeft = false;
    hasSelectedRight = false;
    hasDragList=false;
    hasClosedCurve=false;
    hasAvaiPt=false;
    hasRectRotate=false;
}

bool DesignScene::showPolys(){
    if(!hasPoly){
        poly = new SpPoly(points,curveColor);
        this->addItem(poly);
        hasPoly = true;
        flagShowPoly = true;
        return true;
    }else return false;
}

bool DesignScene::unshowPolys(){
    if(hasPoly){
        this->removeItem(poly);
        poly = NULL;
        hasPoly = false;
        flagShowPoly = false;
        //        qDebug() << "unshow the polygon.";
        return true;
    }
    else return false;
}

//slot for showing polygon

void DesignScene::displayPoly(bool checked){
    if(curves.isEmpty()) return;
    if(checked && !flagShowPoly) {
        if(!hasPoly) {
            this->addItem(poly);
            poly->update();
        }
        hasPoly = true;
        flagShowPoly = true;
    }
    else if(!checked && flagShowPoly){
        if(hasPoly) this->removeItem(poly);
        hasPoly = false;
        flagShowPoly = false;
    }
}

//slots for control flag.

void DesignScene::setFlagMove(bool checked){
    if(checked) flagMove = true;
    else flagMove = false;
}

void DesignScene::setFlagSolverDragPts(bool checked){
    if(checked){
        resetFlagsNotPoly();
        flagSolverDragPts = true;
        qDebug() << "flagSolverDragPts is set.";
    }
    else {
        resetFlagsNotPoly();
        qDebug() << "flagSolverDragPts is unset.";
    }
}

void DesignScene::setFlagCreate(bool checked){
    if(checked){
        resetFlagsNotPoly();
        flagCreate = true;
        qDebug() << "flagCreate is set.";
    }
    else {
        resetFlagsNotPoly();
        qDebug() << "flagCreate is unset.";
    }
}

void DesignScene::setFlagShowPoly(bool checked){
    if(checked) flagShowPoly = true;
    else flagShowPoly = false;
}
void DesignScene::setFlagModPoint(bool checked){
    if(checked){
        resetFlagsNotPoly();
        flagModPoint = true;
    }
    else {
        resetFlagsNotPoly();
    }
}
void DesignScene::setFlagDelete(bool checked){
    if(checked){
        flagMove = false;
        flagCreate = false;
        flagModPoint = false;
        flagDelete = true;
        flagDragPts = false;
        flagSolverDragPts = false;
        flagRotation = false;
        flagSelection = false;
    }
    else {
        flagMove = false;
        flagCreate = false;
        flagModPoint = false;
        flagDelete = false;
        flagDragPts = false;
        flagSolverDragPts = false;
        flagRotation = false;
        flagSelection = false;
    }
}
void DesignScene::setFlagDragPts(bool checked){
    if(checked){
        flagMove = false;
        flagCreate = false;
        flagModPoint = false;
        flagDelete = false;
        flagDragPts = true;
        flagSolverDragPts = false;
        flagRotation = false;
        flagSelection = false;
    }
    else {
        flagMove = false;
        flagCreate = false;
        flagModPoint = false;
        flagDelete = false;
        flagDragPts = false;
        flagSolverDragPts = false;
        flagRotation = false;
        flagSelection = false;
    }
}
void DesignScene::setFlagRotation(bool checked){
    if(checked){
        flagMove = false;
        flagCreate = false;
        flagModPoint = false;
        flagDelete = false;
        flagDragPts = false;
        flagSolverDragPts = false;
        flagRotation = true;
        flagSelection = false;
    }
    else {
        flagMove = false;
        flagCreate = false;
        flagModPoint = false;
        flagDelete = false;
        flagDragPts = false;
        flagSolverDragPts = false;
        flagRotation = false;
        flagSelection = false;
    }
}
void DesignScene::setFlagSelection(bool checked){
    if(checked){
        flagMove = false;
        flagCreate = false;
        flagModPoint = false;
        flagDelete = false;
        flagDragPts = false;
        flagSolverDragPts = false;
        flagRotation = false;
        flagSelection = true;
//        qDebug() << "flagSelection is set.";
    }
    else {
        flagMove = false;
        flagCreate = false;
        flagModPoint = false;
        flagDelete = false;
        flagDragPts = false;
        flagSolverDragPts = false;
        flagRotation = false;
        flagSelection = false;
//        qDebug() << "flagSelection is unset.";
    }
}

void DesignScene::setCurveColor(QColor c){
    curveColor = c;
    if(curves.size()>0) updateScene();
}

void DesignScene::setStatus(EditType s){
    status = s;
}

bool DesignScene::checkCloseCurve(){
    if(points->size() < 2){
        hasClosedCurve = false;
        return false;
    }

    for(QList<SpNode *>::iterator iNode = points->begin(); iNode!=points->end();iNode++){
        if((*iNode)->isFirstNode() && (*iNode)->hasPreNode) {
            hasClosedCurve = true;
            return true;
        }
    }

    hasClosedCurve = false;
    return false;
}

bool DesignScene::checkInPtsArea(QList<SpNode*> pts, QPointF pos){
    if(pts.isEmpty()) return false;
    QList<QPointF> origPos;
    for(QList<SpNode *>::iterator iNode = pts.begin(); iNode!=pts.end();iNode++){
        origPos.append((*iNode)->getPosition());
    }
    QRectF rect = QPolygonF(origPos.toVector()).boundingRect();
    return rect.contains(pos);
}

void DesignScene::hidePoints(){

    if(points->isEmpty()){
        isPtsVisible = false;
        return;
    }

    for(QList<SpNode *>::iterator iNode = points->begin(); iNode!=points->end();iNode++)
        if((*iNode)->isVis) (*iNode)->setVis(false);

    update();
    isPtsVisible = false;
}

void DesignScene::showPoints(){
    if(points->isEmpty()){
        isPtsVisible = true;
        return;
    }

    for(QList<SpNode *>::iterator iNode = points->begin(); iNode!=points->end();iNode++)
        if(!(*iNode)->isVis) (*iNode)->setVis(true);

    update();
    isPtsVisible = true;
}
