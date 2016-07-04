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

#include "include/FrameFlow.hpp"

FrameFlow::FrameFlow(): QListWidget() {
    indCurScene = -1;
    nScene = 0;
    this->setFlow(QListView::LeftToRight);
    this->setFixedHeight(135);
    this->setMovement(QListView::Snap);
    this->setLayoutMode(QListView::Batched);
    this->setDragEnabled(false);
}

void FrameFlow::keyPressEvent(QKeyEvent * keyEvent){
    this->setFocus();
    if(keyEvent->modifiers() == Qt::KeyboardModifier::AltModifier
            && (keyEvent->key() == Qt::Key_Left
                || keyEvent->key() == Qt::Key_Z)){
        jumpToClosestKeyframe(getindCurScene(), false);
        qDebug() << "jump to previous keyframe";
    }
    else if(keyEvent->modifiers() == Qt::KeyboardModifier::AltModifier
            && (keyEvent->key() == Qt::Key_Right
                || keyEvent->key() == Qt::Key_X)){
        jumpToClosestKeyframe(getindCurScene(), true);
        qDebug() << "jump to next keyframe";
    }
    else if(keyEvent->key()==Qt::Key_Left || keyEvent->key()==Qt::Key_Z){
        setPreFrame();
    }
    else if(keyEvent->key()==Qt::Key_Right || keyEvent->key()== Qt::Key_X){
        setNextFrame();
    }
    else if(keyEvent->key()==Qt::Key_1){
        zoomin(0.93);
    }
    else if(keyEvent->key()==Qt::Key_2){
        zoomin(1.07);
    }
}

void FrameFlow::setSceneConnection(DesignView * v){
    for(QList<ImgIcon *>::iterator iNode = icons.begin(); iNode!=icons.end();iNode++){
        QObject::connect(*iNode,SIGNAL(pushImgIconClicked(qreal)),v,SLOT(changeScene(qreal)));
    }
}

void FrameFlow::setIconConnection() {
    if(icons.isEmpty()) return;
    for (QList<ImgIcon *>::iterator iNode = icons.begin(); iNode != icons.end(); iNode++) {
        QObject::connect((*iNode), SIGNAL(pushImgIconClicked(qreal)), this, SLOT(updateCurScene(qreal)));
        //        QObject::connect((*iNode),SIGNAL(pushRecheckDesignActions()),this,SLOT(reCheckDesignActions()));
//        QObject::connect((*iNode), SIGNAL(pushImgIconClicked(qreal)), statStack, SLOT(pushStatus()));
    }
}

void FrameFlow::setDesignActionsConnnection(){
    emit pushImgIcons(icons);
}

void FrameFlow::addImgIcon(QImage* im){
    QListWidgetItem *item = new QListWidgetItem(this);
    item->setSizeHint(QSize(160,110));
    item->setText(QString::number(icons.size()+1));
    item->setTextAlignment(Qt::AlignBottom | Qt::AlignHCenter);
    this->addItem(item);

    ImgIcon *icon = new ImgIcon(im,icons.size());
    icon->setMaximumSize(160,90);
    icon->setScaledContents(true);
    this->setItemWidget(item,icon);
    icons.append(icon);
}

void FrameFlow::initFrameFlow(QList<QImage*> imgs){
    this->clear();
    icons.clear();
    for(QList<QImage *>::iterator iNode = imgs.begin(); iNode!=imgs.end();iNode++){
        this->addImgIcon(*iNode);
    }
    nScene = icons.size();
    this->setIconConnection();
}

void FrameFlow::updateCurScene(qreal ind){
    if(ind>-1 && ind < nScene){
        indCurScene = ind;
    }
}

void FrameFlow::setNextFrame(){
    if(indCurScene<nScene-1){
        this->setCurrentRow(indCurScene+1);
        icons[indCurScene+1]->designActionClick();
//        QListWidgetItem* cur = this->item(indCurScene+1);
    }
}

void FrameFlow::setPreFrame(){
    if(indCurScene>0){
        this->setCurrentRow(indCurScene-1);
        icons[indCurScene-1]->designActionClick();
    }
}

//void FrameFlow::setCurFrame(int ind){
//    if(ind > -1 && ind < nScene) indCurScene = ind;
//}

void FrameFlow::resetImgIconStatus(){
    if(icons.size()>0){
        for(QList<ImgIcon*>::iterator iNode = icons.begin(); iNode!=icons.end();iNode++){
            (*iNode)->setStatus(EditType::NOEDIT);
        }
    }
}

EditType FrameFlow::getEditTypeCurIcon(){
    return icons.at(indCurScene)->status;
}

void FrameFlow::labelKeyframeForCurScene(){
    icons[indCurScene]->setStatus(EditType::KEYFRAME);
}

QList<int> FrameFlow::getIndKeyframes(){
    QList<int> ind;
    for(int iScene = 0; iScene < nScene; iScene++){
        if(icons[iScene]->status == EditType::KEYFRAME)
            ind.append(iScene);
    }
    return ind;
}

QList<int> FrameFlow::getIndEditing(){
    QList<int> ind;
    for(int iScene = 0; iScene < nScene; iScene++){
        if(icons[iScene]->status == EditType::EDITING)
            ind.append(iScene);
    }
    return ind;
}

void FrameFlow::setIndKeyframes(QList<int> keys){
    this->resetImgIconStatus();
    if(keys.size()>0){
        for(QList<int>::iterator iNode = keys.begin(); iNode!=keys.end();iNode++){
            this->icons[*iNode]->setKeyFrameStatus();
        }
    }
}

void FrameFlow::setNextEdit(QList<int> list){
    if(list.isEmpty()) return;
    for(int iScene = 0; iScene < list.size(); iScene++){
        icons.at(list[iScene])->setStatus(EditType::NEXTEDIT);
    }
}

StatusList FrameFlow::getStatusList(){
    StatusList list;
    for(int iScene = 0; iScene < nScene; iScene++){
        list.append(icons[iScene]->status);
    }
    return list;
}

void FrameFlow::setAllStatus(StatusList states){
    this->resetImgIconStatus();
    if(states.size()==this->nScene && nScene > 0){
        for(int iScene = 0; iScene < nScene; iScene++){
            icons[iScene]->setStatus(states[iScene]);
        }
    }
}

void FrameFlow::setAllSuggestOnNotKeyframes(){
    for(int iScene = 0; iScene < nScene; iScene++){
        if(icons[iScene]->status!=EditType::KEYFRAME){
            icons[iScene]->setStatus(EditType::SUGGESTED);
        }
    }
}

void FrameFlow::setStatusCurIcon(EditType t){
    icons[indCurScene]->setStatus(t);
}

void FrameFlow::zoomin(float factor){
    if(icons.isEmpty()) return;
    int nItem = icons.size();
    QSize frameSize = this->frameSize();
    QSize cur = this->item(0)->sizeHint();

    if(cur.width()*factor > 160){
        for(int i = 0; i < nItem; i++){
            this->item(i)->setSizeHint(QSize(160,110));
        }
        return;
    }
    else if(cur.width()*factor < 9){
        for(int i = 0; i < nItem; i++){
            this->item(i)->setSizeHint(QSize(10,110));
        }
        return;
    }else if(cur.width()*factor*nItem < frameSize.width() && cur.width()*factor >= 9){
        for(int i = 0; i < nItem; i++){
            this->item(i)->setSizeHint(QSize(frameSize.width()/nItem,110));
        }
        return;
    }
    else {
        for(int i = 0; i < nItem; i++){
            QSize step = cur*factor;
            if(step.height()>110)
                step.setHeight(110);
            this->item(i)->setSizeHint(step);
        }
    }
    update();
}

void FrameFlow::jumpToClosestKeyframe(int indScene, bool fwd){
    QList<int> keys = this->getIndKeyframes();
    int indLast = this->getIcons().size()-1;
    if(keys.isEmpty()){
        if(fwd && indLast > -1 && indLast != this->getindCurScene()){
            this->getIcons().at(indLast)->pushImgIconClicked(indLast);
            this->setCurrentRow(indLast);
        }else if(!fwd && this->getindCurScene() != 0){
            this->getIcons().at(0)->pushImgIconClicked(0);
            this->setCurrentRow(0);
        }
    }
    int ind = -1;
    if(fwd){
        ind = this->getIndClosestKeyframe(indScene,fwd);
        if(ind < 0 && ind != this->getindCurScene()){
            this->getIcons().at(indLast)->pushImgIconClicked(indLast);
            this->setCurrentRow(indLast);
        }else if(ind != this->getindCurScene()){
            this->getIcons().at(ind)->pushImgIconClicked(ind);
            this->setCurrentRow(ind);
        }
    }else{
        ind = this->getIndClosestKeyframe(indScene,fwd);
        if(ind < 0 && ind != this->getindCurScene()){
            this->getIcons().at(0)->pushImgIconClicked(0);
            this->setCurrentRow(0);
        }else if(ind != this->getindCurScene()){
            this->getIcons().at(ind)->pushImgIconClicked(ind);
            this->setCurrentRow(ind);
        }
    }
}

int FrameFlow::getIndClosestKeyframe(int ind, bool fwd) {
    int tmp = -1;
    QList<int> indKeys = this->getIndKeyframes();
    if(fwd){
        for(int i = indKeys.size()-1; i > -1; i --){
            if(indKeys[i]>ind) tmp = indKeys[i];
        }
    }else{
        for(int i = 0; i < indKeys.size(); i ++){
            if(indKeys[i]<ind) tmp = indKeys[i];
        }
    }
    return tmp;
}

void FrameFlow::setKeyframeAll(){
    if(icons.isEmpty()) return;
    for (QList<ImgIcon *>::iterator iNode = icons.begin(); iNode != icons.end(); iNode++) {
        (*iNode)->setKeyFrameStatus();
    }
}
