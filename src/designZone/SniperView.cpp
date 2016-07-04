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

#include "include/designZone/SniperView.hpp"

SniperView::SniperView(QWidget* parent): QGraphicsView(parent){
    wid = 200;
    ptScale = 10;
    curScene = new DesignScene();
    bacisSettings(wid);
    hasInit = false;
}

SniperView::SniperView(QImage* im, qreal w, qreal s)
{
    this->initView(im,w,s);
}

void SniperView::bacisSettings(qreal w){
    this->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->setInteractive(false);
    this->setMinimumSize(QSize(w, w));
    this->setMaximumSize(QSize(w, w));
    this->setRenderHints(QPainter::Antialiasing|QPainter::HighQualityAntialiasing|QPainter::NonCosmeticDefaultPen|QPainter::TextAntialiasing);
    this->setCacheMode(QGraphicsView::CacheNone);
    this->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
}

void SniperView::initView(QImage* im, qreal w, qreal s){
    wid = w;
    img = im;
    ptScale = s;
    cross = initCross(wid);
    setCurScene(new DesignScene(img));
    curScene->addItem(cross);
    this->scale(ptScale,ptScale);
    this->centerOn(QPoint(0,0));
    bacisSettings(wid);
    hasInit = true;
    qDebug() << "sniperview is initilised!";
}

QGraphicsItemGroup* SniperView::initCross(qreal w){
    QGraphicsItemGroup* list = new QGraphicsItemGroup();
    QGraphicsLineItem* lineup = new QGraphicsLineItem(QLineF(-w,0,w,0));
    QGraphicsLineItem* linedw = new QGraphicsLineItem(QLineF(0,-w,0,w));
    lineup->setPen(QPen(QColor(Qt::red),2));
    linedw->setPen(QPen(QColor(Qt::red),2));
    list->addToGroup(lineup);
    list->addToGroup(linedw);
    list->setFlag(QGraphicsItem::ItemIgnoresTransformations);
    return list;
}

void SniperView::setCenter(QPointF cen){
    focus_scene = cen;
    this->centerOn(cen);
    cross->setPos(cen);
    curScene->update();
}

void SniperView::initScenes(QList<QImage*> im){
    scenes.clear();
    for(int iImg = 0; iImg<im.size();iImg++){
        scenes.append(new DesignScene(im[iImg]));
    }
    qDebug() << "scenes initialization done!";
}

void SniperView::changeScene(qreal ind){
    this->setCurScene(scenes[ind]);
}

void SniperView::setCurScene(DesignScene* s){
    curScene=s;
    this->setScene(curScene);
}

void SniperView::changeImg(QImage* im){
    if(hasInit){
        curScene->setImg(im);
        curScene->update();
    }
    else initView(im);
}

void SniperView::wheelEvent(QWheelEvent* event){
    qreal zoomFactor = 0;
    qreal zoomInFactor = 1.2;
    qreal zoomOutFactor = 1 / zoomInFactor;

    this->setTransformationAnchor(QGraphicsView::NoAnchor);
    this->setResizeAnchor(QGraphicsView::NoAnchor);

    if(event->delta() > 0) zoomFactor = zoomInFactor;
    else zoomFactor = zoomOutFactor;

    qreal tarScale = this->transform().m11() * zoomFactor;
    if(tarScale*img->width() >= wid && tarScale*img->height() >= wid){
        this->translate(focus_scene.x(),focus_scene.y());
        this->scale(zoomFactor,zoomFactor);
        this->translate(-focus_scene.x(),-focus_scene.y());

    }
//    qDebug() << "scale factor: " << this->transform().m11() << " " << this->transform().m12();
}

