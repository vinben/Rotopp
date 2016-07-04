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

#include "include/dialogs/CreateProjDialog.hpp"
#include <iostream>

CreateProjDialog::CreateProjDialog() : QDialog()
{
    this->setWindowIcon(QIcon(":/images/poly.png"));
    flagLoadImgs = false;
    flagLoadVideo = false;
    formLayout = new QFormLayout;

    pathLabel = new QLabel(tr("File Path: "));
    projPathLabel = new QLabel(tr("Project Path: "));
    orLabel = new QLabel(tr("or"));

    nameEdit = new QLineEdit;
    nameEdit->setText(tr("NewProj"));
    curveNameEdit = new QLineEdit;
    curveNameEdit->setText(tr("NewCurve"));

    projPathEdit = new QLineEdit;
    projPathEdit->setText(QDir::homePath());
    projPathEdit->setEnabled(false);
    videoPathEdit = new QLineEdit;
    videoPathEdit->setEnabled(false);

    projPathButton = new QPushButton(tr("Path"));
    loadVideoButton = new QPushButton(tr("Video"));
    loadImgButton = new QPushButton(tr("Images"));
    QObject::connect(projPathButton, SIGNAL(clicked()), this, SLOT(chooseProjFold()));
    QObject::connect(loadVideoButton, SIGNAL(clicked()), this, SLOT(chooseVideo()));
    QObject::connect(loadImgButton, SIGNAL(clicked()), this, SLOT(chooseImgFold()));

    buttonBox = new QDialogButtonBox;

    okButton = new QPushButton(tr("Create"));
    connect(this->okButton, SIGNAL(clicked()), this, SLOT(accept()));
    buttonBox->addButton(this->okButton, QDialogButtonBox::AcceptRole);

    cancelButton = new QPushButton(tr("Cancel"));
    connect(this->cancelButton, SIGNAL(clicked()), this, SLOT(close()));
    buttonBox->addButton(this->cancelButton, QDialogButtonBox::RejectRole);

    buttonBox->setCenterButtons(true);

    fps = new QComboBox;
    fps->addItem("6");
    fps->addItem("8");
    fps->addItem("12");
    fps->addItem("24");
    fps->setEnabled(false);

    videoLayout = new QHBoxLayout;
    videoLayout->addWidget(pathLabel);
    videoLayout->addWidget(videoPathEdit);
    videoLayout->addWidget(loadVideoButton);
    videoLayout->addWidget(orLabel);
    videoLayout->addWidget(loadImgButton);

    nameLayout = new QHBoxLayout;
    nameLayout->addWidget(new QLabel(tr("Project Name: ")));
    nameLayout->addWidget(nameEdit);
    nameLayout->addSpacing(20);
    nameLayout->addWidget(new QLabel(tr("Curve Name: ")));
    nameLayout->addWidget(curveNameEdit);

    projPathLayout = new QHBoxLayout;
    projPathLayout->addWidget(projPathLabel);
    projPathLayout->addWidget(projPathEdit);
    projPathLayout->addWidget(projPathButton);


    formLayout->addRow(nameLayout);
    formLayout->addRow(projPathLayout);
    formLayout->addRow(videoLayout);
    formLayout->addRow(tr("Number of Frame per Second"), fps);
    formLayout->addRow(buttonBox);

    this->setLayout(formLayout);
    this->setWindowTitle("New Project");
//    this->setFixedSize(500, 150);
    this->move(500,300);
}


void CreateProjDialog::chooseVideo(){
    srcVideo = QFileDialog::getOpenFileName(this, tr("Open Source video"),QDir::homePath());
    videoPathEdit->setText(srcVideo);
    flagLoadImgs = false;
    flagLoadVideo = true;
    fps->setEnabled(true);
//    qDebug() << srcVideo;
}

void CreateProjDialog::chooseImgFold(){
    srcImg = QFileDialog::getExistingDirectory(this, tr("Open Directory"), QDir::homePath(),QFileDialog::ShowDirsOnly);
    videoPathEdit->setText(srcImg);
    flagLoadImgs = true;
    flagLoadVideo = false;
    fps->setEnabled(false);
}

void CreateProjDialog::chooseProjFold(){
    projPath = QFileDialog::getExistingDirectory(this, tr("Open Directory"), QDir::homePath(),QFileDialog::ShowDirsOnly);
    projPathEdit->setText(projPath);
    projPathEdit->setEnabled(false);
}

//void CreateProjDialog::display(){
//    this->setWindowModality(Qt::ApplicationModal);
//    this->exec();
//}

QString CreateProjDialog::getFiles(){
    QString tmp;
    if(hasSrcVideo()&& !hasSrcImgs()) return srcVideo;
    else if(!hasSrcVideo()&& hasSrcImgs()) return srcImg;
    else return tmp;
}


