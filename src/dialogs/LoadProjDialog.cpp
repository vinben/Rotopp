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

#include "include/dialogs/LoadProjDialog.hpp"

LoadProjDialog::LoadProjDialog()
{
    this->setWindowIcon(QIcon(":/images/poly.png"));

    projPathLabel = new QLabel(tr("Project File:"));
    projPath = new QLineEdit();
    getPath = new QPushButton(tr("File"));
    QObject::connect(getPath, SIGNAL(clicked()), this, SLOT(chooseProjFile()));

    nameProj = new QLineEdit();
    nameProj->setEnabled(false);
    nameProj->setAlignment(Qt::AlignCenter);

    nameCur = new QLineEdit();
    nameCur->setEnabled(false);
    nameCur->setAlignment(Qt::AlignCenter);

    nFrame = new QLineEdit();
    nFrame->setEnabled(false);
    nFrame->setAlignment(Qt::AlignCenter);

    nKeyframe = new QLineEdit();
    nKeyframe->setEnabled(false);
    nKeyframe->setAlignment(Qt::AlignCenter);

    buttonBox = new QDialogButtonBox();

    okButton = new QPushButton(tr("Load"));
    connect(this->okButton, SIGNAL(clicked()), this, SLOT(accept()));
    buttonBox->addButton(this->okButton, QDialogButtonBox::AcceptRole);

    cancelButton = new QPushButton(tr("Cancel"));
    connect(this->cancelButton, SIGNAL(clicked()), this, SLOT(close()));
    buttonBox->addButton(this->cancelButton, QDialogButtonBox::RejectRole);

    buttonBox->setCenterButtons(true);

    QGridLayout *gridLayout = new QGridLayout;
    gridLayout->addWidget(new QLabel(tr("Project Name:")),0,0);
    gridLayout->addWidget(nameProj,0,1);
    gridLayout->addWidget(new QLabel(tr("Curve Name:")),0,2);
    gridLayout->addWidget(nameCur,0,3);

    gridLayout->addWidget(new QLabel(tr("Keyframe No.:")),1,0);
    gridLayout->addWidget(nKeyframe,1,1);
    gridLayout->addWidget(new QLabel(tr("Frame No.:")),1,2);
    gridLayout->addWidget(nFrame,1,3);


    pathLayout = new QHBoxLayout();
    pathLayout->addWidget(projPathLabel);
    pathLayout->addWidget(projPath);
    pathLayout->addWidget(getPath);

    formLayout = new QFormLayout();
    formLayout->addRow(pathLayout);
    formLayout->addRow(gridLayout);
    formLayout->addRow(buttonBox);

    this->setLayout(formLayout);
    this->setWindowTitle("Load Project");
//    this->setFixedSize(400, 120);
    this->move(500,300);
}

void LoadProjDialog::chooseProjFile(){
    QString projectFilePath = QFileDialog::getOpenFileName(this, tr("Open Project File"),QDir::homePath(),tr("Roto++ File (*.roto *.rotoAutoSave)"));
    if(!projectFilePath.isEmpty()){
        projPath->setText(projectFilePath);
        QFile file(projectFilePath);
        file.open(QIODevice::ReadOnly | QIODevice::Text);
        QTextStream in(&file);
        nameProj->setText(in.readLine());
        nameCur->setText(in.readLine());
        QString saveTime = in.readLine();
        QString tmp = in.readLine();
        if(!tmp.isNull()){
            QStringList frameList = tmp.split(" ");
            if(frameList.at(1).toInt()>=frameList.at(2).toInt()){
                nFrame->setText(frameList.at(1));
                nKeyframe->setText(frameList.at(2));
                projPath->setEnabled(false);
            }
        }
    }
}






