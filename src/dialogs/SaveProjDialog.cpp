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

#include "include/dialogs/SaveProjDialog.hpp"

SaveProjDialog::SaveProjDialog(QString nProj, QString nCurv, QString dir)
{
    this->setWindowIcon(QIcon(":/images/poly.png"));

    projPathLabel = new QLabel(tr("Project Path:"));
    projPath = new QLineEdit();
    projPath->setText(dir);
    getPath = new QPushButton(tr("Path"));
    QObject::connect(getPath, SIGNAL(clicked()), this, SLOT(chooseProjFold()));

    nameProjLabel = new QLabel(tr("Project Name:"));
    nameProj = new QLineEdit();
    nameProj->setText(nProj);

    nameCurveLabel = new QLabel(tr("Curve Name:"));
    nameCur = new QLineEdit();
    nameCur->setText(nCurv);

    buttonBox = new QDialogButtonBox;

    okButton = new QPushButton(tr("Save"));
    connect(this->okButton, SIGNAL(clicked()), this, SLOT(accept()));
    buttonBox->addButton(this->okButton, QDialogButtonBox::AcceptRole);

    cancelButton = new QPushButton(tr("Cancel"));
    connect(this->cancelButton, SIGNAL(clicked()), this, SLOT(close()));
    buttonBox->addButton(this->cancelButton, QDialogButtonBox::RejectRole);

    buttonBox->setCenterButtons(true);

    nameProjLayout = new QHBoxLayout();
    nameProjLayout->addWidget(nameProjLabel);
    nameProjLayout->addWidget(nameProj);

    nameCurveLayout = new QHBoxLayout();
    nameCurveLayout->addWidget(nameCurveLabel);
    nameCurveLayout->addWidget(nameCur);

    pathLayout = new QHBoxLayout();
    pathLayout->addWidget(projPathLabel);
    pathLayout->addWidget(projPath);
    pathLayout->addWidget(getPath);

    formLayout = new QFormLayout();
    formLayout->addRow(nameProjLayout);
    formLayout->addRow(nameCurveLayout);
    formLayout->addRow(pathLayout);
    formLayout->addRow(buttonBox);

    this->setLayout(formLayout);
    this->setWindowTitle("Save As");
//    this->setFixedSize(400, 120);
    this->move(500,300);
}

void SaveProjDialog::chooseProjFold(){
        QString projectPath = QFileDialog::getExistingDirectory(this, tr("Open Directory"), QDir::homePath(),QFileDialog::ShowDirsOnly);
        projPath->setText(projectPath);
}
