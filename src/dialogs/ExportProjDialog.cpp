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

#include "include/dialogs/ExportProjDialog.hpp"

ExportProjDialog::ExportProjDialog(QString defaultName, QString defaultDir) {
    this->setWindowIcon(QIcon(":/images/poly.png"));
    flagTrans = false;

    // file name
    exportFileNameLable = new QLabel(tr("File Name:"));
    exportFileNameEdit = new QLineEdit();
    exportFileNameEdit->setText(defaultName);

    // check box
    transCheck = new QCheckBox(tr("Transpose"));
    QObject::connect(transCheck, SIGNAL(toggled(bool)), this, SLOT(setTranspose(bool)));

    // file path
    exportFilePathLabel = new QLabel(tr("File Path:"));
    exportFilePathEdit = new QLineEdit();
    exportFilePathEdit->setText(defaultDir);

    getPath = new QPushButton(tr("Path"));
    QObject::connect(getPath, SIGNAL(clicked()), this, SLOT(chooseOutputFolder()));

    // buttons
    buttonBox = new QDialogButtonBox;

    okButton = new QPushButton(tr("Export"));
    connect(this->okButton, SIGNAL(clicked()), this, SLOT(accept()));
    buttonBox->addButton(this->okButton, QDialogButtonBox::AcceptRole);

    cancelButton = new QPushButton(tr("Cancel"));
    connect(this->cancelButton, SIGNAL(clicked()), this, SLOT(close()));
    buttonBox->addButton(this->cancelButton, QDialogButtonBox::RejectRole);

    buttonBox->setCenterButtons(true);

    // layout

    exportFileNameLayout = new QHBoxLayout();
    exportFileNameLayout->addWidget(exportFileNameLable);
    exportFileNameLayout->addWidget(exportFileNameEdit);
    exportFileNameLayout->addWidget(transCheck);

    exportFilepathLayout = new QHBoxLayout();
    exportFilepathLayout->addWidget(exportFilePathLabel);
    exportFilepathLayout->addWidget(exportFilePathEdit);
    exportFilepathLayout->addWidget(getPath);

    formLayout = new QFormLayout();
    formLayout->addRow(exportFileNameLayout);
    formLayout->addRow(exportFilepathLayout);
    formLayout->addRow(buttonBox);

    this->setLayout(formLayout);
    this->setWindowTitle("Export Project To");
    this->move(500,300);
}

void ExportProjDialog::chooseOutputFolder(){
        QString projectPath = QFileDialog::getExistingDirectory(this, tr("Open Directory"), QDir::homePath(),QFileDialog::ShowDirsOnly);
        exportFileNameEdit->setText(projectPath);
}

void ExportProjDialog::setTranspose(bool checked){
    if(checked) flagTrans = true;
    else flagTrans = false;
}

bool ExportProjDialog::isTrans(){
    return flagTrans;
}



