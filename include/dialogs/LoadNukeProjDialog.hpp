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

#ifndef LOADNUKEPROJDIALOG_H
#define LOADNUKEPROJDIALOG_H

#include <QDialog>
#include <QLayout>
#include <QFormLayout>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QDebug>
#include <QComboBox>

class LoadNukeProjDialog : public QDialog
{
    Q_OBJECT

public:

    LoadNukeProjDialog();
//    void display();
    QString getFiles();
    QString getName(){return nameEdit->text();}
    QString getCurveName(){return curveNameEdit->text();}
    QString getProjPath(){return projPathEdit->text();}
    QString getNukeFileName(){return nukePathEdit->text();}
    bool hasSrcImgs(){return flagLoadImgs;}
    bool hasSrcVideo(){return flagLoadVideo;}

private slots:

    void chooseImgFold();
    void chooseProjFold();
    void chooseNukeFile();

private:

    bool flagLoadImgs;
    bool flagLoadVideo;

    QFormLayout *formLayout;
    QVBoxLayout *globeLayout;
    QHBoxLayout *videoLayout;
    QHBoxLayout *NukeFileLayout;
    QHBoxLayout *projPathLayout;
    QHBoxLayout *nameLayout;

    QLabel *pathLabel;
    QLabel *projPathLabel;
    QLabel *orLabel;
    QLabel *nukeLabel;

    QLineEdit *nameEdit;
    QLineEdit *curveNameEdit;
    QLineEdit *videoPathEdit;
    QLineEdit *projPathEdit;
    QLineEdit *nukePathEdit;

    QPushButton *projPathButton;
    QPushButton *loadVideoButton;
    QPushButton *loadImgButton;
    QPushButton *nukePathButton;
    QPushButton *okButton;
    QPushButton *cancelButton;
    QDialogButtonBox *buttonBox;

    QString projPath;
    QString srcVideo;
    QString srcImg;
    int fpsFrequence;
};

#endif // LOADNUKEPROJDIALOG_H
