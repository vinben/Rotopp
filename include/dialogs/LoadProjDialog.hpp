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

#ifndef LOADPROJDIALOG_H
#define LOADPROJDIALOG_H

#include <QObject>
#include <QWidget>
#include <QFile>
#include <QDialog>
#include <QDir>
#include <QFormLayout>
#include <QLineEdit>
#include <QLabel>
#include <QDialogButtonBox>
#include <QPushButton>
#include <QFileDialog>
#include <QTextStream>

class LoadProjDialog : public QDialog
{
    Q_OBJECT
private:
    QFormLayout *formLayout;
    QHBoxLayout *pathLayout;

    QLabel *projPathLabel;
    QLineEdit *nameProj;
    QLineEdit *nameCur;
    QLineEdit *nFrame;
    QLineEdit *nKeyframe;
    QLineEdit *projPath;
    QPushButton *getPath;
    QPushButton *okButton;
    QPushButton *cancelButton;
    QDialogButtonBox *buttonBox;
public:
    LoadProjDialog();
    QString getProjFile(){return projPath->text();}
    QString getNameProj(){return nameProj->text();}
    QString getNameCurve(){return nameCur->text();}
    QString getNFrame(){return nFrame->text();}
    QString getNKeyframe(){return nKeyframe->text();}
signals:

public slots:
    void chooseProjFile();
};

#endif // LOADPROJDIALOG_H
