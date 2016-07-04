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

#ifndef ROTOIO_H
#define ROTOIO_H

//this is the IO interface for the tool.

#include <QObject>
#include <QDir>
#include <Eigen/Core>
#include <QDirIterator>
#include <include/utils.hpp>
#include <include/designZone/DesignView.hpp>
#include <include/FrameFlow.hpp>
#include <include/RotoStatus.hpp>
#include <include/RotoCore.hpp>
#include <include/ConfigIO.hpp>


class RotoIO : public QObject {
    Q_OBJECT
private:
    DesignView* view;
    FrameFlow* flowList;
    RotoStatus* statStack;
    ImageList* imgs;
    RotoCore* rotocore;

    ConfigIO* config;

    QString nameOfProject;
    QString nameOfCurve;
    QString projectPath;


//    Access methods
    bool checkStringList(QStringList,int);
    bool checkStringList(QString,int);
    bool checkImgPath(QString,int);
    bool checkImgFiles(QString,QString);
    bool checkAlphaMapPath(QString,QString,QString,int);
    bool checkAlphaMapFiles(QString,QString,QString);

    QDir getImgPath(QString);
    void deleteRotoFiles(QString);

public:
    RotoIO(DesignView*,FrameFlow*,RotoStatus*,ImageList*,RotoCore*);
    bool checkRotoFileExist(QString, QString, QString);
    bool checkRotoExportFileExist(QString, QString);
    void EigenMatrixToFile(const Eigen::MatrixXd &, QTextStream&);
    QDir getAlphaMapPath(QString);
    QDir getCurveMapPath(QString);
    void deleteAlphaMapFiles(QString,QString,QString);

signals:
    void pushImgs();
    void pushSolverType(SolverType);
public slots:
//    Main IO for the tool.
    void saveRoto(QString, QString, QString, SolverType, bool flag=false);
    void saveAlphaMap(QString, QString, QString, bool);
    void saveCurveMap(QString, QString, QString);
    bool loadRoto(QString);
    bool loadNukeExport(QString);
    void autoSave(SolverType);
    void autoExportLog();
    bool exportRoto(QString, bool);
//    void importRoto();
    void saveLog();
};

#endif // ROTOIO_H
