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

#include "include/ConfigIO.hpp"

ConfigIO::ConfigIO(QString configName){
//    QDir iniDir = QFileInfo(configName).absoluteDir();
//    QFileInfo checkFile(iniDir.path());
    QFileInfo checkFile(QDir::currentPath() + "/" + configName);
    qDebug() << "config file:" << QDir::currentPath() + "/" + configName;
    isDebug = true;
    isSmrtBezier = true;
    isFillCurveMap = false;
    isShowPtFillCurveMap = true;
    lineWidthCurveMap = 10;
    isIniFile = checkFile.exists();

    if(isIniFile) {
//        configFilePath = iniDir.absoluteFilePath(configName);
        configFilePath = QDir::currentPath() + "/" + configName;
        this->readConfigs();
    }
//    qDebug() << "linewidght" << getLineWidthCurveMap();
}

void ConfigIO::readConfigs() {
    if(!isIniFile) return;
    libsPath.clear();
    QFile file(configFilePath);
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    QTextStream in(&file);
    while(!in.atEnd()) {
        QString tmp = in.readLine();
        QStringList frameList = tmp.split(" ");
        qDebug() << frameList.at(0) << frameList.at(1);
        if(frameList.at(0) == "RotoLib") libsPath.append(frameList.at(1));
        else if(frameList.at(0) == "RotoSolverType") solverType = frameList.at(1).toInt();
        else if(frameList.at(0) == "DebugOutput") {
            if(frameList.at(1) == "ON") isDebug = true;
            else if(frameList.at(1) == "OFF") isDebug = false;
        }
        else if(frameList.at(0) == "SmartBezier"){
            if(frameList.at(1) == "ON") isSmrtBezier = true;
            else if(frameList.at(1) == "OFF") isSmrtBezier = false;
        }
        else if(frameList.at(0) == "FilledCurveMap"){
            if(frameList.at(1) == "ON") isFillCurveMap = true;
            else if(frameList.at(1) == "OFF") isFillCurveMap = false;
        }
        else if(frameList.at(0) == "LineWidthCurveMap") lineWidthCurveMap = frameList.at(1).toInt();
        else if(frameList.at(0) == "ShowPtFilledCurveMap"){
            if(frameList.at(1) == "ON") isShowPtFillCurveMap = true;
            else if(frameList.at(1) == "OFF") isShowPtFillCurveMap = false;
        }
    }
}
















