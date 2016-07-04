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

#include "include/RotoCore.hpp"

RotoCore::RotoCore(DesignView *v, FrameFlow *fl, ImageList *il) : view(v), flowList(fl), imgs(il) {

}

QPointF RotoCore::getOffset() {
    QPointF of(0, 0);
    if (imgs->isEmpty()) return of;
    else return QPointF(imgs->first()->width() / 2, imgs->first()->height() / 2);
}

QtKeyFrames RotoCore::normQtKeyframes() {
    QtKeyFrames keyframes;
    QPointF off = getOffset();
    DesignSceneList *scenes = view->getScenes();
    QList<int> indKeys = flowList->getIndKeyframes();

    for (int iKey = 0; iKey < indKeys.size(); iKey++) {
        NodeList *nodes = scenes->at(indKeys.at(iKey))->getPoints();
        QtNodeList normNodeList;
        for (QList<SpNode *>::iterator iNode = nodes->begin(); iNode != nodes->end(); iNode++) {
            normNodeList.append((*iNode)->getPosition() + off);
        }
        keyframes[indKeys.at(iKey)] = normNodeList;
    }
    return keyframes;
}

VecKeyFrames RotoCore::normVecKeyframes() {
    VecKeyFrames keyframes;
    QPointF off = getOffset();
    DesignSceneList *scenes = view->getScenes();
    QList<int> indKeys = flowList->getIndKeyframes();

    for (int iKey = 0; iKey < indKeys.size(); iKey++) {
        NodeList *nodes = scenes->at(indKeys.at(iKey))->getPoints();
        VecNodeList normNodeList;
        for (QList<SpNode *>::iterator iNode = nodes->begin(); iNode != nodes->end(); iNode++) {
            VecNode node;
            node.append((*iNode)->getPosition() + off);
            node.append((*iNode)->leftToScene() + off);
            node.append((*iNode)->rightToScene() + off);
            normNodeList.append(node);
        }
        keyframes[indKeys.at(iKey)] = normNodeList;
    }
    return keyframes;
}

VecNodeList RotoCore::normVecFrame(int ind) {
    QPointF off = getOffset();
    NodeList *nodes = view->getScenes()->at(ind)->getPoints();
    VecNodeList out;
    for (QList<SpNode *>::iterator iNode = nodes->begin(); iNode != nodes->end(); iNode++) {
        VecNode node;
        node.append((*iNode)->getPosition() + off);
        node.append((*iNode)->leftToScene() + off);
        node.append((*iNode)->rightToScene() + off);
        out.append(node);
    }
    return out;
}

Eigen::MatrixXd RotoCore::normEigenFrame(int ind) {
    VecNodeList normVec = normVecFrame(ind);
    QList<VecNodeList> normVecList;
    normVecList.append(normVec);
    return VecNodeListToEigenMatrix(normVecList);
}

Eigen::MatrixXd RotoCore::normEigenAllframes(QList<int> index) {
    QPointF off = getOffset();
    DesignSceneList *scenes = view->getScenes();
    int sceneSize = scenes->size();
    QList<int> indKeys = flowList->getIndKeyframes();
    int cols = scenes->at(0)->getPoints()->size() * 6;
    Eigen::MatrixXd output(sceneSize, cols);
    for (int iRow = 0; iRow < sceneSize; iRow++) {
        NodeList *nodes = scenes->at(iRow)->getPoints();
        for (int iNode = 0; iNode < nodes->size(); iNode++) {
            if (index.contains(iRow) || indKeys.contains(iRow)) {
                QPointF center = ((*nodes)[iNode]->getPosition() + off);
                QPointF left = ((*nodes)[iNode]->leftToScene() + off);
                QPointF right = ((*nodes)[iNode]->rightToScene() + off);
                output(iRow, iNode * 6 + 0) = center.x();
                output(iRow, iNode * 6 + 1) = center.y();
                output(iRow, iNode * 6 + 2) = left.x();
                output(iRow, iNode * 6 + 3) = left.y();
                output(iRow, iNode * 6 + 4) = right.x();
                output(iRow, iNode * 6 + 5) = right.y();
            } else {
                output(iRow, iNode * 6 + 0) = -1;
                output(iRow, iNode * 6 + 1) = -1;
                output(iRow, iNode * 6 + 2) = -1;
                output(iRow, iNode * 6 + 3) = -1;
                output(iRow, iNode * 6 + 4) = -1;
                output(iRow, iNode * 6 + 5) = -1;
            }

        }
    }
    qDebug() << "the eigen matrix for the points of a frame: ";
    cout << output << endl;
    return output;
}

Eigen::MatrixXd RotoCore::normEigenKeyframes() {

    QPointF off = getOffset();
    DesignSceneList *scenes = view->getScenes();
    QList<int> indKeys = flowList->getIndKeyframes();
    int cols = scenes->at(indKeys.at(0))->getPoints()->size() * 6;
    Eigen::MatrixXd keyframes(indKeys.size(), cols);
    for (int iKey = 0; iKey < indKeys.size(); iKey++) {
        NodeList *nodes = scenes->at(indKeys.at(iKey))->getPoints();
        for (int iNode = 0; iNode < nodes->size(); iNode++) {

            QPointF center = ((*nodes)[iNode]->getPosition() + off);
            QPointF left = ((*nodes)[iNode]->leftToScene() + off);
            QPointF right = ((*nodes)[iNode]->rightToScene() + off);
            keyframes(iKey, iNode * 6 + 0) = center.x();
            keyframes(iKey, iNode * 6 + 1) = center.y();
            keyframes(iKey, iNode * 6 + 2) = left.x();
            keyframes(iKey, iNode * 6 + 3) = left.y();
            keyframes(iKey, iNode * 6 + 4) = right.x();
            keyframes(iKey, iNode * 6 + 5) = right.y();
        }
    }
//    qDebug() << "the eigen matrix for the points of keyframes: ";
//    cout << keyframes << endl;
    return keyframes;
}


QtNodeList RotoCore::unNormQtNodeList(QtNodeList nodes) {
    QPointF off = getOffset();
    QtNodeList out;
    if (!nodes.isEmpty()) {
        for (QtNodeList::iterator iNode = nodes.begin(); iNode != nodes.end(); iNode++) {
            out.append(*iNode - off);
        }
    }
    return out;
}

QList<VecNodeList> RotoCore::EigenMatrixToVecNodeList(Eigen::MatrixXd nodeList) {
    int nFrame = nodeList.rows();
    int nNode = nodeList.cols() / 6;
    QList<VecNodeList> vecList;
    for (int iFrame = 0; iFrame < nFrame; iFrame++) {
        VecNodeList nodes;
        for (int iNode = 0; iNode < nNode; iNode++) {
            VecNode node;
            node.append(QPointF(nodeList(iFrame, iNode * 6 + 0), nodeList(iFrame, iNode * 6 + 1)));
            node.append(QPointF(nodeList(iFrame, iNode * 6 + 2), nodeList(iFrame, iNode * 6 + 3)));
            node.append(QPointF(nodeList(iFrame, iNode * 6 + 4), nodeList(iFrame, iNode * 6 + 5)));
            nodes.append(node);
        }
        vecList.append(nodes);
    }
//    qDebug() << "EigenMatrixToVecNodeList" << vecList.first().first().first().x();
    return vecList;
}

QList<VecNodeList> RotoCore::unNormVecNodeList(QList<VecNodeList> nodes, QPointF off) {
    QList<VecNodeList> outList;
    if (!nodes.isEmpty()) {
        for (QList<VecNodeList>::iterator iNodeList = nodes.begin(); iNodeList != nodes.end(); iNodeList++) {
            if (iNodeList->isEmpty()) break;
            VecNodeList list;
            for (QList<VecNode>::iterator iNode = iNodeList->begin(); iNode != iNodeList->end(); iNode++) {
                VecNode node;
                node.push_front((*iNode).at(0) - (*iNode).at(2)); // right position = center - left scene position
                node.push_front((*iNode).at(1) - (*iNode).at(0)); // left position = left scene position - center
                node.push_front((*iNode).at(0) - off);
                //                (*iNode)[2] = (*iNode).at(0) - (*iNode).at(2); // right position = center - left scene position
                //                (*iNode)[1] = (*iNode).at(1) - (*iNode).at(0); // left position = left scene position - center
                //                (*iNode)[0] = (*iNode).at(0)- off;
                list.append(node);
            }
            outList.append(list);
        }
    }
    return outList;
}

bool RotoCore::buildGPLVM() {
    Eigen::MatrixXd Y = normEigenKeyframes();
    currIndKeyframesForGPLVM = flowList->getIndKeyframes();
    if (Y.rows() < 1 || Y.cols() == 0) return false;
    const int Q = 2;
    const double initBeta = 0.01;
    gplvm = std::shared_ptr<GPLVM>(new GPLVM(Y, Q,
                                             shared_ptr<Kernel>(new RbfKernel()),
                                             initBeta,
                                             shared_ptr<NoiseHyperPriorFunction>(
                                                     new GaussianNoiseHyperPriorFunction(4.0, 0.01))));
//    gplvm->Print();
//    gplvm->TestGplvmGradients();
    gplvm->LearnParameters();
//    gplvm->Print();
    return true;
}

bool RotoCore::buildGPLVM_MotionEst() {
    return buildGPLVM(motionEst);
}

bool RotoCore::buildGPLVM_GrabDrag(){
    QList<int> keys = flowList->getIndKeyframes();
    if(keys.size() != currIndKeyframesForGPLVM.size()){
//        currIndKeyframesForGPLVM = flowList->getIndKeyframes();
        return buildGPLVM(motionEst);
    }

    if(keys.size()>1){
        for(int iKey = 0; iKey < keys.size(); iKey++){
            if(keys.at(iKey)!=currIndKeyframesForGPLVM.at(iKey)){
//                currIndKeyframesForGPLVM = flowList->getIndKeyframes();
                return buildGPLVM(motionEst);
            }
        }
    }
    return false;
}

bool RotoCore::buildGPLVM(shared_ptr<RigidMotionEstimator> &motionEstimator) {
    Eigen::MatrixXd Y = normEigenKeyframes();
    currIndKeyframesForGPLVM = flowList->getIndKeyframes();
//    qDebug() << "currIndKeyframesForGPLVM size:" << currIndKeyframesForGPLVM.size();
    if (Y.rows() < 1 || Y.cols() == 0) return false;
    const int Q = 2;
    const double initBeta = 0.01;

    shared_ptr<RigidMotionEstimator> tmp(new RigidMotionEstimator(Y));
    motionEstimator.swap(tmp);
    Y = motionEstimator->CalcNormalisedY();

    gplvm = std::shared_ptr<GPLVM>(new GPLVM(Y, Q,
                                             shared_ptr<Kernel>(new RbfKernel()),
                                             initBeta,
                                             shared_ptr<NoiseHyperPriorFunction>(
                                                     new GaussianNoiseHyperPriorFunction(4.0, 0.01))));
//    gplvm->Print();
//    gplvm->TestGplvmGradients();
    gplvm->LearnParameters();
//    gplvm->Print();
    return true;
}

bool RotoCore::SolveTrackAndGPLVMSingleFrame(int ind) {
    QList<int> indList;
    indList.append(ind);
    PlanarTrackingData curFrameTrack = getEigenAllFrame(indList);
    const int numFrames = view->getScenes()->size();
    const int startIdx = 0;
    const int endIdx = view->getScenes()->size() - 1;
    Eigen::MatrixXd keyframeData = this->normEigenKeyframes();
    const int D = keyframeData.cols();

//    auto mapIndex = [startIdx](int frameIdx) -> int { return frameIdx - startIdx; };

    Eigen::MatrixXd U(Eigen::MatrixXd::Constant(numFrames, 2, INFINITY));
    std::vector<int> keyFramesGplvmIndex;
    QList<int> indKeyframes = flowList->getIndKeyframes();
    keyFramesGplvmIndex.reserve(indKeyframes.size());

    for (int i = 0; i < indKeyframes.size(); i++) {
        int k = indKeyframes[i];
        U.row(k - startIdx) = gplvm->GetX().row(i);
        keyFramesGplvmIndex.push_back(k-startIdx);
    }

    // Do linear interpolation for init..
    for (int i = 0, I = indKeyframes.size() - 1; i < I; ++i) {
        const int a = indKeyframes[i];
        const int b = indKeyframes[i + 1];
        Interpolator<int> interp(a, b, U.row(a - startIdx), U.row(b-startIdx));
        for (int k = a + 1; k < b; ++k)
            U.row(k-startIdx) = interp.get(k);
    }

    nassert (!(U.array() == INFINITY).any());
    PlanarTrackingData keyFrameData = getEigenKeyframes();
    std::vector<PlanarTrackingData> trackDataList;
    trackDataList.push_back(curFrameTrack);
    PlanarTrackSolve planarTrackSolver(gplvm, U, trackDataList, startIdx, endIdx,
                                       keyFrameData, keyFramesGplvmIndex,
                                       motionEst);
    planarTrackSolver.RunSolve();
    Eigen::MatrixXd OutputData = planarTrackSolver.GetOutput();
    updateSolveNodeSingelFrame(OutputData, ind);
    return true;
}

bool RotoCore::SolveTrackAndGPLVM(std::vector<PlanarTrackingData> trackDataList, SolverType type) {
    if (trackDataList.size() < 1) return false;

    const int numFrames = trackDataList.front().NumFrames;
    const int startIdx = trackDataList.front().StartIndex;
    const int endIdx = trackDataList.front().EndIndex;
    Eigen::MatrixXd keyframeData = this->normEigenKeyframes();
//    const int D = keyframeData.cols();

//    auto mapIndex = [startIdx](int frameIdx) -> int { return frameIdx - startIdx; };
//    if(type == TRACKER_GPLVM || type == TRACKER_GPLVM_NEXTEDIT)

//    std::vector<int> keys = trackDataList.front().KeyFrames;

//    if(keys.size() > 1){
//        currIndKeyframesForGPLVM.clear();
//        for(int iKey = 0; iKey < keys.size(); iKey++)
//            currIndKeyframesForGPLVM.append(keys.at(iKey));
//    }

    this->buildGPLVM(motionEst);

    Eigen::MatrixXd U(Eigen::MatrixXd::Constant(numFrames, 2, INFINITY));
    std::vector<int> keyFramesGplvmIndex;
    QList<int> indKeyframes = flowList->getIndKeyframes();
    keyFramesGplvmIndex.reserve(indKeyframes.size());

    for (int i = 0; i < indKeyframes.size(); i++) {
        int k = indKeyframes[i];
        U.row(k-startIdx) = gplvm->GetX().row(i);
        keyFramesGplvmIndex.push_back(k-startIdx);
    }

    // Do linear interpolation for init..
    for (int i = 0; i < indKeyframes.size() - 1; ++i) {
        const int a = indKeyframes[i];
        const int b = indKeyframes[i + 1];
        Interpolator<int> interp(a, b, U.row(a-startIdx), U.row(b-startIdx));
        for (int k = a + 1; k < b; ++k)
            U.row(k-startIdx) = interp.get(k);
    }

    // Check all entries have been filled..
    nassert (!(U.array() == INFINITY).any());
    PlanarTrackingData keyFrameData = getEigenKeyframes();

    PlanarTrackSolve planarTrackSolver(gplvm, U, trackDataList, startIdx, endIdx,
                                       keyFrameData, keyFramesGplvmIndex,
                                       motionEst);
    Eigen::MatrixXd OutputData;
    if (type == TRACKER_GPLVM || type == TRACKER_GPLVM_NEXTEDIT) {
        bool isUsable = planarTrackSolver.RunSolve();
        OutputData = planarTrackSolver.GetOutput();
        if (isUsable) updateTrackingResults(OutputData);

        if (isUsable && type == TRACKER_GPLVM_NEXTEDIT) {
            Eigen::MatrixXd TrackOutputData = trackerMerge(trackDataList);
            QList<float> factor = rmsEigenMatrix(OutputData, TrackOutputData);
            suggestNextFrameForEdit(factor);
        }
    }
    else if (type == LINEAR_INTERP) {
        OutputData = planarTrackSolver.GetOutputLinearInterp();
        updateTrackingResults(OutputData);
    } else if (type == TRACKER) {
        OutputData = trackerMerge(trackDataList);
        updateTrackingResults(OutputData);
    }

    qDebug() << "solver output size, rows:" << OutputData.rows() << ", cols:" << OutputData.cols();
    return true;
}

QList<float> RotoCore::rmsEigenMatrix(Eigen::MatrixXd a, Eigen::MatrixXd b) {
    Eigen::MatrixXd diff = a - b;
    int nFrame = diff.rows();
    int nCol = diff.cols();
    QList<float> out;
    for (int iFrame = 0; iFrame < nFrame; iFrame++) {
        float factor = 0;
        for (int iCol = 0; iCol < nCol; iCol++) {
            factor += diff(iFrame, iCol) * diff(iFrame, iCol);
        }
        out.append(sqrt(factor / nCol));
    }
    return out;
}

void RotoCore::suggestNextFrameForEdit(QList<float> factor) {
    if (factor.isEmpty() || factor.size() != imgs->size()) return;
    int samRate = int(float(factor.size()) / 10);
    if (samRate < 1) return;
    QList<float> factorCopy = factor;
    QList<int> indSuggestedFrame;
    qSort(factorCopy.begin(), factorCopy.end());
    for (int iFrame = factor.size() - samRate; iFrame < factor.size(); iFrame++) {
        if (factorCopy[iFrame] > 5)
            indSuggestedFrame.append(factor.indexOf(factorCopy[iFrame]));

    }
    if (indSuggestedFrame.isEmpty()) return;
    flowList->setNextEdit(indSuggestedFrame);
//    flowList->getIcons().at(indSuggestedFrame.last())->pushImgIconClicked(indSuggestedFrame.last());
//    flowList->setCurrentRow(indSuggestedFrame.last());
}

Eigen::MatrixXd RotoCore::trackerMerge(std::vector<PlanarTrackingData> trackDataList) {
    int nFrame = trackDataList.front().NumFrames;
    int cols = trackDataList.front().PointIDs.size();
    Eigen::MatrixXd OutputData(nFrame, cols);
    if (trackDataList.size() == 2) {
        PlanarTrackingData forward = trackDataList.front();
        PlanarTrackingData backward = trackDataList.back();
        for (int iFrame = 0; iFrame < nFrame; iFrame++) {
            if (forward.FrameWeights[iFrame] == 1) {
                for (int iCol = 0; iCol < cols; iCol++)
                    OutputData(iFrame, iCol) = forward.TrackingData(iFrame, iCol);
            }
            else if (forward.FrameWeights[iFrame] > 0 && forward.FrameWeights[iFrame] < 1) {
                for (int iCol = 0; iCol < cols; iCol++) {
                    OutputData(iFrame, iCol) = forward.TrackingData(iFrame, iCol) * forward.FrameWeights[iFrame]
                                               + backward.TrackingData(iFrame, iCol) * backward.FrameWeights[iFrame];
                }
            }
            else {
                for (int iCol = 0; iCol < cols; iCol++)
                    OutputData(iFrame, iCol) = 0;
            }
        }
    }
    return OutputData;
}

PlanarTrackingData RotoCore::QtTrackerDataToPlanarTrackingData(TrackerData3 *qtTrackerData, bool flagBothSide) {
    PlanarTrackingData outputData;
    outputData.TrackingData = VecNodeListToEigenMatrix(qtTrackerData->nodes);
    outputData.StartIndex = 0;
    outputData.EndIndex = qtTrackerData->nodes.size() - 1;
    outputData.NumFrames = qtTrackerData->nodes.size();
    if (qtTrackerData->flagTrackDirection) outputData.DataType = ForwardPlanar;
    else outputData.DataType = BackwardPlanar;
    for (int id = 0; id < qtTrackerData->nodes.first().size() * 6; id++)
        outputData.PointIDs.push_back(id);
    for (int iKey = 0; iKey < qtTrackerData->indKeyframe.size(); iKey++)
        outputData.KeyFrames.push_back(qtTrackerData->indKeyframe[iKey]);
    outputData.FrameWeights = Eigen::VectorXd::Ones(outputData.NumFrames);
    if (flagBothSide) outputData.SetFrameWeights();
    outputData.OrigFileName = "";
    outputData.ShapeName = "";
    return outputData;
}

PlanarTrackingData RotoCore::getEigenKeyframes() {
    PlanarTrackingData outputData;
    outputData.TrackingData = normEigenKeyframes();
    outputData.StartIndex = 0;
    outputData.EndIndex = imgs->size() - 1;
    outputData.NumFrames = outputData.EndIndex - outputData.StartIndex + 1;
    outputData.OrigFileName = "";
    outputData.ShapeName = "";
    QList<int> keys = flowList->getIndKeyframes();
    for (int iKey = 0; iKey < keys.size(); iKey++)
        outputData.KeyFrames.push_back(keys[iKey]);
    for (int id = 0; id < view->getScenes()->at(keys[0])->getPoints()->size() * 6; id++)
        outputData.PointIDs.push_back(id);
    outputData.FrameWeights = Eigen::VectorXd::Ones(outputData.NumFrames);
    outputData.OrigFileName = "";
    outputData.ShapeName = "";
    return outputData;
}

PlanarTrackingData RotoCore::getEigenAllFrame(QList<int> ind) {
    PlanarTrackingData outputData;
    outputData.TrackingData = normEigenAllframes(ind);
    outputData.StartIndex = 0;
    outputData.EndIndex = imgs->size() - 1;
    outputData.NumFrames = outputData.EndIndex - outputData.StartIndex + 1;
    outputData.OrigFileName = "";
    outputData.ShapeName = "";
    QList<int> keys = flowList->getIndKeyframes();
    for (int iKey = 0; iKey < keys.size(); iKey++)
        outputData.KeyFrames.push_back(keys[iKey]);
    for (int id = 0; id < view->getScenes()->at(keys[0])->getPoints()->size() * 6; id++)
        outputData.PointIDs.push_back(id);
    outputData.FrameWeights = Eigen::VectorXd::Ones(outputData.NumFrames);
    outputData.OrigFileName = "";
    outputData.ShapeName = "";
    return outputData;
}

PlanarTrackingData RotoCore::getEigenFrame(int ind) {
    PlanarTrackingData outputData;
    outputData.TrackingData = normEigenFrame(ind);
    outputData.StartIndex = ind;
    outputData.EndIndex = ind;
    outputData.NumFrames = 1;
    outputData.OrigFileName = "";
    outputData.ShapeName = "";
    outputData.DataType = ForwardPlanar;
    QList<int> keys = flowList->getIndKeyframes();
    for (int iKey = 0; iKey < keys.size(); iKey++)
        outputData.KeyFrames.push_back(keys[iKey]);
    for (int id = 0; id < view->getScenes()->at(ind)->getPoints()->size() * 6; id++)
        outputData.PointIDs.push_back(id);
    outputData.FrameWeights = Eigen::VectorXd::Ones(outputData.NumFrames);
    outputData.OrigFileName = "";
    outputData.ShapeName = "";
    qDebug() << "the eigen matrix for the points of the frame: ";
    cout << outputData.TrackingData << endl;
    return outputData;
}

Eigen::MatrixXd RotoCore::VecNodeListToEigenMatrix(QList<VecNodeList> vecNodes) {

    int nFrame = 0;
    if (!vecNodes.isEmpty())
        nFrame = vecNodes.size();
    int cols = vecNodes.at(0).size() * 6;
    Eigen::MatrixXd out(nFrame, cols);

    for (int iKey = 0; iKey < vecNodes.size(); iKey++) {
        VecNodeList nodes = vecNodes.at(iKey);
        for (int iNode = 0; iNode < nodes.size(); iNode++) {

            QPointF center = nodes[iNode].at(0);
            QPointF left = nodes[iNode].at(1);
            QPointF right = nodes[iNode].at(2);
            out(iKey, iNode * 6 + 0) = center.x();
            out(iKey, iNode * 6 + 1) = center.y();
            out(iKey, iNode * 6 + 2) = left.x();
            out(iKey, iNode * 6 + 3) = left.y();
            out(iKey, iNode * 6 + 4) = right.x();
            out(iKey, iNode * 6 + 5) = right.y();
        }
    }
    return out;
}

int RotoCore::getIndClosestKeyframe(int ind) {
    int out = -1;
    int diff = imgs->size();
    QList<int> indKeys = flowList->getIndKeyframes();
    for (QList<int>::iterator iNode = indKeys.begin(); iNode != indKeys.end(); iNode++) {
        if (abs(*iNode - ind) < diff) out = *iNode;
    }
    return out;
}

int RotoCore::getIndClosestKeyframe(int ind, bool fwd) {
    int tmp = -1;
    QList<int> indKeys = flowList->getIndKeyframes();
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

bool RotoCore::track(TrackerData3 *trackRes, TrackType status, SolverType type) {

    VecKeyFrames qtKeys = normVecKeyframes();
    //    TrackerData3 trackRes;
    trackRes->clean();
    int nImg = imgs->size();
    VecNodeList tmpList;
    if (status == TrackType::FORWARD) {
        for (int iImg = 0; iImg < nImg; iImg++) {
            if (qtKeys.contains(iImg)) {
                trackRes->nodes.append(qtKeys[iImg]);
                trackRes->indKeyframe.append(iImg);
                tmpList = qtKeys[iImg];
                for (int iTrack = iImg + 1; iTrack < nImg; iTrack++) {
                    if (qtKeys.contains(iTrack)) break;
                    VecNodeList *to = new VecNodeList();
                    //                this->tracker.Track(tmpList,to,imgs->at(iImg),imgs->at(iTrack));
                    if (!this->avdtracker.Track(tmpList, to, iImg, iTrack) || type == LINEAR_INTERP) {
                        copyVecNodeListFrom(tmpList, to);
                    }
                    trackRes->nodes.append(*to);
                }
            }
        }
        trackRes->flagTrackDirection = true;
//        qDebug() << "forward tracker is performed!";
    } else if (status == TrackType::BACKWARD) {
        for (int iImg = nImg - 1; iImg > -1; iImg--) {
//            qDebug() << "backward runing iter:" << iImg;
            if (qtKeys.contains(iImg)) {
                trackRes->nodes.push_front(qtKeys[iImg]);
                trackRes->indKeyframe.push_front(iImg);
                tmpList = qtKeys[iImg];
                for (int iTrack = iImg - 1; iTrack > -1; iTrack--) {
//                    qDebug() << "backward runing inside iter:" << iTrack;
                    if (qtKeys.contains(iTrack)) break;
                    VecNodeList *to = new VecNodeList();
                    //                this->tracker.Track(tmpList,to,imgs->at(iImg),imgs->at(iTrack));
                    if (!this->avdtracker.Track(tmpList, to, iImg, iTrack) || type == LINEAR_INTERP) {
                        copyVecNodeListFrom(tmpList, to);
                    }
                    trackRes->nodes.push_front(*to);
                }
            }
        }
        trackRes->flagTrackDirection = false;
//        qDebug() << "backward tracker is performed!";
    }
    return true;
}


bool RotoCore::copyVecNodeListFrom(VecNodeList from, VecNodeList *out) {
    if (from.isEmpty()) return false;
    if (!out->isEmpty()) out->clear();
    for (int iNode = 0; iNode < from.size(); iNode++) {
        VecNode pt;
        pt.append(QPointF(from.at(iNode).at(0).x(), from.at(iNode).at(0).y()));
        pt.append(QPointF(from.at(iNode).at(1).x(), from.at(iNode).at(1).y()));
        pt.append(QPointF(from.at(iNode).at(2).x(), from.at(iNode).at(2).y()));
        out->append(pt);
    }
    return true;
}


void RotoCore::updateTrackingResults(TrackerData3 *trackRes) {
    trackRes->unNorm(this->getOffset());
    view->updateNodesAllScene(*trackRes);
    flowList->setAllSuggestOnNotKeyframes();
}

void RotoCore::updateTrackingResults(Eigen::MatrixXd trackRes) {
    TrackerData3 *out = new TrackerData3();
    out->nodes = EigenMatrixToVecNodeList(trackRes);
    out->indKeyframe = flowList->getIndKeyframes();
    out->unNorm(this->getOffset());
    view->updateNodesAllScene(*out);
    flowList->setAllSuggestOnNotKeyframes();
}

void RotoCore::updateSolveNodeSingelFrame(Eigen::MatrixXd trackRes, int ind) {
    TrackerData3 *out = new TrackerData3();
    out->nodes = EigenMatrixToVecNodeList(trackRes);
    out->indKeyframe = flowList->getIndKeyframes();
    out->unNorm(this->getOffset());
    view->getScenes()->at(ind)->updateNodePos((*out).nodes.at(ind), view->getScenes()->at(ind)->hasSltPts);
//    view->getScenes()->at(ind)->updateNodePos((*out).nodes.at(ind), false);
}

void RotoCore::loadImgs(QList<QImage *> imgs) {
    this->avdtracker.setFrames(imgs);
}

bool RotoCore::runActiveContour(int indScene) {
    if (indScene < 0 || indScene > view->getScenes()->size() - 1) return false;
    DesignScene *scene = view->getScenes()->at(indScene);
    NodeList *nodes = scene->getPoints();
    if (nodes->isEmpty()) return false;

    QtNodeList nodelist = this->normQtNodeList(nodes);
    QtNodeList *out = new QtNodeList;
    contour.SolveActiveContour(imgs->at(indScene), nodelist, out);
    QtNodeList unnormlist = unNormQtNodeList(*out);

    if (scene->hasSltPts) qDebug() << "there is draglist in current scene.";
    else
        qDebug() << "there is no draglist in current scene.";

    for (int iNode = 0; iNode < nodes->size(); iNode++) {
        if (!scene->hasSltPts) {
            if (nodes->at(iNode)->distance(unnormlist.at(iNode)) < 50) {
                nodes->at(iNode)->setPosition(unnormlist.at(iNode));
                nodes->at(iNode)->setPos(unnormlist.at(iNode));
            }
        } else {
            if (nodes->at(iNode)->isSlted
                && nodes->at(iNode)->distance(unnormlist.at(iNode)) < 50) {
                nodes->at(iNode)->setPosition(unnormlist.at(iNode));
                nodes->at(iNode)->setPos(unnormlist.at(iNode));
            }
        }
    }
    scene->updateScene();
    return true;
}

QtNodeList RotoCore::normQtNodeList(NodeList *nodes) {
    QtNodeList out;
    QPointF off = this->getOffset();
    for (int iNode = 0; iNode < nodes->size(); iNode++) {
        out.append(nodes->at(iNode)->getPosition() + off);
    }
    return out;
}

bool RotoCore::checkNumPtsConsistency() {
    if (imgs->isEmpty()) return false;
    if (view->getScenes()->first()->getPoints()->isEmpty()) return false;
    DesignSceneList *scenes = view->getScenes();
    int numPtsFirstFrame = scenes->first()->getPoints()->size();
    for (int iFrame = 0; iFrame < imgs->size(); iFrame++)
        if (numPtsFirstFrame != scenes->at(iFrame)->getPoints()->size()) return false;
    return true;
}

Eigen::MatrixXd RotoCore::normAllPoints() {
    if (!imgs->isEmpty() && checkNumPtsConsistency()) {
        QList<VecNodeList> vecList;
        for (int iFrame = 0; iFrame < imgs->size(); iFrame++)
            vecList.append(normVecFrame(iFrame));
        return VecNodeListToEigenMatrix(vecList);
    }
    else return Eigen::MatrixXd(0, 0);
}






