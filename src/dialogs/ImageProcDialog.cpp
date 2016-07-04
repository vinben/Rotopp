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

#include "include/dialogs/ImageProcDialog.hpp"

ImageProcDialog::ImageProcDialog( DesignView *v):view(v) {

    DesignScene* scene = view->getCurScene();

    // contrast
    contrastSlider = new QSlider();
    contrastSlider->setMaximum(MAX_IMAGEPROC);
    contrastSlider->setOrientation(Qt::Horizontal);
    contrastSlider->setValue(scene->getContrast());

    contrastSpinBox = new QSpinBox();
    contrastSpinBox->setMaximum(MAX_IMAGEPROC);
    contrastSpinBox->setValue(scene->getContrast());

    contrastLabel = new QLabel(tr("Contrast"));

    // brightness
    brightSlider = new QSlider();
    brightSlider->setMaximum(MAX_IMAGEPROC);
    brightSlider->setOrientation(Qt::Horizontal);
    brightSlider->setValue(scene->getBrightness());

    brightSpinBox = new QSpinBox();
    brightSpinBox->setMaximum(MAX_IMAGEPROC);
    brightSpinBox->setValue(scene->getBrightness());

    brightLabel = new QLabel(tr("Brigthness"));

    // gamma
    gammaSlider = new QSlider();
    gammaSlider->setMaximum(MAX_IMAGEPROC);
    gammaSlider->setOrientation(Qt::Horizontal);
    gammaSlider->setValue(scene->getGamma());

    gammaSpinBox = new QSpinBox();
    gammaSpinBox->setMaximum(MAX_IMAGEPROC);
    gammaSpinBox->setValue(scene->getGamma());

    gammaLabel = new QLabel(tr("Gamma"));

    // button box
    buttonBox = new QDialogButtonBox;

    resetButton = new QPushButton(tr("Reset"));
    connect(this->resetButton, SIGNAL(clicked()), this, SLOT(reset()));
    buttonBox->addButton(this->resetButton, QDialogButtonBox::AcceptRole);

    closeButton = new QPushButton(tr("Close"));
    connect(this->closeButton, SIGNAL(clicked()), this, SLOT(close()));
    buttonBox->addButton(this->closeButton, QDialogButtonBox::RejectRole);

    buttonBox->setCenterButtons(true);

    // layout
    grid = new QGridLayout();
    grid->addWidget(contrastLabel,0,0);
    grid->addWidget(contrastSpinBox,0,1);
    grid->addWidget(contrastSlider,0,2);
    grid->addWidget(brightLabel,1,0);
    grid->addWidget(brightSpinBox,1,1);
    grid->addWidget(brightSlider,1,2);
    grid->addWidget(gammaLabel,2,0);
    grid->addWidget(gammaSpinBox,2,1);
    grid->addWidget(gammaSlider,2,2);
    grid->addWidget(buttonBox,3,2);

    QObject::connect(contrastSlider, SIGNAL(valueChanged(int)), contrastSpinBox, SLOT(setValue(int)));
    QObject::connect(contrastSlider, SIGNAL(valueChanged(int)), this, SLOT(setImageProperty()));
    QObject::connect(brightSlider, SIGNAL(valueChanged(int)), brightSpinBox, SLOT(setValue(int)));
    QObject::connect(brightSlider, SIGNAL(valueChanged(int)), this, SLOT(setImageProperty()));
    QObject::connect(gammaSlider, SIGNAL(valueChanged(int)), gammaSpinBox, SLOT(setValue(int)));
    QObject::connect(gammaSlider, SIGNAL(valueChanged(int)), this, SLOT(setImageProperty()));
    QObject::connect(contrastSpinBox, SIGNAL(valueChanged(int)), contrastSlider, SLOT(setValue(int)));
    QObject::connect(brightSpinBox, SIGNAL(valueChanged(int)), brightSlider, SLOT(setValue(int)));
    QObject::connect(gammaSpinBox, SIGNAL(valueChanged(int)), gammaSlider, SLOT(setValue(int)));

    this->setLayout(grid);
    this->setWindowTitle("Adjust Image Brightness/Contrast/Gamma");
}

void ImageProcDialog::reset(){
    view->resetImageProperty();
    contrastSlider->setValue(DEFAULT_CONTRAST);
    brightSlider->setValue(DEFAULT_BRIGHTNESS);
    gammaSlider->setValue(DEFAULT_GAMMA);
}

void ImageProcDialog::setImageProperty(){
    int b = brightSlider->value();
    int c = contrastSlider->value();
    int g = gammaSlider->value();
    view->setImagePropertyAllScene(b,c,g);
}
