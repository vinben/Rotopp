# Roto++
---

Rotoscoping (cutting out different characters/objects/layers in raw video footages) is a ubiquitous task in modern post-production and represents a significant investment in person-hours. In this work, we study the task of rotoscoping and use machine learning techniques to improve the workflow and productivity of roto-artists using machine learning techniques. We implement a user-friendly tool for rotoscoping. Our tool gives instant and accurate shape prediction for frames by using planar tracker and shape manifold that learned from user's input.

### Licence

The source code is released under a BSD like license as below. A closed-source professional edition may be available for commercial purposes in the very near future. In this case, please contact the authors for further information.

```
Copyright (c) 2016, Wenbin Li 
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the 
following conditions are met:

-- Redistributions of source code and data must retain the above copyright notice, this list of conditions and 
   the following disclaimer.
-- Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the 
   following disclaimer in the documentation and/or other materials provided with the distribution.

THIS WORK AND THE RELATED SOFTWARE, SOURCE CODE AND DATA IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF 
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

### Cite

If you use any part of this code in an academic context, please cite the following publication:

```
@article{Rotopp2016,
  title   = {Roto++: Accelerating Professional Rotoscoping using Shape Manifolds},
  author  = {Wenbin Li and Fabio Viola and Jonathan Starck and Gabriel J. Brostow and Neill D.F. Campbell},
  journal =  {ACM Transactions on Graphics (In proceeding of ACM SIGGRAPH'16)},
  volume  = {35},
  pages   = {4},
  year    = {2016}
}
```

### Supported Platforms

The `Roto++` supports two main platforms of `Ubuntu 14.04+` and `Mac OSX 10.6+`.

### Dependencies

By default, the `Roto++` requires three main dependencies of `opencv 2.4.10/11`, `ceres-solver 1.10+` and `Qt 5.0+`.

- Ubuntu

```
    sudo apt-get install libopencv-dev
```
	
```
    sudo apt-get install libeigen3-dev libatlas-base-dev libgoogle-glog-dev libsuitesparse-dev
    git clone https://ceres-solver.googlesource.com/ceres-solver
    cd ceres-solver
    mkdir build 
    cd build 
    cmake ..
    make
    sudo make install
```

```
    sudo apt-get install qtbase5-dev qtdeclarative5-dev
```

- Mac

    To use `Qt 5.0+` on Mac, you could download the source code and compile; or you could install the `QtCreator` with the `Qt 5.0+` library. For the latter, the project can be load and compiled by `QtCreator`.
    
	For the easy way to install dependencies, you are suggested to use `brew`. Please install the `brew` and set the right permission to your `/usr/local` folder: 

```
	sudo chown -R YOUR_USERNAME /usr/local
```

```
    brew install glog
    brew install eigen
    brew install suite-sparse
    brew tap homebrew/science
    brew install ceres-solver
    brew install opencv
```

### Compilation

We suggest to load and compile the project using `QtCreator` with `Qt 5.4+` compile kit or to follow:

```
    cd Rotoscope
    mkdir build 
    cd build 
    cmake ..
    make
```

For either Unix or Mac OS without installing the `QtCreator`, you should modify the value of `CMAKE_PREFIX_PATH` in `CMakeLists.txt` by pointing it to the `Qt` library.

### Data Structure

- Configuration (`RotoConfig.rotoInit`)

To config the program, you could create `RotoConfig.rotoInit` within the same folder of the main program. This configuration supports `tabs` like:

```
<RotoLib> : add path of additional Qt library
<DebugOutput> : ON or OFF, enable the debug information
<RotoSolverType> : Int (0..4), initial solver type
<SmartBezier> : ON or OFF, enable the smart Bezier assistance
<FilledCurveMap> : ON or OFF, enable to fill the curve when it is exported as curveMap
<LineWidthCurveMap> : float (>0), set the line width of plotting curveMap
<ShowPtFilledCurveMap> : ON or OFF, enable to show the control points when exporting curveMap
```

Within a configuration file, each row consist of a tab and the content which is split by `SPACE`. This is an example file as follows:

```
RotoLib /usr/lib/x86_64-linux-gnu
RotoLib /usr/local/Qt5.5
DebugOutput ON
```

- Saved/AutoSaved (`*.roto`, `*.rotoAutoSave`)

```
<name of project>
<name of curve>
<Time stamp for last save>
<solver status> <number of frames> <number of keyframes> <image resolution: (width,height)> <total number of points (only center point not include tangent points)>
<list of points per frame>

<points per frame> = <frame status> <number of points> <index of first node> <is Closed Curve (0 or 1)>
                     <list of points>

<solver status> = <TRACKER_GPLVM_NEXTEDIT:0 TRACKER_GPLVM:1 TRACKER:2 LINEAR_INTERP:3>
<frame status> = <NOEDIT:0 KEYFRAME:1 SUGGESTED:2 EDITING:3 NEXTEDIT:4>
<point> = <center.x center.y left.x left.y right.x right.y>

*** Follow a coordinate of right x and down y; with the origin at the center of image.
```

- Logging (`*.rotoLog`)

```
<name of project>
<name of curve>
<Time stamp for last save>
<number of frames> <image resolution: (width,height)>
<List of Logs>

<Log> = <Time stamp>
        <solver status> <editType flags for frames>
        <Index of current frame> <design flags>
        <Mouse click (x,y)> <Mouse release (x,y)>
```

- Export (`*.rotoExport`)

```
<name of project>
<name of curve>
<number of frames> <6 X number of points> <is data transposed (0 or 1)> <is Closed Curve (0 or 1)>
<List of points>

<point> = <center.x center.y left.x left.y right.x right.y>

*** if the data is not transposed, each row contains all points for the frame.
```

- AutoExport (`*.rotoAutoExport`)

```
<name of project>
<name of curve>
<Time stamp for first save>
<number of frames> <6 X number of points> <is data transposed (0 or 1)> 
<list of status stamps>

<status stamp> = <Time stamp for last save>
				 <List of points>

<point> = <center.x center.y left.x left.y right.x right.y>

*** if the data is not transposed, each row contains all points for the frame.
```