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

#include "include/utils.hpp"

/// Global vars

bool isFiled_curveMap = false;
bool isPrintedPt_curveMap = true;

/// data structure
///

qreal qtDistance(QPointF from, QPointF to){
    QPointF diff = from - to;
    return sqrt(diff.x()*diff.x() + diff.y()*diff.y());
}

QList<QPointF> TrackerData3::getPtListofCenter(int iList){
    QList<QPointF> centerList;
    if(iList>-1 && iList<nodes.size() && !nodes[iList].isEmpty()){
        VecNodeList tmpList = nodes[iList];
        for(int iNode = 0; iNode<nodes[iList].size(); iNode++){
            centerList.append(tmpList[iNode].first());
        }
    }
    return centerList;
}

void TrackerData3::unNorm(QPointF off){
    if(!nodes.isEmpty()){
        for(QList<VecNodeList>::iterator iNodeList = nodes.begin(); iNodeList!=nodes.end();iNodeList++){
            if(iNodeList->isEmpty()) break;
            for(QList<VecNode>::iterator iNode = iNodeList->begin(); iNode!=iNodeList->end();iNode++){
                (*iNode)[1] = (*iNode).at(1) - (*iNode).at(0); // left position = left scene position - center
                (*iNode)[2] = (*iNode).at(2) - (*iNode).at(0);// right position = right scene position - center
                (*iNode)[0] = (*iNode).at(0)- off;
            }
        }
    }
}

int TrackerData3::mostCloseIndKeys(int ind, bool fwd){

//    qDebug() << "the first keyframe: " << indKeyframe.first();
    int tmp = -1;
    if(fwd){
        for(int i = indKeyframe.size()-1; i > -1; i --){
            if(indKeyframe[i]>ind) tmp = indKeyframe[i];
        }
    }else{
        for(int i = 0; i < this->indKeyframe.size(); i ++){
            if(indKeyframe[i]<ind) tmp = indKeyframe[i];
        }
    }
    return tmp;
}

void TrackerData3::clean(){
    if(!nodes.isEmpty()) nodes.clear();
    if(!indKeyframe.isEmpty()) indKeyframe.clear();
}

int changeBrightness( int value, int brightness ){
    return kClamp( value + brightness * 255 / 100, 0, 255 );
}

int changeContrast( int value, int contrast ){
    return kClamp((( value - 127 ) * contrast / 100 ) + 127, 0, 255 );
}

int changeGamma( int value, int gamma ){
    return kClamp( int( pow( value / 255.0, 100.0 / gamma ) * 255 ), 0, 255 );
}

int changeUsingTable( int value, const int table[] ){
    return table[ value ];
}

/*
 Applies either brightness, contrast or gamma conversion on the image.
 If the image is not truecolor, the color table is changed. If it is
 truecolor, every pixel has to be changed. In order to make it as fast
 as possible, alpha value is converted only if necessary. Additionally,
 since color components (red/green/blue/alpha) can have only 256 values
 but images usually have many pixels, a conversion table is first
 created for every color component value, and pixels are converted
 using this table.
*/


template< int operation( int, int ) >

QImage changeImage( const QImage& image, int value ) {
    QImage im(image);
    im.detach();
    if( im.colorCount() == 0 ) {
        if( im.format() != QImage::Format_RGB32 ) {
            qDebug() << "the image depth is not 32";
            im = im.convertToFormat( QImage::Format_RGB32 );
        }

        int table[ 256 ];
        for( int i = 0; i < 256; ++i )
            table[ i ] = operation( i, value );

        if( im.hasAlphaChannel() ) {
            for( int y = 0; y < im.height(); ++y ) {
                QRgb* line = reinterpret_cast< QRgb* >( im.scanLine( y ));
                for( int x = 0; x < im.width(); ++x )
                    line[ x ] = qRgba( changeUsingTable( qRed( line[ x ] ), table ),
                                       changeUsingTable( qGreen( line[ x ] ), table ),
                                       changeUsingTable( qBlue( line[ x ] ), table ),
                                       changeUsingTable( qAlpha( line[ x ] ), table ));
            }
        }
        else {
            for( int y = 0; y < im.height(); ++y ) {
                QRgb* line = reinterpret_cast< QRgb* >( im.scanLine( y ));
                for( int x = 0; x < im.width(); ++x )
                    line[ x ] = qRgb( changeUsingTable( qRed( line[ x ] ), table ),
                                      changeUsingTable( qGreen( line[ x ] ), table ),
                                      changeUsingTable( qBlue( line[ x ] ), table ));
            }
        }
    }
    else {
        QVector<QRgb> colors = im.colorTable();
        for( int i = 0; i < im.colorCount(); ++i )
            colors[ i ] = qRgb( operation( qRed( colors[ i ] ), value ),
                                operation( qGreen( colors[ i ] ), value ),
                                operation( qBlue( colors[ i ] ), value ));
    }
    return im;
}

// brightness is multiplied by 100 in order to avoid floating point numbers
QImage changeBrightness( const QImage& image, int brightness ){
    if( brightness == 0 ) // no change
        return image;
    return changeImage< changeBrightness >( image, brightness );
}

// contrast is multiplied by 100 in order to avoid floating point numbers
QImage changeContrast( const QImage& image, int contrast ){
    if( contrast == 100 ) // no change
        return image;
    return changeImage< changeContrast >( image, contrast );
}

// gamma is multiplied by 100 in order to avoid floating point numbers
QImage changeGamma( const QImage& image, int gamma ){
    if( gamma == 100 ){
//        qDebug() << "no change.";
        return image;
    }

    return changeImage< changeGamma >( image, gamma );
}


struct vec2 {
    float x, y;
    inline vec2() : x(0), y(0) {}
    inline vec2(float x, float y) : x(x), y(y) {}
    inline vec2(float v[2]) : x(v[0]), y(v[1]) {}
#ifdef QSTRING_H
    inline QString toString() const {
        return QString("%1 %2").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2);
    }
#endif
};

inline vec2 operator -( vec2 a ) { return vec2( -a.x, -a.y ); }
inline vec2 operator +( vec2 a, vec2 b ) { return vec2( a.x+b.x, a.y+b.y ); }
inline vec2 operator +( vec2 a, float b ) { return vec2( a.x+b, a.y+b ); }
inline vec2 operator -( vec2 a, vec2 b ) { return vec2( a.x-b.x, a.y-b.y ); }
inline vec2 operator -( vec2 a, float b ) { return vec2( a.x-b, a.y-b ); }
inline vec2 operator *( float b, vec2 a ) { return vec2( a.x*b, a.y*b ); }
inline vec2 operator /( vec2 a, float b ) { return vec2( a.x/b, a.y/b ); }
inline vec2 operator /( float a, vec2 b ) { return vec2( a/b.x, a/b.y ); }
inline vec2 operator /( vec2 a, vec2 b ) { return vec2( a.x/b.x, a.y/b.y ); }
inline bool operator <( vec2 a, vec2 b ) { return a.x < b.x && a.y < b.y; }
inline bool operator >( vec2 a, vec2 b ) { return a.x > b.x && a.y > b.y; }

struct vec3 {
    float x, y, z;
    inline vec3() : x(0), y(0), z(0) {}
    inline vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    inline vec3(vec2 v, float z) : x(v.x), y(v.y), z(z) {}
    inline vec3(float v[3]) : x(v[0]), y(v[1]), z(v[2]) {}
    inline vec3 operator -() { return vec3(-x, -y, -z); }
    inline void operator +=(vec3 a) { x += a.x, y += a.y, z += a.z; }
    inline void operator *=(float a) { x *= a, y *= a, z *= a; }
    inline void operator /=(float a) { x /= a, y /= a, z /= a; }
    inline float& operator[](int i) { return (&x)[i]; }
    inline vec2 xy() { return vec2(x, y); }
#ifdef QSTRING_H
    inline QString toString() const {
        return QString("%1 %2 %3")
                .arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(z, 0, 'f', 2);
    }
#endif
};
inline float dot( vec3 a, vec3 b ) { return a.x*b.x + a.y*b.y + a.z*b.z; }
inline vec3 operator +(vec3 a, vec3 b) {
    return vec3( a.x+b.x, a.y+b.y, a.z+b.z );
}
inline vec3 operator +(vec3 a, float b) {
    return vec3( a.x+b, a.y+b, a.z+b );
}
inline vec3 operator -(vec3 a, vec3 b) {
    return vec3( a.x-b.x, a.y-b.y, a.z-b.z );
}
inline vec3 operator -(vec3 a, float b) {
    return vec3( a.x-b, a.y-b, a.z-b );
}
inline vec3 operator *(vec3 a, vec3 b) {
    return vec3( a.x*b.x, a.y*b.y, a.z*b.z );
}
inline vec3 operator /(vec3 a, vec3 b) {
    return vec3( a.x/b.x, a.y/b.y, a.z/b.z );
}
inline vec3 operator *(float b, vec3 a) {
    return vec3( a.x*b, a.y*b, a.z*b );
}
inline vec3 operator *(vec3 a, float b) {
    return vec3( a.x*b, a.y*b, a.z*b );
}
inline vec3 operator /(vec3 a, float b) {
    return vec3( a.x/b, a.y/b, a.z/b );
}
inline vec3 operator /(float a, vec3 b) {
    return vec3( a/b.x, a/b.y, a/b.z );
}
inline bool operator <(vec3 a, vec3 b) {
    return a.x < b.x && a.y < b.y && a.z < b.z;
}
inline bool operator >(vec3 a, vec3 b) {
    return a.x > b.x && a.y > b.y && a.z > b.z;
}
inline float length(vec3 a) { return sqrt(dot(a, a)); }
inline vec3 normalize(vec3 a) { return a*(1.0/length(a)); }

/*matrix operation*/

void mul_v3_m3v3(float r[3], float M[3][3], const float a[3]) {
    r[0] = M[0][0] * a[0] + M[1][0] * a[1] + M[2][0] * a[2];
    r[1] = M[0][1] * a[0] + M[1][1] * a[1] + M[2][1] * a[2];
    r[2] = M[0][2] * a[0] + M[1][2] * a[1] + M[2][2] * a[2];
}

void copy_m3_m3d(float R[3][3], double A[3][3])
{
    /* Keep it stupid simple for better data flow in CPU. */
    R[0][0] = (float)A[0][0];
    R[0][1] = (float)A[0][1];
    R[0][2] = (float)A[0][2];

    R[1][0] = (float)A[1][0];
    R[1][1] = (float)A[1][1];
    R[1][2] = (float)A[1][2];

    R[2][0] = (float)A[2][0];
    R[2][1] = (float)A[2][1];
    R[2][2] = (float)A[2][2];
}
