#include "Mat3.h"

using namespace sc;

Mat3::Mat3(){
    set(0,0,0,
        0,0,0,
        0,0,0);
}

Mat3::~Mat3(){

}

void Mat3::set( double a11, double a12, double a13,
                double a21, double a22, double a23,
                double a31, double a32, double a33){
    m_data[0] = a11;
    m_data[1] = a12;
    m_data[2] = a13;
    m_data[3] = a21;
    m_data[4] = a22;
    m_data[5] = a23;
    m_data[6] = a31;
    m_data[7] = a32;
    m_data[8] = a33;
}

    /*
Vec3 Mat3::multiplyVector(const Vec3& v) const {
    Vec3 result;
    result.set( e[0]*x + e[1]*y + e[2]*z,
                e[3]*x + e[4]*y + e[5]*z,
                e[6]*x + e[7]*y + e[8]*z);
    return result;
};
    */

Mat3 Mat3::multiplyMatrix(const Mat3& m) const {
    Mat3 result;
    /*
    for(var i=0; i<3; i++){
        for(var j=0; j<3; j++){
            var sum = 0.0;
            for(var k=0; k<3; k++){
                sum += m.elements[i+k*3] * this.elements[k+j*3];
            }
            r.elements[i+j*3] = sum;
        }
    }
    */
    return result;
};

    /*
Mat3 Mat3::scale(const Vec3& v) const{
    Mat3 result;
    var e = this.elements,
        t = target.elements;
    for(var i=0; i!==3; i++){
        t[3*i + 0] = v.x * e[3*i + 0];
        t[3*i + 1] = v.y * e[3*i + 1];
        t[3*i + 2] = v.z * e[3*i + 2];
    }
    return result;
};
    */
