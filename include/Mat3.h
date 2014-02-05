#ifndef SCMAT3_H
#define SCMAT3_H

#include "stdlib.h"
#include "Vec3.h"

namespace sc {

    class Mat3 {

        private:
            double m_data[9];

        public:
            Mat3();
            virtual ~Mat3();
            void set(double, double, double,
                           double, double, double,
                           double, double, double);
            Mat3 multiplyMatrix(const Mat3& m) const;
            Vec3 multiplyVector(const Vec3& m) const;
            Mat3 scale(const Vec3& v);
            Vec3 getColumn(int i) const;

            double getElement(int row, int column) const;
            void setElement(int row, int column, double value);

            Mat3 operator* (const Mat3& y) const {
                return this->multiplyMatrix(y);
            }

            Vec3 operator* (const Vec3& y) const {
                return this->multiplyVector(y);
            }

    };

};

#endif
