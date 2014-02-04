#ifndef SCMAT3_H
#define SCMAT3_H

#include "stdlib.h"

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
            //Mat3 scale(const Vec3& v) const;

            Mat3 operator* (const Mat3& y) const {
                return this->multiplyMatrix(y);
            }

    };

};

#endif
