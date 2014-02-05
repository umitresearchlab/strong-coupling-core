#ifndef SCJACOBIANELEMENT_H
#define SCJACOBIANELEMENT_H

#include "Vec3.h"

namespace sc {

    class JacobianElement {

        private:
            Vec3 m_spatial;
            Vec3 m_rotational;

        public:

            JacobianElement();
            virtual ~JacobianElement();

            double multiply(const Vec3& spatial, const Vec3& rotational);
            double multiply(const JacobianElement& e);

            void setSpatial(double,double,double);
            void setRotational(double,double,double);
            Vec3 getSpatial() const;
            Vec3 getRotational() const;
            void print();
    };

};

#endif
