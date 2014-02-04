#ifndef SCJACOBIANELEMENT_H
#define SCJACOBIANELEMENT_H

#include "Vec3.h"

namespace sc {

    class JacobianElement {

        private:
            Vec3 spatial;
            Vec3 rotational;

        public:

            JacobianElement();
            virtual ~JacobianElement();

            double multiply(const Vec3& spatial, const Vec3& rotational);
            double multiply(const JacobianElement& e);
    };

};

#endif
