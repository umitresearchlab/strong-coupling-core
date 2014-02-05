#ifndef SCBALLJOINTCONSTRAINT_H
#define SCBALLJOINTCONSTRAINT_H

#include "Constraint.h"

namespace sc {

/**
 * Locks all degrees of freedom between two connectors.
 */
class BallJointConstraint : public Constraint {

private:
    Equation m_x;
    Equation m_y;
    Equation m_z;
    Vec3 m_localAnchorA;
    Vec3 m_localAnchorB;

public:
    BallJointConstraint(Connector* connA,
                        Connector* connB,
                        const Vec3& localAnchorA,
                        const Vec3& localAnchorB);
    virtual ~BallJointConstraint();

    void update();
};

}

#endif
