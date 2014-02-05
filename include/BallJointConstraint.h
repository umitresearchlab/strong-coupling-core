#ifndef SCBALLJOINTCONSTRAINT_H
#define SCBALLJOINTCONSTRAINT_H

#include "Constraint.h"

namespace sc {

/// Connects the connectors with a ball joint
class BallJointConstraint : public Constraint {

private:
    Equation m_x;
    Equation m_y;
    Equation m_z;
    Vec3 m_localAnchorA;
    Vec3 m_localAnchorB;

public:

    /**
     * @param connA         First connector
     * @param connB         Second connector
     * @param localAnchorA  Anchor point in connector A frame
     * @param localAnchorB  Anchor point in connector B frame
     */
    BallJointConstraint(Connector* connA,
                        Connector* connB,
                        const Vec3& localAnchorA,
                        const Vec3& localAnchorB);
    virtual ~BallJointConstraint();

    /// Update equations and violations. Should be called whenever the connector states changed.
    void update();
};

}

#endif
