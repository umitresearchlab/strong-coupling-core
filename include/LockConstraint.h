#ifndef SCLOCKCONSTRAINT_H
#define SCLOCKCONSTRAINT_H

#include "Constraint.h"

namespace sc {

/**
 * Locks all degrees of freedom between two connectors.
 */
class LockConstraint : public Constraint {

private:
    Equation m_xs;
    Equation m_ys;
    Equation m_zs;
    Equation m_xr;
    Equation m_yr;
    Equation m_zr;

public:
    LockConstraint(Connector* connA, Connector* connB);
    virtual ~LockConstraint();
    void update();
};

}

#endif
