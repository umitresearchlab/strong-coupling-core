#ifndef SCLOCKCONSTRAINT_H
#define SCLOCKCONSTRAINT_H

#include "Constraint.h"

namespace sc {

/**
 * @brief Locks all 6 degrees of freedom between two connectors.
 * @todo Should be some way of specifying transform
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
