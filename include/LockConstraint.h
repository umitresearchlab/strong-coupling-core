#ifndef SCLOCKCONSTRAINT_H
#define SCLOCKCONSTRAINT_H

#include "Constraint.h"

namespace sc {

/**
 * Locks all degrees of freedom between two connectors.
 */
class LockConstraint : public Constraint {

private:

public:
    LockConstraint(Connector* connA, Connector* connB);
    virtual ~LockConstraint();
};

}

#endif
