#ifndef SCCONSTRAINT_H
#define SCCONSTRAINT_H

#include <vector>
#include "Equation.h"
#include "Connector.h"

namespace sc {

/**
 * Base class for constraints.
 */
class Constraint {

protected:
    std::vector<Equation*> m_equations;

public:

    Constraint(Connector*, Connector*);
    virtual ~Constraint();

    /// Arbitrary data from the user
    void * userData;

    Connector * m_connA;
    Connector * m_connB;

    /// Get number of equations in this constraint
    virtual int getNumEquations();

    /// Get one of the equations
    virtual Equation * getEquation(int i) = 0;
};

}

#endif
