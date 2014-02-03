#include "Connector.h"
#include "LockConstraint.h"
#include "Equation.h"
#include "stdio.h"
#include "stdlib.h"

namespace sc {

LockConstraint::LockConstraint(Connector* connA, Connector* connB) : Constraint(connA,connB){

    // Create 6 equations, one for each DOF
    for (int i = 0; i < 6; ++i){
        Equation * eq = new Equation(connA,connB);
        eq->m_G[i] =    -1;
        eq->m_G[i + 6] = 1;
        m_equations.push_back(eq);
    }
}

LockConstraint::~LockConstraint() {
    // Deallocate all equations
    while(m_equations.size() > 0){
        delete m_equations.back();
        m_equations.pop_back();
    }
}

}
