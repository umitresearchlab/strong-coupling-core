#include "Connector.h"
#include "LockConstraint.h"
#include "Equation.h"
#include "stdio.h"
#include "stdlib.h"

using namespace sc;

LockConstraint::LockConstraint(Connector* connA, Connector* connB) : Constraint(connA,connB){
    addEquation(&m_xs);
    addEquation(&m_ys);
    addEquation(&m_zs);
    addEquation(&m_xr);
    addEquation(&m_yr);
    addEquation(&m_zr);

    // 6 equations, one for each DOF
    m_xs.setG(-1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);
    m_ys.setG( 0,-1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0);
    m_zs.setG( 0, 0,-1, 0, 0, 0, 0, 0, 1, 0, 0, 0);
    m_xr.setG( 0, 0, 0,-1, 0, 0, 0, 0, 0, 1, 0, 0);
    m_yr.setG( 0, 0, 0, 0,-1, 0, 0, 0, 0, 0, 1, 0);
    m_zr.setG( 0, 0, 0, 0, 0,-1, 0, 0, 0, 0, 0, 1);

    for(int i=0; i<getNumEquations(); i++){
        getEquation(i)->setConnectors(connA,connB);
        getEquation(i)->setDefault();
    }
}

LockConstraint::~LockConstraint(){}

void LockConstraint::update(){
    for(int i=0; i<getNumEquations(); i++){
        getEquation(i)->setDefaultViolation();
    }
}
