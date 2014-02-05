#include "Connector.h"
#include "LockConstraint.h"
#include "Equation.h"
#include "stdio.h"
#include "stdlib.h"

using namespace sc;

LockConstraint::LockConstraint(Connector* connA, Connector* connB) : Constraint(connA,connB){
    m_xs.setConnectors(connA,connB);
    m_ys.setConnectors(connA,connB);
    m_zs.setConnectors(connA,connB);
    m_xr.setConnectors(connA,connB);
    m_yr.setConnectors(connA,connB);
    m_zr.setConnectors(connA,connB);

    // Create 6 equations, one for each DOF
    m_xs.setG(-1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);
    m_ys.setG( 0,-1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0);
    m_zs.setG( 0, 0,-1, 0, 0, 0, 0, 0, 1, 0, 0, 0);
    m_xr.setG( 0, 0, 0,-1, 0, 0, 0, 0, 0, 1, 0, 0);
    m_yr.setG( 0, 0, 0, 0,-1, 0, 0, 0, 0, 0, 1, 0);
    m_zr.setG( 0, 0, 0, 0, 0,-1, 0, 0, 0, 0, 0, 1);

    m_xs.setDefault();
    m_ys.setDefault();
    m_zs.setDefault();
    m_xr.setDefault();
    m_yr.setDefault();
    m_zr.setDefault();
}

LockConstraint::~LockConstraint(){}

int LockConstraint::getNumEquations(){
    return 6;
}

Equation * LockConstraint::getEquation(int i){
    switch(i){
        case 0: return &m_xs;
        case 1: return &m_ys;
        case 2: return &m_zs;
        case 3: return &m_xr;
        case 4: return &m_yr;
        case 5: return &m_zr;
    }
}
