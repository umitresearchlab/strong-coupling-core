#include "Connector.h"
#include "Constraint.h"
#include "stdio.h"

using namespace sc;

Constraint::Constraint(Connector* connA, Connector* connB){
    m_connA = connA;
    m_connB = connB;
}

Constraint::~Constraint(){}

int Constraint::getNumEquations(){
    return m_equations.size();
}
