#include <stdio.h>
#include "Slave.h"
#include "Solver.h"
#include "LockConstraint.h"
#include "Equation.h"
#include "Test.h"

using namespace sc;

int main(int argc, char const *argv[]){

    // Construct test
    Solver solver;

    // Create slaves
    Slave slaveA;
    Slave slaveB;
    solver.addSlave(&slaveA);
    solver.addSlave(&slaveB);

    // Create connectors
    Connector connA;
    Connector connB;
    slaveA.addConnector(&connA);
    slaveB.addConnector(&connB);

    // Set connector states. These are used by the solver to compute violations.
    connA.setPosition(1,0,0);
    connB.setPosition(0,2,0);
    connA.setOrientation(0,0,0,1); // Quaternions
    connB.setOrientation(0,0,0,1);
    connA.setVelocity(0,0,1);
    connB.setVelocity(0,0,-3);
    connA.setAngularVelocity(0,1,1);
    connB.setAngularVelocity(-1,0,2);

    // Constrain connectors
    LockConstraint lock(&connA,&connB);
    solver.addConstraint(&lock);

    // Loop over all equations in the lock constraint.
    // In the lock case, we have 6 equations.
    int numEquations = lock.getNumEquations(), i;
    for(i=0; i<numEquations; i++){

        Equation * eq = lock.getEquation(i);

        // Get seed vector for getting directional derivatives
        // Equivalent to G(row,body)*eps
        // Todo

        // The seed forces are applied in the slave and we will get back
        // jacobian entries, which are stored as follows
        eq->setSpatialJacobianA     (eq->m_G[0],eq->m_G[1],eq->m_G[2]); // Use jacobian as directional derivative => dv/df==1
        eq->setRotationalJacobianA  (eq->m_G[3],eq->m_G[4],eq->m_G[5]);
        eq->setSpatialJacobianB     (eq->m_G[6],eq->m_G[7],eq->m_G[8]);
        eq->setRotationalJacobianB  (eq->m_G[9],eq->m_G[10],eq->m_G[11]);
    }

    // Solve the system
    solver.solve();

    // Get results, to be applied on participating bodies
    connA.getConstraintForce(0);
    connA.getConstraintForce(1);
    connA.getConstraintForce(2);
    connA.getConstraintTorque(1);
    connA.getConstraintTorque(2);
    connA.getConstraintTorque(3);

    connB.getConstraintForce(0);
    connB.getConstraintForce(1);
    connB.getConstraintForce(2);
    connB.getConstraintTorque(1);
    connB.getConstraintTorque(2);
    connB.getConstraintTorque(3);

    return 0;
}