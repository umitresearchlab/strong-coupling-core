#include "Quat.h"
#include "Vec3.h"
#include "Solver.h"
#include "Slave.h"
#include "RigidBody.h"
#include "Connector.h"
#include "Constraint.h"
#include "LockConstraint.h"
#include "stdio.h"
#include <vector>
#include "string.h"

using namespace sc;

void printHelp(char * command){
    printf("\nUsage:\n");
    printf("\t%s [options]\n\n",command);
    printf("[options]:\n");
    printf("\t--numBodies N\tSets the number of bodies to N\n");
    printf("\t--help,-h\tPrint help and quit\n");
    printf("\n");
}

/*
 * Runs "delete" on all instances in a vector, and removes all of them from the vector.
 */
template<typename T>
void deleteVectorContent(std::vector<T> v){
    while(v.size() > 0){
        delete v.back();
        v.pop_back();
    }
}

/*
 * Prints out the first row of CSV with variable names, for example "time,x0,f0,x1,f1,..." depending on the number of bodies.
 */
void printFirstCSVRow(std::vector<RigidBody*> bodies){
    printf("time");
    for (int i = 0; i < bodies.size(); ++i){
        printf(",x%d", i);
        printf(",f%d", i);
    }
    printf("\n");
}

/*
 * Prints a CSV data row
 */
void printCSVRow(double t, std::vector<RigidBody*> bodies){
    // Print results
    printf("%lf", t);
    for (int j = 0; j < bodies.size(); ++j){
        RigidBody * body = bodies[j];
        printf(",%lf", body->m_position[0]);
        printf(",%lf", body->m_force[0]);
    }
    printf("\n");
}

int main(int argc, char ** argv){
    int N = 2,
        NT = 10,
        quiet = 0;
    double  dt = 0.01,
            relaxation = 3,
            compliance = 0.001,
            invMass = 1,
            invInertia = 1,
            gravityX = 0;

    // Parse arguments
    for (int i = 0; i < argc; ++i){
        char * a = argv[i];
        int last = (i == argc-1);

        if(!last){
            if(!strcmp(a,"--numBodies"))   N = atoi(argv[i+1]);
            if(!strcmp(a,"--numSteps"))    NT = atoi(argv[i+1]);
            if(!strcmp(a,"--compliance"))  compliance = atof(argv[i+1]);
            if(!strcmp(a,"--relaxation"))  relaxation = atof(argv[i+1]);
            if(!strcmp(a,"--timeStep"))    dt = atof(argv[i+1]);
            if(!strcmp(a,"--gravityX"))    gravityX = atof(argv[i+1]);
        }

        if(strcmp(argv[i],"--help")==0 || strcmp(argv[i],"-h")==0){
            printHelp(argv[0]);
            return 0;
        }

        if(strcmp(argv[i],"--quiet")==0 || strcmp(argv[i],"-q")==0)
            quiet = 1;
    }

    Solver solver;

    // Init bodies / slave systems
    std::vector<RigidBody*> bodies;
    std::vector<Slave*> slaves;
    std::vector<Connector*> connectors;
    std::vector<Constraint*> constraints;
    Connector * lastConnector = NULL;
    for (int i = 0; i < N; ++i){

        // Create body
        RigidBody * body = new RigidBody();
        body->m_position[0] = (double)i;
        body->m_invMass = i==0 ? 0 : invMass;
        body->m_invInertia = i==0 ? 0 : invInertia;
        body->m_gravity.set(gravityX,0,0);

        // Create slave
        Slave * slave = new Slave();

        // Create connector at center of mass of the body
        Connector * conn = new Connector();
        slave->addConnector(conn);
        conn->m_userData = (void*)body;

        // Note: Must add slave *after* adding connectors
        solver.addSlave(slave);

        // Create lock joint between this and last connector
        if(lastConnector != NULL){
            Constraint * constraint = new LockConstraint(lastConnector, conn);
            solver.addConstraint(constraint);
            constraints.push_back(constraint);
        }

        lastConnector = conn;

        bodies.push_back(body);
        slaves.push_back(slave);
        connectors.push_back(conn);
    }

    // Set spook parameters on all equations
    solver.setSpookParams(relaxation,compliance,dt);

    // Get system equations
    std::vector<Equation *> eqs;
    solver.getEquations(&eqs);

    // Print CSV first column
    if(!quiet) printFirstCSVRow(bodies);
    if(!quiet) printCSVRow(0,bodies);

    // Time loop
    for (int i = 0; i < NT; ++i){

        // Simulation time
        double t = i*dt;

        // Set connector values
        for (int j = 0; j < N; ++j){
            RigidBody * body = bodies[j];
            Connector * conn = connectors[j];
            conn->m_position.copy(body->m_position);
            conn->m_velocity.copy(body->m_velocity);
        }

        // Get jacobian information
        for (int j = 0; j < eqs.size(); ++j){
            Equation * eq = eqs[j];

            RigidBody * bodyA = (RigidBody *)eq->getConnA()->m_userData;
            RigidBody * bodyB = (RigidBody *)eq->getConnB()->m_userData;

            Vec3 spatSeed,
                 rotSeed,
                 ddSpatial,
                 ddRotational;

            // Set jacobians
            eq->getSpatialJacobianSeedA(spatSeed);
            eq->getRotationalJacobianSeedA(rotSeed);
            bodyA->getDirectionalDerivative(ddSpatial,ddRotational,bodyA->m_position,spatSeed,rotSeed);
            eq->setSpatialJacobianA(ddSpatial);
            eq->setRotationalJacobianA(ddRotational);

            //printf("A = (%f %f %f)\n", ddSpatial[0], ddSpatial[1], ddSpatial[2]);

            eq->getSpatialJacobianSeedB(spatSeed);
            eq->getRotationalJacobianSeedB(rotSeed);
            bodyB->getDirectionalDerivative(ddSpatial,ddRotational,bodyB->m_position,spatSeed,rotSeed);
            eq->setSpatialJacobianB(spatSeed);
            eq->setRotationalJacobianB(rotSeed);

            //printf("B = (%f %f %f)\n", ddSpatial[0], ddSpatial[1], ddSpatial[2]);
        }

        // Solve system
        solver.solve();

        // Add resulting constraint forces to the bodies
        for (int j = 0; j < slaves.size(); ++j){
            bodies[j]->m_force.copy(slaves[j]->getConnector(0)->m_force);
            bodies[j]->m_torque.copy(slaves[j]->getConnector(0)->m_torque);
        }

        // Integrate bodies
        for (int j = 0; j < N; ++j){
            RigidBody * body = bodies[j];
            body->integrate(dt);
        }

        // Print results
        if(!quiet) printCSVRow(t,bodies);
    }

    deleteVectorContent(bodies);
    deleteVectorContent(slaves);
    deleteVectorContent(connectors);
    deleteVectorContent(constraints);
}
