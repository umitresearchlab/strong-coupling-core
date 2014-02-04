#include "Equation.h"
#include "stdio.h"

using namespace sc;

Equation::Equation(Connector * connA, Connector * connB){
    m_connA = connA;
    m_connB = connB;

    // Set default solver params
    setSpookParams(4,1e-5,1.0/100.0);

    for (int i = 0; i < 12; ++i){
        m_G[i] = 0;
        m_invMGt[i] = 0;
    }
}

Equation::~Equation(){}

Connector * Equation::getConnA(){
    return m_connA;
}
Connector * Equation::getConnB(){
    return m_connB;
}

void Equation::setSpookParams(double relaxation, double compliance, double timeStep){
    m_a = 4/(1+4*relaxation)/timeStep;
    m_b = 4*relaxation/(1+4*relaxation);
    m_epsilon = 4 * compliance / (timeStep*timeStep * (1 + 4*relaxation));
    m_timeStep = timeStep;
}

double Equation::getViolation(){
    Vec3 zero;
    return Gmult(   m_connA->m_position,
                    zero,
                    m_connB->m_position,
                    zero);
}

double Equation::getVelocity(){
    return Gmult(   m_connA->m_velocity,
                    m_connA->m_angularVelocity,
                    m_connB->m_velocity,
                    m_connB->m_angularVelocity);
}

double Equation::Gmult(const Vec3& x1, const Vec3& v1, const Vec3& x2, const Vec3& v2){
    return  x1.x() * m_G[0] +
            x1.y() * m_G[1] +
            x1.z() * m_G[2] +
            v1.x() * m_G[3] +
            v1.y() * m_G[4] +
            v1.z() * m_G[5] +
            x2.x() * m_G[6] +
            x2.y() * m_G[7] +
            x2.z() * m_G[8] +
            v2.x() * m_G[9] +
            v2.y() * m_G[10] +
            v2.z() * m_G[11];
}

double Equation::GmultG(double G[]){
    return  G[0] * m_G[0] +
            G[1] * m_G[1] +
            G[2] * m_G[2] +
            G[3] * m_G[3] +
            G[4] * m_G[4] +
            G[5] * m_G[5] +
            G[6] * m_G[6] +
            G[7] * m_G[7] +
            G[8] * m_G[8] +
            G[9] * m_G[9] +
            G[10] * m_G[10] +
            G[11] * m_G[11];
}

void Equation::setJacobian(   double G1,
                                double G2,
                                double G3,
                                double G4,
                                double G5,
                                double G6,
                                double G7,
                                double G8,
                                double G9,
                                double G10,
                                double G11,
                                double G12 ){
    setSpatialJacobianA(G1,G2,G3);
    setRotationalJacobianA(G4,G5,G6);
    setSpatialJacobianB(G7,G8,G9);
    setRotationalJacobianB(G10,G11,G12);
}

void Equation::setSpatialJacobianA(double x, double y, double z){
    m_invMGt[0] = x;
    m_invMGt[1] = y;
    m_invMGt[2] = z;
}

void Equation::setSpatialJacobianA(const Vec3& seed){
    m_invMGt[0] = seed.x();
    m_invMGt[1] = seed.y();
    m_invMGt[2] = seed.z();
}

void Equation::setRotationalJacobianA(double x, double y, double z){
    m_invMGt[3] = x;
    m_invMGt[4] = y;
    m_invMGt[5] = z;
}

void Equation::setRotationalJacobianA(const Vec3& seed){
    m_invMGt[3] = seed.x();
    m_invMGt[4] = seed.y();
    m_invMGt[5] = seed.z();
}

void Equation::setSpatialJacobianB(double x, double y, double z){
    m_invMGt[6] = x;
    m_invMGt[7] = y;
    m_invMGt[8] = z;
}

void Equation::setSpatialJacobianB(const Vec3& seed){
    m_invMGt[6] = seed.x();
    m_invMGt[7] = seed.y();
    m_invMGt[8] = seed.z();
}

void Equation::setRotationalJacobianB(double x, double y, double z){
    m_invMGt[9] = x;
    m_invMGt[10] = y;
    m_invMGt[11] = z;
}

void Equation::setRotationalJacobianB(const Vec3& seed){
    m_invMGt[9] =  seed.x();
    m_invMGt[10] = seed.y();
    m_invMGt[11] = seed.z();
}

void Equation::getSpatialJacobianSeedA(Vec3& seed){
    seed[0] = m_G[0];
    seed[1] = m_G[1];
    seed[2] = m_G[2];
}

void Equation::getRotationalJacobianSeedA(Vec3& seed){
    seed[0] = m_G[3];
    seed[1] = m_G[4];
    seed[2] = m_G[5];
}

void Equation::getSpatialJacobianSeedB(Vec3& seed){
    seed[0] = m_G[6];
    seed[1] = m_G[7];
    seed[2] = m_G[8];
}

void Equation::getRotationalJacobianSeedB(Vec3& seed){
    seed[0] = m_G[9];
    seed[1] = m_G[10];
    seed[2] = m_G[11];
}
