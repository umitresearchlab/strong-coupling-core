#include "Connector.h"

using namespace sc;

Connector::Connector(){
    m_index = 0;
}

Connector::~Connector(){

}

void Connector::setPosition(double x, double y, double z){
    m_position[0] = x;
    m_position[1] = y;
    m_position[2] = z;
}

void Connector::setVelocity(double vx, double vy, double vz){
    m_velocity[0] = vx;
    m_velocity[1] = vy;
    m_velocity[2] = vz;
}

void Connector::setOrientation(double x, double y, double z, double w){
    m_quaternion[0] = x;
    m_quaternion[1] = y;
    m_quaternion[2] = z;
    m_quaternion[3] = w;
}

void Connector::setAngularVelocity(double wx, double wy, double wz){
    m_angularVelocity[0] = wx;
    m_angularVelocity[1] = wy;
    m_angularVelocity[2] = wz;
}

double Connector::getConstraintForce(int element){
    return 0;
}

double Connector::getConstraintTorque(int element){
    return 0;
}
