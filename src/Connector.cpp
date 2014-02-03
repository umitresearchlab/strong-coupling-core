#include "Connector.h"

namespace sc {

Connector::Connector(){
    m_index = 0;
    // Set all members to zero
    for (int i = 0; i < 4; ++i){
        m_quaternion[i] = 0;
        if(i<3){
            m_position[i] = 0;
            m_velocity[i] = 0;
            m_angularVelocity[i] = 0;
            m_force[i] = 0;
            m_torque[i] = 0;
        }
    }
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

}
