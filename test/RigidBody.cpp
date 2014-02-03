#include "RigidBody.h"
#include "Quat.h"
#include "Vec3.h"
#include "stdio.h"

namespace sc {

RigidBody::RigidBody(){
    m_invMass = 1;
    m_invInertia = 1;

    Quat::set(m_quaternion,0,0,0,1);
    Quat::set(m_tmpQuat1,0,0,0,1);
    Quat::set(m_tmpQuat2,0,0,0,1);

    Vec3::set(m_position,0,0,0);
    Vec3::set(m_velocity,0,0,0);
    Vec3::set(m_angularVelocity,0,0,0);
    Vec3::set(m_force,0,0,0);
    Vec3::set(m_torque,0,0,0);
}

RigidBody::~RigidBody(){}

void RigidBody::integrate(double dt){
    for (int i = 0; i < 3; ++i){

        // Linear
        m_velocity[i] += m_invMass * m_force[i] * dt;
        m_position[i] += dt * m_velocity[i];

        // Angular
        m_angularVelocity[i] += m_invInertia * m_torque[i] * dt;
        m_position[i] += dt * m_angularVelocity[i];
    }

    Quat::set(m_tmpQuat1, m_angularVelocity[0], m_angularVelocity[1], m_angularVelocity[2], 0.0);

    Quat::multiply(m_tmpQuat1, m_quaternion, m_tmpQuat2);

    m_quaternion[0] += 0.5 * dt * m_tmpQuat2[0];
    m_quaternion[1] += 0.5 * dt * m_tmpQuat2[1];
    m_quaternion[2] += 0.5 * dt * m_tmpQuat2[2];
    m_quaternion[3] += 0.5 * dt * m_tmpQuat2[3];

    Quat::normalize(m_quaternion,m_quaternion);
}

void RigidBody::resetForces(){
    Vec3::set(m_force,0,0,0);
    Vec3::set(m_torque,0,0,0);
}

void RigidBody::getDirectionalDerivative( double * outSpatial,
                                            double * outRotational,
                                            double * position,
                                            double * spatialDirection,
                                            double * rotationalDirection){
    saveState();

    // Just do spatial on the CM for now
    Vec3::copy(position,m_position);

    // Add force
    Vec3::set(m_force,0,0,0);
    Vec3::add(m_force,m_force,spatialDirection);

    // Step
    integrate(1);

    // The derivative is difference in velocity
    Vec3::subtract(outSpatial, m_velocity, m_velocity2);
    //Vec3::multiplyElementWise(outSpatial,outSpatial,spatialDirection);
    Vec3::set(outRotational,0,0,0);

    restoreState();
}

void RigidBody::saveState(){
    Vec3::copy(m_position2,m_position);
    Vec3::copy(m_velocity2,m_velocity);
    Vec3::copy(m_force2,m_force);
    Vec3::copy(m_torque2,m_torque);
    Vec3::copy(m_angularVelocity2,m_angularVelocity);
    Quat::copy(m_quaternion2,m_quaternion);
}

void RigidBody::restoreState(){
    Vec3::copy(m_position,m_position2);
    Vec3::copy(m_velocity,m_velocity2);
    Vec3::copy(m_force,m_force2);
    Vec3::copy(m_torque,m_torque2);
    Vec3::copy(m_angularVelocity,m_angularVelocity2);
    Quat::copy(m_quaternion,m_quaternion2);
}

}
