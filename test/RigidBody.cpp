#include "RigidBody.h"
#include "Quat.h"
#include "Vec3.h"
#include "Mat3.h"
#include "stdio.h"

using namespace sc;

RigidBody::RigidBody(){
    m_invMass = 1;
    m_invInertia = 1;
    m_invInertiaWorld.set(  1,0,0,
                            0,1,0,
                            0,0,1 );
}

RigidBody::~RigidBody(){}

void RigidBody::integrate(double dt){

    // Add some gravity
    m_force += m_gravity;

    // Integrate
    int i;
    for(i=0; i<3; i++){

        // Linear
        m_velocity[i] += m_invMass * m_force[i] * dt;
        m_position[i] += dt * m_velocity[i];

        // Angular
        m_angularVelocity[i] += m_invInertia * m_torque[i] * dt;
        m_position[i] += dt * m_angularVelocity[i];
    }

    // Integrate orientation
    m_tmpQuat1.set(m_angularVelocity[0], m_angularVelocity[1], m_angularVelocity[2], 0.0);
    m_tmpQuat1 = m_quaternion.multiply(m_quaternion, m_tmpQuat2);
    m_quaternion[0] += 0.5 * dt * m_tmpQuat2[0];
    m_quaternion[1] += 0.5 * dt * m_tmpQuat2[1];
    m_quaternion[2] += 0.5 * dt * m_tmpQuat2[2];
    m_quaternion[3] += 0.5 * dt * m_tmpQuat2[3];
    m_quaternion.normalize();

    // Reset gravity
    m_force.set(0,0,0);
}

void RigidBody::resetForces(){
    m_force.set(0,0,0);
    m_torque.set(0,0,0);
}

void RigidBody::getDirectionalDerivative(   Vec3& outSpatial,
                                            Vec3& outRotational,
                                            Vec3& position,
                                            const Vec3& spatialDirection,
                                            const Vec3& rotationalDirection){

    Vec3 velo_noforce,
         velo_withforce;

    // Just do spatial on the CM for now
    position.copy(m_position);

    // Step with external force
    saveState();
    m_force.set(0,0,0);
    m_force += spatialDirection;
    integrate(1);
    velo_withforce.copy(m_velocity);
    restoreState();

    // Step without added force
    saveState();
    integrate(1);
    velo_noforce.copy(m_velocity);
    restoreState();

    // The derivative is difference in velocity
    outSpatial = velo_noforce.subtract(velo_withforce);
    //Vec3::multiplyElementWise(outSpatial,outSpatial,spatialDirection);
    outRotational.set(0,0,0);

}

void RigidBody::saveState(){
    m_position2.copy(m_position);
    m_velocity2.copy(m_velocity);
    m_force2.copy(m_force);
    m_torque2.copy(m_torque);
    m_angularVelocity2.copy(m_angularVelocity);
    m_quaternion2.copy(m_quaternion);
}

void RigidBody::restoreState(){
    m_position.copy(m_position2);
    m_velocity.copy(m_velocity2);
    m_force.copy(m_force2);
    m_torque.copy(m_torque2);
    m_angularVelocity.copy(m_angularVelocity2);
    m_quaternion.copy(m_quaternion2);
}

