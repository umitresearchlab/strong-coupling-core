#ifndef SCRIGIDBODY_H
#define SCRIGIDBODY_H

#include "Mat3.h"
#include "Vec3.h"
#include "Quat.h"

namespace sc {

/// Simple rigid body class for testing.
class RigidBody {

public:

    RigidBody();
    virtual ~RigidBody();

    Mat3 m_invInertiaWorld;

    Vec3 m_position;
    Vec3 m_velocity;
    Vec3 m_angularVelocity;
    Vec3 m_force;
    Vec3 m_torque;

    Vec3 m_gravity;

    double m_invMass;
    Vec3 m_invInertia;

    Quat m_quaternion;
    Quat m_tmpQuat1;
    Quat m_tmpQuat2;

    // For storing states
    Vec3 m_position2;
    Vec3 m_velocity2;
    Vec3 m_force2;
    Vec3 m_torque2;
    Vec3 m_angularVelocity2;
    Quat m_quaternion2;

    Vec3 m_gravity2;
    double m_invMass2;
    Vec3 m_invInertia2;
    Mat3 m_invInertiaWorld2;

    /// Move forward in time
    void integrate(double dt);

    void resetForces();
    void getDirectionalDerivative(  Vec3& outSpatial,
                                    Vec3& outRotational,
                                    Vec3& position,
                                    const Vec3& spatialDirection,
                                    const Vec3& rotationalDirection);
    void saveState();
    void restoreState();
    void setLocalInertiaAsBox(double mass, const Vec3& halfExtents);
};

}

#endif

