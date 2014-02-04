#ifndef EQUATION_H
#define EQUATION_H

#include "Connector.h"
#include "JacobianElement.h"

namespace sc {

/**
 * Base class for equations. Constrains two instances of Connector.
 */
class Equation {

private:
    Connector * m_connA;
    Connector * m_connB;

    JacobianElement G_A;
    JacobianElement G_B;

public:

    Equation(Connector*,Connector*);
    virtual ~Equation();

    /// Get connector A
    Connector * getConnA();

    /// Get connector B
    Connector * getConnB();

    double m_G[12];         // @todo should make own class for these
    double m_invMGt[12];

    /// Spook parameter "a"
    double m_a;

    /// Spook parameter "b"
    double m_b;

    /// Spook parameter "epsilon"
    double m_epsilon;

    /// Time step
    double m_timeStep;

    /**
     * @brief Sets a, b, epsilon according to SPOOK.
     * @param relaxation
     * @param compliance
     * @param timeStep
     */
    void setSpookParams(double relaxation, double compliance, double timeStep);

    /// Get constraint violation, g
    double getViolation();

    /// Get constraint velocity, G*W
    double getVelocity();

    /// Multiply the 12 equation elements (G) with another 12-element vector
    double Gmult(const Vec3& x1, const Vec3& v1, const Vec3& x2, const Vec3& v2);

    /// Multiply the 12 equation elements (G) with another 12-element vector
    double GmultG(double G[]);

    /**
     * @brief Set the spatial components of connector A jacobian.
     * @param x
     * @param y
     * @param z
     */
    void setSpatialJacobianA   (double x, double y, double z);
    void setSpatialJacobianA   (const Vec3& seed);

    /// Set the rotational components of connector A jacobian.
    void setRotationalJacobianA(double x, double y, double z);
    void setRotationalJacobianA(const Vec3& seed);

    /// Set the spatial components of connector B jacobian.
    void setSpatialJacobianB   (double x, double y, double z);
    void setSpatialJacobianB   (const Vec3& seed);

    /// Set the rotational components of connector B jacobian.
    void setRotationalJacobianB(double x, double y, double z);
    void setRotationalJacobianB(const Vec3& seed);

    /// Set all jacobian elements at once.
    void setJacobian(double,double,double,double,double,double,double,double,double,double,double,double);

    /**
     * @brief Get the spatial components of connector A jacobian seed.
     * @param seed
     */
    void getSpatialJacobianSeedA   (Vec3& seed);

    /// Get the rotational components of connector A jacobian.
    void getRotationalJacobianSeedA(Vec3& seed);

    /// Get the spatial components of connector B jacobian.
    void getSpatialJacobianSeedB   (Vec3& seed);

    /// Get the rotational components of connector B jacobian.
    void getRotationalJacobianSeedB(Vec3& seed);

};

}

#endif
