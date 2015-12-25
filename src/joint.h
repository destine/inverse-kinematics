#ifndef __incl_joint__
#define __incl_joint__

#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <iostream>

#include "body.h"

class Joint
{
protected:
    Body* m_inboard;
    Body* m_outboard;

public:
    Joint(Body* inboard, Body* outboard)
    {
        m_inboard = inboard;
        m_outboard = outboard;
    }

    virtual Body* getInboardBody(void) const;
    virtual Body* getOutboardBody(void) const;
    virtual void setInboardBody(Body* b);
    virtual void setOutboardBody(Body* b);

    virtual int getNumOfConstraints(void) const = 0;
    virtual void changeConstraint(int num, float delta) = 0;
    virtual Eigen::Vector3f transform(const Eigen::Vector3f& point) const = 0;
    virtual Eigen::MatrixXf getJacobian(void) = 0;
    virtual Eigen::Matrix4f getTransform(void) const = 0;

    virtual void render(void) = 0;
    virtual std::string getInstance(void) const = 0;
};

class BallJoint : public Joint
{
    Eigen::Vector3f m_expMap;

public:
    BallJoint(Body* inboard, Body* outboard):
    Joint(inboard, outboard)
    {
        m_expMap << 1, 1, 1;
    }

    virtual int getNumOfConstraints(void) const;
    virtual void changeConstraint(int num, float delta);
    virtual Eigen::Vector3f transform(const Eigen::Vector3f& point) const;
    virtual Eigen::MatrixXf getJacobian(void);
    virtual Eigen::Matrix4f getTransform(void) const;

    virtual void render(void);
    virtual std::string getInstance(void) const { //Debugging purposes only
        return "Ball Joint";
    }
};

class PrismJoint : public Joint
{
    float m_length;

public:
    PrismJoint(Body* inboard, Body* outboard):
    Joint(inboard, outboard)
    {
        m_length = 0.05;
    }

    virtual int getNumOfConstraints(void) const;
    virtual void changeConstraint(int num, float delta);
    virtual Eigen::Vector3f transform(const Eigen::Vector3f& point) const; 
    virtual Eigen::MatrixXf getJacobian(void);
    virtual Eigen::Matrix4f getTransform(void) const;

    virtual void render(void);
    virtual std::string getInstance(void) const { //Debugging purposes only
        return "Prismatic Joint";
    }
};

class PinJoint : public Joint
{
    float m_angle;

public:
    PinJoint(Body* inboard, Body* outboard):
    Joint(inboard, outboard)
    {
        m_angle = 0;
    }

    virtual int getNumOfConstraints(void) const;
    virtual void changeConstraint(int num, float delta);
    virtual Eigen::Vector3f transform(const Eigen::Vector3f& point) const; 
    virtual Eigen::MatrixXf getJacobian(void);
    virtual Eigen::Matrix4f getTransform(void) const;

    virtual void render(void);
    virtual std::string getInstance(void) const { //Debugging purposes only
        return "Pin Joint";
    }
};

class DoublePinJoint : public Joint
{
    float m_angle_x;
    float m_angle_y;

public:
    DoublePinJoint(Body* inboard, Body* outboard):
    Joint(inboard, outboard)
    {
        m_angle_x = 0;
        m_angle_y = 0;
    }

    virtual int getNumOfConstraints(void) const;
    virtual void changeConstraint(int num, float delta);
    virtual Eigen::Vector3f transform(const Eigen::Vector3f& point) const; 
    virtual Eigen::MatrixXf getJacobian(void);
    virtual Eigen::Matrix4f getTransform(void) const;

    virtual void render(void);
    virtual std::string getInstance(void) const { //Debugging purposes only
        return "Double Pin Joint";
    }
};

#endif
