#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#ifdef __APPLE__
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif

#include "joint.h"

#define EPSILON 0.015625f

Body*
Joint::getInboardBody(void) const
{
    return m_inboard;
}

Body*
Joint::getOutboardBody(void) const
{
    return m_outboard;
}

void
Joint::setInboardBody(Body* b) {
    m_inboard = b;
    return;
}

void
Joint::setOutboardBody(Body* b) {
    m_outboard = b;
    return;
}

//--------------BallJoint------------------

int
BallJoint::getNumOfConstraints(void) const
{
    return 3;
}

void
BallJoint::changeConstraint(int num, float delta)
{
    m_expMap(num) += delta;
}

Eigen::Vector3f
BallJoint::transform(const Eigen::Vector3f& point) const
{
    float angle = m_expMap.norm();
    Eigen::Vector3f axis = m_expMap;
    if (angle != 0) {
        axis = axis/axis.norm();
    }

    Eigen::Matrix3f transform;

    transform <<        0,  -axis(2),  axis(1),
                  axis(2),         0, -axis(0),
                 -axis(1),   axis(0),        0;

    transform = axis * axis.transpose() +
                transform * sin(angle) -
                transform * transform * cos(angle);

    return transform * point;
}

Eigen::MatrixXf
BallJoint::getJacobian(void)
{
    Eigen::Matrix3f jacobian;
    Eigen::Vector3f endPoint;
    Eigen::Vector3f deltaEnd;

    Eigen::Vector3f initialConfig;

    Eigen::Vector3f bodyVector(m_outboard->getLength(), 0, 0);
    endPoint = transform(bodyVector);

    initialConfig = m_expMap;
    m_expMap = initialConfig + Eigen::Vector3f(EPSILON, 0, 0);
    deltaEnd = transform(bodyVector) - endPoint;
    jacobian.col(0) = deltaEnd;

    m_expMap = initialConfig + Eigen::Vector3f(0, EPSILON, 0);
    deltaEnd = transform(bodyVector) - endPoint;
    jacobian.col(1) = deltaEnd;

    m_expMap = initialConfig + Eigen::Vector3f(0, 0, EPSILON);
    deltaEnd = transform(bodyVector) - endPoint;
    jacobian.col(2) = deltaEnd;

    m_expMap = initialConfig;

    return jacobian / EPSILON;
}

void BallJoint::render(void) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    GLUquadric* quad = gluNewQuadric();
    gluSphere(quad, 0.03, 10, 10);
}

Eigen::Matrix4f
BallJoint::getTransform(void) const
{
    float angle = m_expMap.norm();
    Eigen::Vector3f axis = m_expMap / m_expMap.norm();
    Eigen::Vector4f homogen;
    homogen <<  axis(0), axis(1), axis(2), 0;

    Eigen::Matrix4f transform;

    transform << 0,        -axis(2), axis(1), 0,
                 axis(2),  0,       -axis(0), 0,
                 -axis(1), axis(0), 0,        0,
                 0,        0,       0,        0;

    transform = homogen * homogen.transpose() +
                transform * sin(angle) -
                transform * transform * cos(angle);

    transform(3, 3) = 1;

    return transform;
}

//--------------PinJoint------------------

int
PinJoint::getNumOfConstraints(void) const
{
    return 1;
}

void
PinJoint::changeConstraint(int num, float delta)
{
    if (num == 0)
        m_angle += delta;
}

Eigen::Vector3f
PinJoint::transform(const Eigen::Vector3f& point) const
{
    float newX = point(0) * cos(m_angle) - point(1) * sin(m_angle);
    float newY = point(1) * cos(m_angle) + point(0) * sin(m_angle);
    return Eigen::Vector3f(newX, newY, point(2));
}

Eigen::MatrixXf
PinJoint::getJacobian(void)
{
    Eigen::Matrix<float, 3, 1> jacobian;
    Eigen::Vector3f endPoint;
    Eigen::Vector3f deltaEnd;

    float initialConfig;

    Eigen::Vector3f bodyVector(m_outboard->getLength(), 0, 0);
    endPoint = transform(bodyVector);

    initialConfig = m_angle;
    m_angle = initialConfig + EPSILON;
    deltaEnd = transform(bodyVector) - endPoint;
    jacobian.col(0) = deltaEnd;

    m_angle = initialConfig;

    return jacobian / EPSILON;
}

Eigen::Matrix4f
PinJoint::getTransform(void) const
{
    Eigen::Matrix4f transform;

    transform <<  cos(m_angle), -sin(m_angle), 0, 0,
                  sin(m_angle), cos(m_angle), 0, 0,
                  0,            0,            1, 0,
                  0,            0,            0, 1;

    return transform;
}

void PinJoint::render(void) {
    glPushMatrix();
    glTranslatef(0, 0, -0.04);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    GLUquadric* quad = gluNewQuadric();
    gluCylinder(quad, 0.02, 0.02, 0.08, 10, 10);

    gluDisk(quad, 0, 0.02, 10, 10);
    glTranslatef(0, 0, 0.08);
    gluDisk(quad, 0, 0.02, 10, 10);

    glPopMatrix();
}

//--------------PrismJoint------------------

int
PrismJoint::getNumOfConstraints(void) const
{
    return 1;
}

void
PrismJoint::changeConstraint(int num, float delta)
{
    if (num == 0 && m_length + delta >= 0)
        m_length += delta;
}

Eigen::Vector3f
PrismJoint::transform(const Eigen::Vector3f& point) const
{
    return point + Eigen::Vector3f(m_length, 0, 0);
}

Eigen::MatrixXf
PrismJoint::getJacobian(void)
{
    Eigen::Matrix<float, 3, 1> jacobian;
    Eigen::Vector3f endPoint;
    Eigen::Vector3f deltaEnd;

    float initialConfig;

    Eigen::Vector3f bodyVector(m_outboard->getLength(), 0, 0);
    endPoint = transform(bodyVector);

    initialConfig = m_length;
    m_length = initialConfig + EPSILON;
    deltaEnd = transform(bodyVector) - endPoint;
    jacobian.col(0) = deltaEnd;

    m_length = initialConfig;

    return jacobian / EPSILON;
}

Eigen::Matrix4f
PrismJoint::getTransform(void) const
{
    Eigen::Matrix4f transform;

    transform << 1, 0, 0, m_length,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1;

    return transform;
}

void PrismJoint::render(void) {
    glPushMatrix();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glRotatef(90, 0, 1, 0);
    glTranslatef(0, 0, -m_length);
    GLUquadric* quad = gluNewQuadric();
    gluCylinder(quad, 0.03, 0.03, m_length, 10, 10);
    glPopMatrix();
}

//--------------DoublePinJoint------------------

int
DoublePinJoint::getNumOfConstraints(void) const
{
    return 2;
}

void
DoublePinJoint::changeConstraint(int num, float delta)
{
    if (num == 0)
        m_angle_x += delta;
    else if (num == 1)
        m_angle_y += delta;
}

Eigen::Vector3f
DoublePinJoint::transform(const Eigen::Vector3f& point) const
{
    Eigen::Vector4f homogen;
    homogen <<      point(0), point(1), point(2), 1;
    Eigen::Vector4f result;
    result = getTransform() * homogen;
    Eigen::Vector3f toReturn;
    toReturn <<     result(0), result(1), result(2);
    return toReturn;
}

Eigen::MatrixXf
DoublePinJoint::getJacobian(void)
{
    Eigen::Matrix<float, 3, 2> jacobian;
    Eigen::Vector3f endPoint;
    Eigen::Vector3f deltaEnd;

    float initialConfig_x;
    float initialConfig_y;

    Eigen::Vector3f bodyVector(m_outboard->getLength(), 0, 0);
    endPoint = transform(bodyVector);

    initialConfig_x = m_angle_x;
    m_angle_x = initialConfig_x + EPSILON;
    deltaEnd = transform(bodyVector) - endPoint;
    jacobian.col(0) = deltaEnd;
    m_angle_x = initialConfig_x;

    initialConfig_y = m_angle_y;
    m_angle_y = initialConfig_y + EPSILON;
    deltaEnd = transform(bodyVector) - endPoint;
    jacobian.col(1) = deltaEnd;
    m_angle_y = initialConfig_y;

    return jacobian / EPSILON;
}

Eigen::Matrix4f
DoublePinJoint::getTransform(void) const
{
    Eigen::Matrix4f r1;

    r1 <<         cos(m_angle_x), -sin(m_angle_x), 0, 0,
                  sin(m_angle_x), cos(m_angle_x), 0, 0,
                  0,            0,            1, 0,
                  0,            0,            0, 1;

    Eigen::Matrix4f r2;
    r2 <<         cos(m_angle_y),   0,  sin(m_angle_y), 0,
                  0,                1,  0,              0,
                  -sin(m_angle_y),  0,  cos(m_angle_y), 0,
                  0,                0,  0,              1;     

    return r1 * r2;
}

void DoublePinJoint::render(void) {
    glPushMatrix();
    glTranslatef(0, 0, -0.04);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    GLUquadric* quad = gluNewQuadric();
    gluCylinder(quad, 0.02, 0.02, 0.08, 10, 10);

    gluDisk(quad, 0, 0.02, 10, 10);
    glTranslatef(0, 0, 0.08);
    gluDisk(quad, 0, 0.02, 10, 10);
    glPopMatrix();

    glPushMatrix();
    glRotatef(90, 1, 0, 0);
    glTranslatef(0, 0, -0.04);
    gluCylinder(quad, 0.02, 0.02, 0.08, 10, 10);

    gluDisk(quad, 0, 0.02, 10, 10);
    glTranslatef(0, 0, 0.08);
    gluDisk(quad, 0, 0.02, 10, 10);
    glPopMatrix();
}
