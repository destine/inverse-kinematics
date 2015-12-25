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

#include <string>
#include "arm.h"

void
Arm::appendJoint(Joint* joint)
{
    m_joints.push_back(joint);
    m_pLastJoint = joint;
}

Eigen::Vector3f
Arm::getEndEffector(void) const
{
    //Position(e) = (R1 * T1 * R2 * T2 ...) * origin <-- [0, 0, 0, 1]
    Eigen::Vector4f point;
    point <<    0, 0, 0, 1;
    Eigen::Matrix4f transforms;
    transforms <<   1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;
    std::list<Joint*>::const_iterator iter;
    for (iter = m_joints.begin(); iter != m_joints.end(); ++iter) {
        transforms = transforms * (*iter)->getTransform(); //Rotation
        Eigen::Matrix4f translation;
        translation <<  1, 0, 0, (*iter)->getOutboardBody()->getLength(),
                        0, 1, 0, 0,
                        0, 0, 1, 0,
                        0, 0, 0, 1;
        transforms = transforms * translation;
    }
    point = transforms * point;
    Eigen::Vector3f toReturn;
    toReturn <<     point(0), point(1), point(2);
    return toReturn;
}

void
Arm::approachPoint(const Eigen::Vector3f& point, const float strength)
{
    int numJacobianColumns = 0;
    float armLength = 0.0f;
    std::list<Joint*>::iterator iter;
    bool prismatic = false;
    for (iter = m_joints.begin(); iter != m_joints.end(); ++iter)
    {
        numJacobianColumns += (*iter)->getNumOfConstraints();
        armLength += (*iter)->getOutboardBody()->getLength();
        if ((*iter)->getInstance() == "Prismatic Joint") {
            prismatic = true;
        }
    }

    Eigen::Vector3f armTip = getEndEffector();

    Eigen::Vector3f goal = point;
    if (goal.norm() > armLength && !prismatic)
        goal = (goal / goal.norm()) * armLength;

    Eigen::Vector3f deltaP = goal - armTip;

    Eigen::MatrixXf fullJacobian(3, numJacobianColumns);

    Eigen::Matrix3f jointTransform;
    Eigen::MatrixXf jointJacobian;

    jointTransform << 1, 0, 0, // Identity Matrix
                      0, 1, 0,
                      0, 0, 1;

    int columnCounter = 0;
    int subCounter = 0;

    Eigen::Matrix3f jointTransformSub;

    for (iter = m_joints.begin(); iter != m_joints.end(); ++iter)
    {
        jointJacobian = jointTransform * (*iter)->getJacobian();

        for (subCounter = 0;
             subCounter < (*iter)->getNumOfConstraints();
             ++subCounter, ++columnCounter)
        {
            fullJacobian.col(columnCounter) = jointJacobian.col(subCounter);
        }

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                jointTransformSub(i, j) = (*iter)->getTransform()(i, j);
            }
        }

        jointTransform = jointTransform * jointTransformSub; // incompatible
    }

    Eigen::VectorXf deltaTheta = fullJacobian.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(deltaP);

    int vectorIndex = 0;

    for (iter = m_joints.begin(); iter != m_joints.end(); ++iter)
    {
        for (int i = 0; i < (*iter)->getNumOfConstraints(); ++i)
        {
            (*iter)->changeConstraint(i, deltaTheta(vectorIndex) * strength);
            ++vectorIndex;
        }
    }
}

void
Arm::render(float interpolation)
{
    glPushMatrix();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    std::list<Joint*>::iterator iter;
    for (iter = m_joints.begin(); iter != m_joints.end(); ++iter) {
        Eigen::MatrixXf transform = (*iter)->getTransform();
        if (transform.size() != 16) {
            std::cout << "Error: transform is not 4x4" << std::endl;
            break;
        }
        GLfloat vals[16];
        for (int col = 0; col < transform.cols(); col++) {
            for (int row = 0; row < transform.rows(); row++) {
                vals[col * 4 + row] = transform(row, col);
            }
        }

        glMultMatrixf(vals);

        //Draw joint and body here
        glPushMatrix();
        glRotatef(90, 0, 1, 0);
        glutSolidCone(0.03, (*iter)->getOutboardBody()->getLength(), 32, 32);
        glPopMatrix();
        (*iter)->render();

        glTranslatef((*iter)->getOutboardBody()->getLength(), 0.0, 0.0);
    }
    glPopMatrix();
}
