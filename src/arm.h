#ifndef __incl_arm__
#define __incl_arm__

#include "joint.h"

#include <iostream> //remove later
#include <list>

class Arm
{
    std::list<Joint*> m_joints;
    Joint* m_pLastJoint;

public:
    Arm(void) {
    	m_pLastJoint = NULL;
    }

    Joint* getLastJoint() {
    	return m_pLastJoint;
    }

    std::list<Joint*> getJoints() {
    	return m_joints;
    }

    void appendJoint(Joint* joint);

    Eigen::Vector3f getEndEffector(void) const;
    void approachPoint(const Eigen::Vector3f& point, const float strength);

    void render(float interpolation);

    //Debugging purposes, can remove later
    void print() {
    	std::list<Joint*>::iterator iter;
    	int i = 0;
    	for (iter = m_joints.begin(); iter != m_joints.end(); ++iter) {
    		std::cout << "Joint " << i << ": ";
    		std::cout << "Type: " << (*iter)->getInstance() << ", ";
    		if (!(*iter)->getInboardBody()) {
    			std::cout << "Inboard body = NULL, ";
    		} else {
    			std::cout << "Inboard body = length " << (*iter)->getInboardBody()->getLength() << ", ";
    		}
    		std::cout << "Outboard body = length " << (*iter)->getOutboardBody()->getLength() << std::endl;
    		i++;
    	}
    	return;
    }
};

#endif
