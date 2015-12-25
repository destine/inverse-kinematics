#ifndef __incl_path__
#define __incl_path__

#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#define PI 3.14159

class Path
{
float a, b; //Coefficients for z = ax^3 + by^3
float rad1, rad2; //Horizontal and vertical radius, 1 by default
float degree; //Degree (in degrees not radians)
public:
    Path(void) {
        degree = 0;
        a = 1;
        b = 1;
    }

    virtual ~Path(void) {}

    /*
        Ellipse modeled by the equation (x/a)^2 + (y/b)^2 = 1
        Radius(a, b, Ø) = ab/sqrt(a^2*(sinØ)^2 + b^2*(cosØ)^2)
        x = Radius*cosØ, y = Radius*sinØ
        Plug x and y to equation z = ax^3 + by^3
    */
    virtual Eigen::Vector3f getCurrPoint(void);

    virtual Eigen::Vector3f getNextPoint(float degree_) {
        addDegree(degree_);
        return getCurrPoint();
    }

    virtual float getDegree(void) const {
        return degree;
    }

    virtual void setCoeff(float a_, float b_) {
        a = a_;
        b = b_;
    }

    virtual void setRad(float rad1_, float rad2_) {
        rad1 = rad1_;
        rad2 = rad2_;
    }

    virtual void addDegree(float d) {
        degree += d;
        if (degree >= 360) degree -= 360;
    }

    virtual void render(void);

    virtual void print() { //Debugging purposes, can remove later
        std::cout << "Path:\n";
        std::cout << "Coefficients: a = " << a << ", b = " << b << std::endl;
        std::cout << "Radii: x = " << rad1 << ", y = " << rad2 << std::endl;
        std::cout << "Current degree: " << degree << std::endl;
    }
};

class NoPath : public Path
{
public:
    NoPath(void) {}
    virtual ~NoPath(void) {}

    virtual void reset(void) {}
    virtual Eigen::Vector3f getNextPoint(float delta)
    {
        return Eigen::Vector3f();
    }
};

#endif
