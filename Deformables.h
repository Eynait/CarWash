//
//  Deformables.h
//  application
//
//  Created by Tianye Lu on 5/10/14.
//  Copyright (c) 2014 Force Dimension. All rights reserved.
//

#ifndef application_Deformables_h
#define application_Deformables_h

#include "chai3d.h"
using namespace chai3d;

class MassSpring {
public:
    MassSpring(const cVector3d& pt0, const cVector3d& pt1);
    ~MassSpring() {
        delete spring;
        delete sphere;
    }
    cShapeLine* spring;
    cShapeSphere* sphere;
    double mass;
    double Ln;
    double stiffness;
    double damping;
    cVector3d endPoint[2];
    cVector3d vel;
    cVector3d computeForce();
    void updatePosition(cVector3d& f, double dt);
};

class Membrane {
public:
    Membrane(const cVector3d pos[4]);
    ~Membrane() {
        for(int i=0; i<9; i++)
            for(int j=0; j<9; j++) {
                delete sphere[i][j];
            }
    }
    cShapeSphere* sphere[9][9];
    cVector3d position[9][9];
    cVector3d vel[9][9];
    double mass;
    double Ln;
    double stiffness;
    double damping;
    void computeForce(cVector3d f[9][9]);
    void updatePosition(cVector3d f[9][9], double dt);
};

class Cloth {
public:
    Cloth(const cVector3d pos[4],int n = 9);
    ~Cloth() {
        for(int i=0; i<size; i++){
            delete sphere[i];
            delete position[i];
            delete position_old[i];
            delete vel[i];
            delete f[i];
        }
        delete sphere;
        delete position;
        delete position_old;
        delete vel;
        delete f;
    }
    cShapeSphere*** sphere;
    cVector3d** position;
    cVector3d** position_old;
    cVector3d** vel;
    cVector3d** f;

    double mass;
    double Ln;
    double stiffness[3];
    double damping;
    int size;
    
    cVector3d update(double dt, cShapeSphere* cursor);
private:
    void computeForce();
    cVector3d collision(cShapeSphere* cursor);
    void updatePosition(cShapeSphere*, double dt);
    
};





#endif
