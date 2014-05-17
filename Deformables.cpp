//
//  Deformables.cpp
//  application
//
//  Created by Tianye Lu on 5/10/14.
//  Copyright (c) 2014 Force Dimension. All rights reserved.
//

#include "Deformables.h"

MassSpring::MassSpring(const cVector3d& pt0, const cVector3d& pt1) {
    endPoint[0] = pt0;
    endPoint[1] = pt1;
    spring = new cShapeLine(endPoint[0], endPoint[1]);
    spring->setLineWidth(3.0);
    sphere = new cShapeSphere(0.01);
    sphere->setLocalPos(endPoint[1]);
    Ln = cSub(pt0, pt1).length();
    mass = 1;
    stiffness = 1;
    damping = 1;
    vel = cVector3d(0.,0.,0.);
}
cVector3d MassSpring::computeForce(){
    cVector3d force = cVector3d(0.,0.,0.);
    cVector3d r = endPoint[1] - endPoint[0];
    double L = r.length();
    cVector3d u = r/(L+1e-9);
    force += -stiffness*(L-Ln)*u;//spring force
    force += -damping*vel.dot(u)*u;//damping force
    force += -vel;//air damping force
    force += cVector3d(0,0,-mass*9.81);//gravity
    return force;
}
void MassSpring::updatePosition(cVector3d& f, double dt){
    //Euler Forward Integration
    vel += f/mass*dt;
    endPoint[1] += vel*dt;
    spring->m_pointB = endPoint[1];
    sphere->setLocalPos(endPoint[1]);
}

Membrane::Membrane(const cVector3d pos[4]) {
    for (int i=0; i<9; i++)
        for(int j=0; j<9; j++) {
            double u = 1.0 - i/9.0;
            double v = 1.0 - j/9.0;
            cVector3d p1 = u*pos[0]+(1.0-u)*pos[1];
            cVector3d p2 = u*pos[2]+(1.0-u)*pos[3];
            position[i][j] = v*p1 + (1.0-v)*p2;
            vel[i][j] = cVector3d(0.,0.,0.);
            sphere[i][j] = new cShapeSphere(0.02);
            sphere[i][j]->setLocalPos(position[i][j]);
        }
    Ln = cSub(pos[0], pos[1]).length()/8.0;
    mass = 0.05;
    stiffness = 5000;
    damping = 5.0;
}

void Membrane::computeForce(cVector3d f[9][9]){
    for (int i=0; i<9; i++)
        for (int j=0; j<9; j++) {
            f[i][j] = cVector3d(0.,0.,0.);
            f[i][j] += cVector3d(0,0,-mass*9.81);//gravity
            f[i][j] += -vel[i][j];//air damping force
            //spring forces
            //i-1,j
            if (i>0) {
                cVector3d r = cSub(position[i][j], position[i-1][j]);
                double L = r.length();
                cVector3d u = r/(L+1e-9);
                f[i][j] += -stiffness*(L-Ln)*u;//spring force
                f[i][j] += -damping*vel[i][j].dot(u)*u;//damping force
            }
            //i+1,j
            if (i<8) {
                cVector3d r = cSub(position[i][j], position[i+1][j]);
                double L = r.length();
                cVector3d u = r/(L+1e-9);
                f[i][j] += -stiffness*(L-Ln)*u;//spring force
                f[i][j] += -damping*vel[i][j].dot(u)*u;//damping force
            }
            //i,j-1
            if (j>0) {
                cVector3d r = cSub(position[i][j], position[i][j-1]);
                double L = r.length();
                cVector3d u = r/(L+1e-9);
                f[i][j] += -stiffness*(L-Ln)*u;//spring force
                f[i][j] += -damping*vel[i][j].dot(u)*u;//damping force
            }
            //i,j+1
            if (j<8) {
                cVector3d r = cSub(position[i][j], position[i][j+1]);
                double L = r.length();
                cVector3d u = r/(L+1e-9);
                f[i][j] += -stiffness*(L-Ln)*u;//spring force
                f[i][j] += -damping*vel[i][j].dot(u)*u;//damping force
            }
        }
}
void Membrane::updatePosition(cVector3d f[9][9], double dt){
    //Euler forward integration
    for (int i=0; i<9; i++)
        for (int j=0; j<9; j++) {
            if ((i==0 || i==8) && (j==0 ||j==8))
                continue;
            else{
                vel[i][j] += f[i][j]/mass*dt;
                position[i][j] += vel[i][j]*dt;
                sphere[i][j]->setLocalPos(position[i][j]);
            }
    }
}

Cloth::Cloth(const cVector3d pos[4], int n) {
    size = n;
    sphere = new cShapeSphere**[n];
    position = new cVector3d*[n];
    position_old = new cVector3d*[n];
    vel = new cVector3d*[n];
    f = new cVector3d*[size];
    for (int i=0; i<n; i++) {
        sphere[i] = new cShapeSphere*[n];
        position[i] = new cVector3d[n];
        position_old[i] = new cVector3d[n];
        vel[i] = new cVector3d[n];
        f[i] = new cVector3d[size];
        
        for(int j=0; j<n; j++) {
            double u = 1.0 - (i+0.)/n;
            double v = 1.0 - (j+0.)/n;
            cVector3d p1 = u*pos[0]+(1.0-u)*pos[1];
            cVector3d p2 = u*pos[2]+(1.0-u)*pos[3];
            position[i][j] = v*p1 + (1.0-v)*p2;
            position_old[i][j] = position[i][j];
            vel[i][j] = cVector3d(0.,0.,0.);
            sphere[i][j] = new cShapeSphere(0.03);
            sphere[i][j]->m_material->setGray();
            sphere[i][j]->m_material->m_specular = cColorf(0.,0.,0.);
            sphere[i][j]->setLocalPos(position[i][j]);
        }
    }
    
    Ln = cSub(pos[0], pos[1]).length()/(n-1);
    mass = 0.01;
    stiffness[0] = 500;//streching
    stiffness[1] = 100;//shearing
    stiffness[2] = 10;//bending
    damping = 0.0;
}

cVector3d Cloth::update(double dt, cShapeSphere* cursor) {
    
    computeForce();
    cVector3d tool_force = collision(cursor);
    updatePosition(cursor, dt);
    
    return cVector3d(0,0,0);
    return tool_force;
}

void Cloth::computeForce() {
    for (int i=0; i<size; i++)
        for (int j=0; j<size; j++) {
            f[i][j] = cVector3d(0.,0.,0.);
            f[i][j] += cVector3d(0,0,-mass*9.81);//gravity
            f[i][j] += -0.1*vel[i][j];//air damping force
            //strech spring forces
            //i-1,j
            if (i>0) {
                cVector3d r = cSub(position[i][j], position[i-1][j]);
                double L = r.length();
                cVector3d u = r/(L+1e-9);
                f[i][j] += -stiffness[0]*(L-Ln)*u;//spring force
                f[i][j] += -damping*vel[i][j].dot(u)*u;//damping force
            }
            //i+1,j
            if (i<size-1) {
                cVector3d r = cSub(position[i][j], position[i+1][j]);
                double L = r.length();
                cVector3d u = r/(L+1e-9);
                f[i][j] += -stiffness[0]*(L-Ln)*u;//spring force
                f[i][j] += -damping*vel[i][j].dot(u)*u;//damping force
            }
            //i,j-1
            if (j>0) {
                cVector3d r = cSub(position[i][j], position[i][j-1]);
                double L = r.length();
                cVector3d u = r/(L+1e-9);
                f[i][j] += -stiffness[0]*(L-Ln)*u;//spring force
                f[i][j] += -damping*vel[i][j].dot(u)*u;//damping force
            }
            //i,j+1
            if (j<size-1) {
                cVector3d r = cSub(position[i][j], position[i][j+1]);
                double L = r.length();
                cVector3d u = r/(L+1e-9);
                f[i][j] += -stiffness[0]*(L-Ln)*u;//spring force
                f[i][j] += -damping*vel[i][j].dot(u)*u;//damping force
            }
            
            //shear spring forces
            if (i>0 && j>0) {
                cVector3d r = cSub(position[i][j], position[i-1][j-1]);
                double L = r.length();
                cVector3d u = r/(L+1e-9);
                f[i][j] += -stiffness[1]*(L-1.414*Ln)*u;//spring force
                f[i][j] += -damping*vel[i][j].dot(u)*u;//damping force
                
            }
            if (i<size-1 && j>0) {
                cVector3d r = cSub(position[i][j], position[i+1][j-1]);
                double L = r.length();
                cVector3d u = r/(L+1e-9);
                f[i][j] += -stiffness[1]*(L-1.414*Ln)*u;//spring force
                f[i][j] += -damping*vel[i][j].dot(u)*u;//damping force
                
            }
            if (i<size-1 && j<size-1) {
                cVector3d r = cSub(position[i][j], position[i+1][j+1]);
                double L = r.length();
                cVector3d u = r/(L+1e-9);
                f[i][j] += -stiffness[1]*(L-1.414*Ln)*u;//spring force
                f[i][j] += -damping*vel[i][j].dot(u)*u;//damping force
                
            }
            if (i>0 && j<size-1) {
                cVector3d r = cSub(position[i][j], position[i-1][j+1]);
                double L = r.length();
                cVector3d u = r/(L+1e-9);
                f[i][j] += -stiffness[1]*(L-1.414*Ln)*u;//spring force
                f[i][j] += -damping*vel[i][j].dot(u)*u;//damping force
                
            }
            //bending spring forces
            if (i>1) {
                cVector3d r = cSub(position[i][j], position[i-2][j]);
                double L = r.length();
                cVector3d u = r/(L+1e-9);
                f[i][j] += -stiffness[2]*(L-2*Ln)*u;//spring force
                f[i][j] += -damping*vel[i][j].dot(u)*u;//damping force
                
            }
            if (i<size-2) {
                cVector3d r = cSub(position[i][j], position[i+2][j]);
                double L = r.length();
                cVector3d u = r/(L+1e-9);
                f[i][j] += -stiffness[2]*(L-2*Ln)*u;//spring force
                f[i][j] += -damping*vel[i][j].dot(u)*u;//damping force
                
            }
            if (j>1) {
                cVector3d r = cSub(position[i][j], position[i][j-2]);
                double L = r.length();
                cVector3d u = r/(L+1e-9);
                f[i][j] += -stiffness[2]*(L-2*Ln)*u;//spring force
                f[i][j] += -damping*vel[i][j].dot(u)*u;//damping force
                
            }
            if (j<size-2) {
                cVector3d r = cSub(position[i][j], position[i][j+2]);
                double L = r.length();
                cVector3d u = r/(L+1e-9);
                f[i][j] += -stiffness[2]*(L-2*Ln)*u;//spring force
                f[i][j] += -damping*vel[i][j].dot(u)*u;//damping force
                
            }
            
        }

    
}
bool Collision(cShapeSphere* s1, cShapeSphere* s2, cVector3d& f1, cVector3d& f2) {
    double x;
    cVector3d v = cSub(s1->getLocalPos(), s2->getLocalPos());
    x = s1->getRadius()+s2->getRadius()-v.length();
    if (x > 1e-5) {
        f1 += 300*v/(v.length()+1e-9)*x;
        f2 += -300*v/(v.length()+1e-9)*x;
        return true;
    }
    return false;
}

cVector3d Cloth::collision(cShapeSphere* cursor) {
    cVector3d tool_force = cVector3d(0.0,0.0,0.0);
     //collision detection
     for (int i=0; i<size; i++)
         for (int j=0; j<size; j++) {
             Collision(sphere[i][j], cursor, f[i][j], tool_force);
     }
    return tool_force;
}
void Cloth::updatePosition(cShapeSphere* cursor, double dt) {
    //numerical integration
    for (int i=0; i<size; i++)
        for (int j=0; j<size; j++) {
            if ((i==0 || i==size-1) && j==size-1 ){
                //position[i][j] = cursor->getLocalPos();
                //sphere[i][j]->setLocalPos(position[i][j]);
                continue;
            }
            else{
                //Verlet Algorithm
                cVector3d pos_new;
                vel[i][j] += f[i][j]/mass*dt;
                cVector3d a = f[i][j]/mass;
                pos_new = 2*position[i][j] - position_old[i][j] + a*dt*dt;
               
                position_old[i][j] = position[i][j];
                position[i][j] = pos_new;
                sphere[i][j]->setLocalPos(position[i][j]);
            }
        }
}



