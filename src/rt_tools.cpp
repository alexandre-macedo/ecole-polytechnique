// Methods / tools for the project

#include <cmath>
#include <math.h> 

#include "rt_tools.h"

namespace RtTools{
    bool intersection(RtSphere &sph, RtRay &r, RtVector &i){
        RtVector center = sph.getCenter(); double radius = sph.getRadius();
        RtVector dir = r.getDirection();
        dir = dir.unit(); // norm == 1 now
        RtVector p = r.getOrigin();
        double d = ((dir.cross(center-p)).norm2());
        
        if(d > radius){
            return false;
        }
        
        double hip = (center-p).norm2();
        double cateto = sqrt(hip*hip - d*d);
        double correction = sqrt(radius*radius - d*d);
        // Updating intersection point 
        //std::cout << "p: " << p << " dir: " << dir << " cat: " << cateto << " dist: " << d << std::endl;;
        i = p + dir*(cateto-correction);

        return true;
    }


    // "viewer" is the vector from the camera pointing towards the point 
    // using Phong Reflection Model - https://en.wikipedia.org/wiki/Phong_reflection_model
    RtColor colorOfPoint(RtVector &pt, RtSphere &sph, RtVector &viewer, RtLight &light){
        double ks, kd, ka, alpha;
        int ra, ga, ba; // red, green and blue of the ambient
        int rl, gl, bl; // red, green and blue of Light
        int rp, gp, bp; // same for our point

        RtVector V = ((-1)*viewer).unit(); // viewer
        RtVector N = (pt-sph.getCenter()).unit(); // normal
        RtVector L = (light.getPoint() - pt).unit(); // light
        RtColor colorOfLight = light.getColor();
        RtVector R = (2*(L*N)*N-L).unit(); // reflection direction

        // Constants
        ks=0.5;
        kd=0.5;
        ka=0.5;
        alpha=0.5;

        // Color of my ambient
        ra=ga=ba=50;

        rl = colorOfLight.getR();
        gl = colorOfLight.getG();
        bl = colorOfLight.getB();

        // Percentage for diffusion
        double pForDiffusion = 0.7;

        rp = ka*ra;
        gp = ka*ga;
        bp = ka*ba;
        if((L*N)>0.){
            rp += kd*(L*N)*(pForDiffusion*rl);
            gp += kd*(L*N)*(pForDiffusion*gl);
            bp += kd*(L*N)*(pForDiffusion*bl);
            if((R*V)>0.){
                rp += ks * pow(R*V, alpha) * (1-pForDiffusion)*rl;
                gp += ks * pow(R*V, alpha) * (1-pForDiffusion)*gl;
                bp += ks * pow(R*V, alpha) * (1-pForDiffusion)*bl;
            }
        }

        return RtColor(rp, gp, bp); // the point's color
    }

}