// Methods / tools for the project

#include <stdexcept>
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
        if(pt.distanceTo(sph.getCenter())!=sph.getRadius()){
            throw std::runtime_error("Point must be in the surface of the sphere in colorOfPoint.");
        }
        
        double ks, kd, ka, alpha;
        int ra, ga, ba; // red, green and blue of the ambient
        //int rl, gl, bl; // red, green and blue of Light
        int rs, gs, bs; // red, green and blue of Sphere
        int rp, gp, bp; // same for our point
        RtColor cSphere; // color of the sphere

        RtVector V = ((-1)*viewer).unit(); // viewer
        RtVector N = (pt-sph.getCenter()).unit(); // normal
        RtVector L = (light.getPoint() - pt).unit(); // light
        RtColor colorOfLight = light.getColor();
        RtVector R = (2*(L*N)*N-L).unit(); // reflection direction

        // Constants
        ks=0.25;
        kd=0.5;
        ka=0.5;
        alpha=2.;

        // Color of my ambient
        ra=ga=ba=50;

        // We will use the light and its color later
        //rl = colorOfLight.getR();
        //gl = colorOfLight.getG();
        //bl = colorOfLight.getB();
        
        cSphere = sph.getColor();
        rs = cSphere.getR();
        gs = cSphere.getG();
        bs = cSphere.getB(); 

        rp = ka*ra;
        gp = ka*ga;
        bp = ka*ba;
        if((L*N)>0.){
            // Percentage for diffusion
            double pForDiffusion = .7;

            rp += kd*(L*N)*(pForDiffusion*rs);
            gp += kd*(L*N)*(pForDiffusion*gs);
            bp += kd*(L*N)*(pForDiffusion*bs);
            if((R*V)>0.){
                rp += ks * pow(R*V, alpha) * (1-pForDiffusion)*rs;
                gp += ks * pow(R*V, alpha) * (1-pForDiffusion)*gs;
                bp += ks * pow(R*V, alpha) * (1-pForDiffusion)*bs;
            }
        }

        return RtColor(rp, gp, bp); // the point's color
    }

}