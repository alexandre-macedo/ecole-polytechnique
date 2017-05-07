#include "rt_ray.h"
#include "rt_sphere.h"
#include "rt_scene.h"

int main(){
    // Testing Vector:
    cout << "########## Testing Vector ##########" << endl;
    RtVector dir(1.0, 1.0, 1.0);
    RtVector dir2(2.0, 2.0, 2.0);
    dir.print();
    dir += dir2;
    dir.print();
    // dir.print();n


    // Testing Ray:
    cout << "########## Testing Ray ##########" << endl;
    RtRay ray(2.0, 2.0, 2.0, dir); 
    ray.print();

    // Testing Spheres:
    cout << "########## Testing Spheres ##########" << endl;
    RtSphere sph (0., 0., 0., 1., 0, 0, 0);
    sph.print();

    // Testing Scenes:
    cout << "########## Testing Scenes ##########" << endl;
    RtSphere sph2 (0., 0., 0., 3., 37, 73, 10);
    RtScene sc;
    sc.add(sph); sc.add(sph2);
    //sc.take_last();
    sc.print();
}