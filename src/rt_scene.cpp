#include "rt_scene.h"
#include <assert.h>

void RtScene::add(RtSphere sph){
    list_of_spheres.push_back(sph);
}

RtSphere RtScene::take_last(){
    assert(list_of_spheres.size()>0);
    list_of_spheres.pop_back();
}

int RtScene::size(){
    return list_of_spheres.size();
}

void RtScene::print(){
    cout << "List of spheres in the scene: " << endl ;
    for(int i=0; i<this->size(); i++){
        list_of_spheres[i].print();
    }
}