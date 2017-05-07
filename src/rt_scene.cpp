#include "rt_scene.h"
#include <assert.h>

void RtScene::add(RtSphere sph)
{
    list_of_spheres_.push_back(sph);
}

RtSphere RtScene::take_last()
{
    assert(list_of_spheres_.size() > 0);
    list_of_spheres_.pop_back();
}

int RtScene::size()
{
    return list_of_spheres_.size();
}

void RtScene::print()
{
    std::cout << "List of spheres in the scene: " << std::endl;
    for (int i = 0; i < this->size(); i++)
    {
        list_of_spheres_[i].print();
    }
}

RtSphere RtScene::at_index(int i){
	return list_of_spheres_[i];
}