#include <algorithm>
#include <assert.h>

#include "rt_scene.h"

RtScene::RtScene() {}

RtScene::~RtScene() {}

void RtScene::add(RtSphere sph) { list_of_spheres_.push_back(sph); }

RtSphere RtScene::take_last() {
  assert(list_of_spheres_.size() > 0);
  list_of_spheres_.pop_back();
}

int RtScene::size() { return list_of_spheres_.size(); }

RtSphere RtScene::at_index(int i) { return list_of_spheres_[i]; }

std::ostream &operator<<(std::ostream &s, RtScene &sc) {
  s << "{";
  for (int i = 0; i < sc.list_of_spheres_.size() - 1; i++)
    s << sc.list_of_spheres_[i] << ", ";
  s << sc.list_of_spheres_[sc.list_of_spheres_.size() - 1] << "}";
  return s;
}

// Strictly smaller than
bool lessThan(RtSphere &s1, RtSphere &s2, RtVector &ref){
    RtVector c1 = s1.getCenter();
    double r1 = s1.getRadius();
    RtVector c2 = s2.getCenter();
    double r2 = s2.getRadius();
    double d1 = c1.distanceTo(ref)-r1;
    //assert(d1 >= 0);
    double d2 = c2.distanceTo(ref)-r2;
    //assert(d2 >= 0);
    //std::cout << "d1 and d2 : " << d1 << " , " << d2 << " bool menor q: " << (d1<d2)<< std::endl;
    return (d1 < d2);
}

//Strictly greater than
bool greaterThan(RtSphere &s1, RtSphere &s2, RtVector &ref){
    RtVector c1 = s1.getCenter();
    double r1 = s1.getRadius();
    RtVector c2 = s2.getCenter();
    double r2 = s2.getRadius();
    double d1 = c1.distanceTo(ref)-r1;
    //assert(d1 >= 0);
    double d2 = c2.distanceTo(ref)-r2;
    //assert(d2 >= 0);
    //std::cout << "d1 and d2 : " << d1 << " , " << d2 << " bool maior q: " << (d1>d2)<< std::endl;
    return (d1 > d2);
}

void quickSort(RtSphere* arr, int left, int right, RtVector &ref) {
      int i = left, j = right;
      RtSphere tmp; // RtVector centertmp; RtColor colortmp; double radiustmp; 
      RtSphere pivot = arr[(left + right) / 2];
 
      /* partition */
      while (i <= j) {
            while (lessThan(arr[i], pivot, ref)) i++;
            while (greaterThan(arr[j], pivot, ref)) j--;
            if (i <= j) {
                  //centertmp = arr[i].getCenter();
                  //colortmp = arr[i].getColor();
                  //radiustmp = arr[i].getRadius();
                  //RtSphere tmp (centertmp, radiustmp, colortmp); //tmp = arr[i];
                  tmp.copyHere(arr[i]);
                  arr[i].copyHere(arr[j]);
                  arr[j].copyHere(tmp); //RtSphere(centertmp, radiustmp, colortmp);
                  i++;
                  j--;
            }
      }
 
      /* recursion */
      if (left < j) quickSort(arr, left, j, ref);
      if (i < right) quickSort(arr, i, right, ref);
}

void RtScene::sort(RtVector &ref) {
  quickSort(&list_of_spheres_[0], 0, list_of_spheres_.size()-1, ref);
  //std::sort(list_of_spheres_.begin(), list_of_spheres_.end(), less_than_key());
}