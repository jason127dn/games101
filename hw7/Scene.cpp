//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection int_one = Scene::intersect(ray);
    Vector3f L_dir;
    Vector3f L_indir;

    if (int_one.happened)
    {
        if(int_one.m->hasEmission()&&depth==0)
        {
            return int_one.emit;
        }
        //std::cout<<"in happened\n";
        Vector3f p1 = int_one.coords;
        Material *m = int_one.m;
        Vector3f N = int_one.normal;
        Intersection hitsec;
        float pdf;
        sampleLight(hitsec,pdf);
        Ray r(p1,(hitsec.coords-p1).normalized());
        Intersection intL = Scene::intersect(r);
        if (intL.distance>(hitsec.coords-p1).norm()-EPSILON && hitsec.emit.norm()>EPSILON)
        {
            Vector3f wi = (p1-hitsec.coords).normalized();
            Vector3f wo = (ray.origin - p1).normalized();

            //std::cout << "th2:" << wi <<"\n";
            L_dir = hitsec.emit * m->eval(wi,wo,N)*dotProduct(-wi,N)*dotProduct(hitsec.normal,wi)/pow((hitsec.coords-p1).norm(),2.)/pdf; 
            //std::cout<< L_dir <<"\n";
        }
        float rr = get_random_float();
        if (rr < RussianRoulette)
        {
            Vector3f wi;
            Vector3f wo = m->sample(wi,N);
            L_indir= castRay(Ray(p1,wo),depth+1)*m->eval(wi,wo,N)*dotProduct(wo,N)/m->pdf(wi,wo,N) /RussianRoulette;
        }
    }
    return L_dir+L_indir;
}
