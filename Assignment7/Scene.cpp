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
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    // initialize direct light and indirect light
    Vector3f ldir = { 0, 0, 0 };
    Vector3f lindir = { 0, 0, 0 };
    
    Intersection objectInter = intersect(ray);
    if (!objectInter.happened)
    {
        return {};
    }
    // ray cast to light source, return light emission
    if (objectInter.m->hasEmission())
    {
        return objectInter.m->getEmission();
    }
    // ray cast to object then sample light uniformly
    Intersection lightInter;
    float lightPdf = 0.0f;
    sampleLight(lightInter, lightPdf);//acquire light sample, including light position and sample pdf

    // cal variables below to prepare cal direct light
    Vector3f obj2light = lightInter.coords - objectInter.coords;
    Vector3f obj2lightdir = obj2light.normalized();
    float distancePow2 = obj2light.x * obj2light.x + obj2light.y * obj2light.y + obj2light.z * obj2light.z;

    Ray obj2lightray = Ray{ objectInter.coords, obj2lightdir };
    Intersection t = intersect(obj2lightray);
	if (t.distance - obj2light.norm() > -EPSILON)// No obstruct means this light source is reachable
    {
        //L_dir = L_i * f_r * cos_theta * cos_theta_x / |x - p | ^ 2 / pdf_light
        ldir = lightInter.emit * objectInter.m->eval(ray.direction, obj2lightdir, objectInter.normal) * dotProduct(obj2lightdir, objectInter.normal) * dotProduct(-obj2lightdir, lightInter.normal) / distancePow2 / lightPdf;
    }
    // for indirect light use RR（Russian Roulette Algorithm) to determine whether continue to sample
    if (get_random_float() > RussianRoulette)
    {
        return ldir;
    }
    // get a random direction in semisphere
    Vector3f obj2nextobjdir = objectInter.m->sample(ray.direction, objectInter.normal).normalized();
    // create a ray
    Ray obj2nextobjray = { objectInter.coords, obj2nextobjdir };
    // get object intersection
    Intersection nextObjInter = intersect(obj2nextobjray);
    // if intersect happen and intersection object is not light source
    if (nextObjInter.happened && !nextObjInter.m->hasEmission())
    {
        // cal probability density function
        float pdf = objectInter.m->pdf(ray.direction, obj2nextobjdir, objectInter.normal);
        // shade(q, wi) * f_r * cos_theta / pdf_hemi / P_RR
        lindir = castRay(obj2nextobjray, depth + 1) * objectInter.m->eval(ray.direction, obj2nextobjdir, objectInter.normal) * dotProduct(obj2nextobjdir, objectInter.normal) / pdf / RussianRoulette;
    }
    return ldir + lindir;
}
