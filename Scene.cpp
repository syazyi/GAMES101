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
    float pdf_light;
    Intersection lightpoint;
    Intersection point = intersect(ray);
    if (!point.happened)
    {
        return Vector3f();
    }
    if (point.m->hasEmission())
    {
        return  point.m->getEmission();
    }
    sampleLight(lightpoint, pdf_light);

    Vector3f lightpotision = lightpoint.coords;
    Vector3f ws = lightpotision - point.coords;
    Vector3f NN = lightpoint.normal;
    Vector3f emit = lightpoint.emit;
    Vector3f wsnor = ws.normalized();
    float wsPow = dotProduct(ws, ws);
    Ray p2l(point.coords, wsnor);
    Intersection p2lInter = intersect(p2l);

    Vector3f L_dir;

    if (p2lInter.distance - ws.norm() > -0.005f)
    {
        L_dir = emit * point.m->eval(ray.direction, wsnor, point.normal) * dotProduct(wsnor, point.normal) * dotProduct(-wsnor, NN) / std::powf(ws.norm(), 2.f) / pdf_light;
    }

    Vector3f L_indir;

    float randnum = get_random_float();
    if (randnum > RussianRoulette)
    { 
        return L_dir;
    }
    else
    {
        Vector3f wi = point.m->sample(ray.direction, point.normal);
        Ray reflectRay(point.coords, wi.normalized());
        Intersection inter = intersect(reflectRay);
        if (inter.happened && !inter.m->hasEmission())
        {
            L_indir = castRay(reflectRay, depth + 1) * point.m->eval(ray.direction, reflectRay.direction, point.normal) * dotProduct(reflectRay.direction, point.normal) / point.m->pdf(ray.direction, reflectRay.direction, point.normal) / RussianRoulette;
        }
    }
    return L_dir + L_indir;
}