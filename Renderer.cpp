//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include<mutex>
#include<thread>
#include "Scene.hpp"
#include "Renderer.hpp"


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    int process = 0;
    std::mutex mutex_ins;

    const int block_x = 5;
    const int block_y = 5;
    std::thread th[block_x * block_y];

    int strideX = scene.width / block_x + 1;
    int strideY = scene.height / block_y + 1;

    // change the spp value to change sample ammount
    int spp = 16;
    std::cout << "SPP: " << spp << "\n";
    auto CastRayMultiThread = [&](int min_x, int min_y, int max_x, int max_y)
    {
        for (uint32_t j = min_y; j < max_y; ++j)
        {
            int m = j * scene.width + min_x;
            for (uint32_t i = min_x; i < max_x; ++i) {
                // generate primary ray direction
                float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                    imageAspectRatio * scale;
                float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

                Vector3f dir = normalize(Vector3f(-x, y, 1));
                for (int k = 0; k < spp; k++) {
                    framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
                }
                m++;
            }
            {
                std::lock_guard<std::mutex> lock(mutex_ins);
                ++process;//process值在x轴上循环五次，block次
                UpdateProgress((float)process / scene.height / block_x);
                std::cout << process;
            }
        }
    };
    int id = 0;
    for (int j = 0; j < block_y; j++)
    {
        for (int i = 0; i < block_x; i++)
        {
            th[id] = std::thread(CastRayMultiThread, i * strideX, j * strideY, std::min(scene.width, (i + 1) * strideX), std::min(scene.height, (j + 1) * strideY));
            id++;
        }
    }
    for (int i = 0; i < block_x * block_y; i++)
    {
        th[i].join();
    }
    UpdateProgress(1.f);
    /*
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            // generate primary ray direction
            float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                      imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

            Vector3f dir = normalize(Vector3f(-x, y, 1));
            for (int k = 0; k < spp; k++){
                framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
            }
            m++;
        }
        UpdateProgress(j / (float)scene.height);
    }
    UpdateProgress(1.f);
    */
    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
