#include "StaticEsdfSim.h"

#include <chrono>
#include <iostream>
#include <random>

std::default_random_engine eng;

void generatePointCloud(
    const float minX, const float maxX,
    const float minY, const float maxY,
    const float minZ, const float maxZ,
    const float res,
    esdfsim::PointCloud<float>& cloud)
{
    cloud.pts.clear();

    // Set floor
    for (float x = minX; x < maxX; x += res)
    {
        for (float y = minY; y < maxY; y += res)
        {
            constexpr float z = 0.0f;
            
            cloud.pts.emplace_back(x, y, z);
        }
    }

    // Set obstacles
    std::cout << "Generating obstacles at:" << std::endl;
    constexpr int numObstacles = 10;
    constexpr float obsWidth = 10.0f;
    std::uniform_real_distribution<float> randX(minX, maxX);
    std::uniform_real_distribution<float> randY(minY, maxY);
    std::uniform_real_distribution<float> randZ(minZ, maxZ);
    for (int i = 0; i < numObstacles; i++)
    {
        const float obsX = randX(eng);
        const float obsY = randY(eng);
        const float obsHeight = randZ(eng);
        std::cout << i << ", x: " << obsX << ", y:" << obsY << std::endl;
        for (float x = obsX - obsWidth / 2; x < obsX + obsWidth / 2; x += res)
        {
            for (float y = obsY - obsWidth / 2; y < obsY + obsWidth / 2; y += res)
            {
                for (float z = 0; z < obsHeight; z += res)
                {
                    cloud.pts.emplace_back(x, y, z);
                }
            }
        }
    }
}

int main(int ac, char** av)
{
    using namespace std::chrono;
    high_resolution_clock clock;

    constexpr float minX = -100.0f;
    constexpr float minY = -100.0f;
    constexpr float minZ = 0.0f;

    constexpr float maxX = 100.0f;
    constexpr float maxY = 100.0f;
    constexpr float maxZ = 50.0f;

    constexpr float res = 0.5f;

    esdfsim::PointCloud<float> cloud;
    generatePointCloud(
        minX, maxX,
        minY, maxY,
        minZ, maxZ,
        res,
        cloud);
    std::cout << "Num cloud points: " << cloud.pts.size() << std::endl;
    
    const auto t0 = clock.now();

    //esdfsim::StaticEsdfSim<float> sim(cloud, res, "esdf.bin"); // compute ESDF on first usage
    esdfsim::StaticEsdfSim<float> sim("esdf.bin"); // read cached ESDF on subsequent usage

    const auto t1 = clock.now();
    const auto duration = duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0f;
    std::cout << "init dur: " << duration << " ms" << std::endl;


    std::uniform_real_distribution<float> randY(minY, maxY);
    std::uniform_real_distribution<float> randZ(minZ, maxZ);
    std::uniform_real_distribution<float> randX(minX, maxX);

    constexpr int numQueries = 100;

    esdfsim::PointCloud<float> queryPoints;
    queryPoints.pts.resize(numQueries);
    for (int i = 0; i < numQueries; i++)
    {
        const float x = randX(eng);
        const float y = randY(eng);
        const float z = randZ(eng);
        queryPoints.pts.at(i) = {x, y, z};
    }

    // query ESDF
    {
        constexpr double range = 7.0;

        const auto t0 = clock.now();

        for (const auto& pt : queryPoints.pts)
        {
            auto res = sim.query(pt.x, pt.y, pt.z, range);
        }

        const auto t1 = clock.now();
        const auto duration = duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0f;
        std::cout << "queryEsdf dur: " << duration << " ms" << std::endl;
        std::cout << "queryEsdf dur average: " << duration / numQueries << " ms" << std::endl;
    }

    return 0;
}