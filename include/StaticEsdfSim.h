#pragma once

#include "PointCloud.h"

#include <algorithm>
#include <functional>
#include <string>
#include <memory>
#include <sstream>
#include <vector>

#include "bonxai/bonxai.hpp"
#include "bonxai/serialization.hpp"
#include "nanoflann/nanoflann.hpp"

namespace esdfsim
{
template<typename T>
class StaticEsdfSim
{
public:
    explicit StaticEsdfSim(const std::string filepath);
    StaticEsdfSim(const PointCloud<T>& cloud, const double resolution);
    StaticEsdfSim(const PointCloud<T>& cloud, const double resolution, const std::string filepath);

    void initKdTree(const PointCloud<T>& cloud);
    void initVoxelGrid(const PointCloud<T>& cloud, const double resolution);

    struct EsdfResult
    {
        EsdfResult(const Point<T> pt, const double d) : point(pt), dist(d)
        {};

        Point<T> point;
        double dist;
    };

    std::vector<EsdfResult> query(const double x, const double y, const double z, const double range) const;

private:
    T queryPoint(const T x, const T y, const T z);
    void saveGrid(const std::string filepath);
    void readGrid(const std::string filepath);

    using KdTree = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<T, PointCloud<T>>,
        PointCloud<T>,
        3 // dim
    >;    
    std::unique_ptr<KdTree> mKdTree; // for nearest point look-up
    std::unique_ptr<Bonxai::VoxelGrid<T>> mGrid; // for caching distance values and checking line-of-sight
};

template<typename T>
StaticEsdfSim<T>::StaticEsdfSim(const std::string filepath)
{
    readGrid(filepath);
}

template<typename T>
StaticEsdfSim<T>::StaticEsdfSim(const PointCloud<T>& cloud, const double resolution)
{
    initKdTree(cloud);
    initVoxelGrid(cloud, resolution);
}

template<typename T>
StaticEsdfSim<T>::StaticEsdfSim(const PointCloud<T>& cloud, const double resolution, const std::string filepath)
{
    initKdTree(cloud);
    initVoxelGrid(cloud, resolution);
    saveGrid(filepath);
}

template<typename T>
void StaticEsdfSim<T>::initKdTree(const PointCloud<T>& cloud)
{
    mKdTree.reset();
    mKdTree = std::make_unique<KdTree>(
        3,  // dim
        cloud
    );
}

template<typename T>
void StaticEsdfSim<T>::initVoxelGrid(const PointCloud<T>& cloud, const double resolution)
{
    mGrid = std::make_unique<Bonxai::VoxelGrid<T>>(resolution);

    // Find pointcloud bounds
    Point<T> min {std::numeric_limits<T>::max(), std::numeric_limits<T>::max(), std::numeric_limits<T>::max()};
    Point<T> max {std::numeric_limits<T>::lowest(), std::numeric_limits<T>::lowest(), std::numeric_limits<T>::lowest()};
    for (const Point<T>& pt : cloud.pts)
    {
        min = {
            std::min(pt.x, min.x),
            std::min(pt.y, min.y),
            std::min(pt.z, min.z)};
        max = {
            std::max(pt.x, max.x),
            std::max(pt.y, max.y),
            std::max(pt.z, max.z)};
    }

    // Precompute ESDF values
    typename Bonxai::VoxelGrid<T>::Accessor accessor = mGrid->createAccessor();
    Bonxai::CoordT minCoord = mGrid->posToCoord(min.x, min.y, min.z);
    Bonxai::CoordT maxCoord = mGrid->posToCoord(max.x, max.y, max.z);
    for (int32_t x = minCoord.x; x <= maxCoord.x; x++)
    {
        for (int32_t y = minCoord.y; y <= maxCoord.y; y++)
        {
            for (int32_t z = minCoord.z; z <= maxCoord.z; z++)
            {
                const Bonxai::CoordT coord {x, y, z};
                const Bonxai::Point3D pt = mGrid->coordToPos(coord);
                const T dist = queryPoint(pt.x, pt.y, pt.z);
                accessor.setValue(coord, dist);
            }
        }
    }
}

template<typename T>
std::vector<typename StaticEsdfSim<T>::EsdfResult> StaticEsdfSim<T>::query(const double x, const double y, const double z, const double range) const
{
    typename Bonxai::VoxelGrid<T>::ConstAccessor accessor = mGrid->createConstAccessor();
    const double resolution = mGrid->voxelSize();
    const size_t maxQueries = static_cast<size_t>(std::pow(std::ceil(2.0 * range / resolution), 3));
    std::vector<EsdfResult> result;
    result.reserve(maxQueries);

    constexpr double epsilon = 1e-3;
    for (double dx = -range; dx < range + epsilon; dx += resolution)
    {
        for (double dy = -range; dy < range + epsilon; dy += resolution)
        {
            for (double dz = -range; dz < range + epsilon; dz += resolution)
            {
                const Bonxai::CoordT coord = mGrid->posToCoord(x + dx, y + dy, z + dz);
                const T* val = accessor.value(coord);

                if (val)
                {
                    result.emplace_back(Point<T>(x, y, z), *val);
                }
            }
        }
    }

    return result;
}

template<typename T>
T StaticEsdfSim<T>::queryPoint(const T x, const T y, const T z)
{
    constexpr T invalidDist = static_cast<T>(0);

    constexpr size_t k = 1;
    const T point[3] = {x, y, z};
    std::vector<uint32_t> retIndex(k);
    std::vector<T> outDistSqr(k);
    const size_t hasResult = mKdTree->knnSearch(
        &point[0], k, &retIndex[0], &outDistSqr[0]);

    return hasResult ? outDistSqr[0] : invalidDist;
}

template<typename T>
void StaticEsdfSim<T>::saveGrid(const std::string filepath)
{
    std::ofstream file(filepath, std::ios::binary);
    std::ostringstream oss(std::ios::binary);
    Bonxai::Serialize(oss, *mGrid);
    file << oss.str();
    file.close();
}

template<typename T>
void StaticEsdfSim<T>::readGrid(const std::string filepath)
{
    std::ifstream file(filepath, std::ios::binary);
    std::stringstream iss;
    iss << file.rdbuf();
    char header[256];
    iss.getline(header, 256);
    Bonxai::HeaderInfo info = Bonxai::GetHeaderInfo(header);
    mGrid = std::make_unique<Bonxai::VoxelGrid<T>>(Bonxai::Deserialize<T>(iss, info));
    file.close();
}
} // namespace esdfsim