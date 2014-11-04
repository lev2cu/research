#ifndef CCNY_GICP_OCTREE_POINTCLOUD_STORAGE_HPP
#define CCNY_GICP_OCTREE_POINTCLOUD_STORAGE_HPP

#include "ccny_gicp/octree_pointcloud_storage.h"

namespace pcl
{

namespace octree
{

template <typename PointT, typename LeafT, typename OctreeT>
OctreePointCloudStorage<PointT, LeafT, OctreeT>::OctreePointCloudStorage(double resolution):
  OctreePointCloudSearch<PointT, LeafT, OctreeT>(resolution)
{

}

template<typename PointT, typename LeafT, typename OctreeT>
OctreePointCloudStorage<PointT, LeafT, OctreeT>::~OctreePointCloudStorage()
{

}

template<typename PointT, typename LeafT, typename OctreeT>
bool OctreePointCloudStorage<PointT, LeafT, OctreeT>::addPointWithReplacement(const PointT& point,
                                                                                    PointCloudTPtr cloud,
                                                                                    int& index)
{

  // generate key
  OctreeKey key;     
  genOctreeKeyforPoint (point, key);

  LeafT* leaf = findLeaf(key);

  if (leaf == 0)
  {
    // not found - create new one

    addPointToCloud(point, cloud);

    index = cloud->points.size() - 1;
    return false;
  }
  else
  {
    // found - update

    const int* i;
    leaf->getData(i);
  
    cloud->points[*i] = point;

    index = *i;
    return true;
  }
}

} // namespace octree

} // namespace pcl

#endif // CCNY_GICP_OCTREE_POINTCLOUD_STORAGE_HPP
