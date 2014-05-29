/*
    Copyright (c) 2014, <copyright holder> <email>
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include "pcl_octree_impl.h"
#include <pthread.h>

static pthread_mutex_t returnPointMutex;

PCL_OCTREE_IMPL::PCL_OCTREE_IMPL(int width, int height)
{
  pCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pCloud->width = width;
  pCloud->height = height;
  pCloud->points.resize(0);

  octree = new pcl::octree::OctreePointCloud<pcl::PointXYZ>(0.25);
  octree->setInputCloud(pCloud);
  octree->addPointsFromInputCloud();
  
//   octree = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(128.0f);
//   octree->defineBoundingBox(0.1, 0.1, 0.1);
}

PCL_OCTREE_IMPL::~PCL_OCTREE_IMPL()
{
    delete octree;
}

void PCL_OCTREE_IMPL::AddPoint(double x, double y, double z)
{
    pcl::PointXYZ searchPt = pcl::PointXYZ(x, y, z);

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSearchDistance;

    octree->addPointToCloud(searchPt, pCloud);

//    int search = octree->radiusSearch(searchPt, 0.1, pointIdxRadiusSearch, pointRadiusSearchDistance);
//    if(search > 0)
//    {
//    }
//    else
//    {
//        pCloud->points.push_back(pcl::PointXYZ(x, y, z));
//    }
    std::cout << pCloud->points.size() << "\t";
}

void PCL_OCTREE_IMPL::DownSample(float voxelResolution)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
  sor.setInputCloud(pCloud);
  sor.setLeafSize(voxelResolution, voxelResolution, voxelResolution);
  sor.filter(*pCloudFiltered);
  pCloud = pCloudFiltered;
}


std::vector< pcl::PointXYZ > PCL_OCTREE_IMPL::GetPoints()
{
  std::vector<pcl::PointXYZ> pts(pCloud->points.size());
  pthread_mutex_lock(&returnPointMutex);
  std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > copyPoints(pCloud->points);
  for(int i = 0; i < copyPoints.size(); i++)
  {
    pts[i] = copyPoints[i];
  }
  pthread_mutex_unlock(&returnPointMutex);
  return pts;
  
}
