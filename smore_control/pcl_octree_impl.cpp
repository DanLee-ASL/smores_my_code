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
	octoMapTree = new octomap::OcTree(0.025);
}

PCL_OCTREE_IMPL::~PCL_OCTREE_IMPL()
{
//     delete octree;
	std::cout << "Writing to tree.bt" << std::endl;
	octoMapTree->writeBinary("tree.bt");
	std::cout << "Done writing." << std::endl;
	delete octoMapTree;
}

void print_query_info(octomap::point3d query, octomap::OcTreeNode* node) {
  if (node != NULL) {
    std::cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << std::endl;
  }
  else 
    std::cout << "occupancy probability at " << query << ":\t is unknown" << std::endl;    
}

void PCL_OCTREE_IMPL::AddPoint(double x, double y, double z)
{
	octomap::point3d pt(x, y, z);
	octoMapTree->updateNode(pt, true);
}

void PCL_OCTREE_IMPL::AddScan(double* xPts, double* yPts, double* zPts, int nPoints, double poseX, double poseY, double poseZ)
{
	octomap::Pointcloud scan;
	for(int i = 0; i < nPoints; i++)
	{
		scan.push_back(xPts[i], yPts[i], zPts[i]);
		pthread_mutex_lock(&returnPointMutex);
		AddPoint(xPts[i], yPts[i], zPts[i]);
		pthread_mutex_unlock(&returnPointMutex);
	}
// 	octoMapTree->insertScan(scan, octomap::point3d(poseX, poseY, poseZ), 10);
// 	pthread_mutex_unlock(&returnPointMutex);
	ocGridUpdatedSignal();
}


void PCL_OCTREE_IMPL::DownSample(float voxelResolution)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
  sor.setInputCloud(pCloud);
  sor.setLeafSize(voxelResolution, voxelResolution, voxelResolution);
  sor.filter(*pCloudFiltered);
  std::cout << pCloud->points.size() << "\t";
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

std::vector<octomap::point3d> PCL_OCTREE_IMPL::GetVoxels(int max_tree_depth, float &cubeSize)
{
	std::vector<octomap::point3d> occupiedVoxels;
	pthread_mutex_lock(&returnPointMutex);
	for(octomap::OcTree::tree_iterator it = octoMapTree->begin_tree(max_tree_depth), end = octoMapTree->end_tree(); it!= end; ++it)
	{
		if(it.isLeaf())
		{
			if(octoMapTree->isNodeOccupied(*it))
			{
				occupiedVoxels.push_back(octomap::point3d(it.getX(), it.getY(), it.getZ()));
			}
		}
	}
	pthread_mutex_unlock(&returnPointMutex);
	cubeSize = octoMapTree->getResolution();
	return occupiedVoxels;
}
