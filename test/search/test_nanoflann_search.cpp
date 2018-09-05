/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

#include <iostream>
#include <gtest/gtest.h>
#include <pcl/common/distances.h>
#include <pcl/common/time.h>
#include <pcl/search/pcl_search.h>
#include <pcl/search/nanoflann_search.h>
#include <pcl/search/impl/nanoflann_search.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace pcl;

PointCloud<PointXYZ> cloud, cloud_big;

void
init ()
{
  float resolution = 0.1f;
  for (float z = -0.5f; z <= 0.5f; z += resolution)
    for (float y = -0.5f; y <= 0.5f; y += resolution)
      for (float x = -0.5f; x <= 0.5f; x += resolution)
        cloud.points.push_back (PointXYZ (x, y, z));
  cloud.width = int (cloud.points.size ());
  cloud.height = 1;

  cloud_big.width = 640;
  cloud_big.height = 480;
  srand (int (time (NULL)));
  // Randomly create a new point cloud
  for (size_t i = 0; i < cloud_big.width * cloud_big.height; ++i)
    cloud_big.points.push_back (
                                PointXYZ (
                                  float (1024 * rand () / (RAND_MAX + 1.0)),
                                  float (1024 * rand () / (RAND_MAX + 1.0)),
                                  float (1024 * rand () / (RAND_MAX + 1.0))));
}


/* Test for NanoFlannSearch nearestKSearch */
TEST (PCL, NanoFlannSearch_nearestKSearch)
{
  pcl::search::Search<PointXYZ>* NanoFlannSearch = new pcl::search::NanoFlannSearch<PointXYZ> ();
  NanoFlannSearch->setInputCloud (cloud.makeShared ());
  PointXYZ test_point (0.01f, 0.01f, 0.01f);
  unsigned int no_of_neighbors = 20;
  multimap<float, int> sorted_brute_force_result;
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    float distance = euclideanDistance (cloud.points[i], test_point);
    sorted_brute_force_result.insert (make_pair (distance, int (i)));
  }
  float max_dist = 0.0f;
  unsigned int counter = 0;
  for (multimap<float, int>::iterator it = sorted_brute_force_result.begin (); it != sorted_brute_force_result.end ()
      && counter < no_of_neighbors; ++it)
  {
    max_dist = max (max_dist, it->first);
    ++counter;
  }

  vector<int> k_indices;
  k_indices.resize (no_of_neighbors);
  vector<float> k_distances;
  k_distances.resize (no_of_neighbors);

  NanoFlannSearch->nearestKSearch (test_point, no_of_neighbors, k_indices, k_distances);

  //if (k_indices.size () != no_of_neighbors)  cerr << "Found "<<k_indices.size ()<<" instead of "<<no_of_neighbors<<" neighbors.\n";
  EXPECT_EQ (k_indices.size (), no_of_neighbors);

  // Check if all found neighbors have distance smaller than max_dist
  for (size_t i = 0; i < k_indices.size (); ++i)
  {
    const PointXYZ& point = cloud.points[k_indices[i]];
    bool ok = euclideanDistance (test_point, point) <= max_dist;
    if (!ok)
    ok = (fabs (euclideanDistance (test_point, point)) - max_dist) <= 1e-6;
    //if (!ok)  cerr << k_indices[i] << " is not correct...\n";
    //else      cerr << k_indices[i] << " is correct...\n";
    EXPECT_EQ (ok, true);
  }

  ScopeTime scopeTime ("NANOFLANN nearestKSearch");
  {
    pcl::search::Search<PointXYZ>* NanoFlannSearch = new pcl::search::NanoFlannSearch<PointXYZ>();
    //NanoFlannSearch->initSearchDS ();
    NanoFlannSearch->setInputCloud (cloud_big.makeShared ());
    for (size_t i = 0; i < cloud_big.points.size (); ++i)
      NanoFlannSearch->nearestKSearch (cloud_big.points[i], no_of_neighbors, k_indices, k_distances);
  }
}

/* Test the templated NN search (for different query point types) */
TEST (PCL, NanoFlannSearch_differentPointT)
{

  unsigned int no_of_neighbors = 20;

  pcl::search::Search<PointXYZ>* NanoFlannSearch = new pcl::search::NanoFlannSearch<PointXYZ> ();
  //NanoFlannSearch->initSearchDS ();
  NanoFlannSearch->setInputCloud (cloud_big.makeShared ());

  PointCloud<PointXYZRGB> cloud_rgb;

  copyPointCloud (cloud_big, cloud_rgb);



  std::vector< std::vector< float > > dists;
  std::vector< std::vector< int > > indices;
  NanoFlannSearch->nearestKSearchT (cloud_rgb, std::vector<int> (),no_of_neighbors,indices,dists);

  vector<int> k_indices;
  k_indices.resize (no_of_neighbors);
  vector<float> k_distances;
  k_distances.resize (no_of_neighbors);

  //vector<int> k_indices_t;
  //k_indices_t.resize (no_of_neighbors);
  //vector<float> k_distances_t;
  //k_distances_t.resize (no_of_neighbors);

  for (size_t i = 0; i < cloud_rgb.points.size (); ++i)
  {
    //NanoFlannSearch->nearestKSearchT (cloud_rgb.points[i], no_of_neighbors, k_indices_t, k_distances_t);
    NanoFlannSearch->nearestKSearch (cloud_big.points[i], no_of_neighbors, k_indices, k_distances);
    EXPECT_EQ (k_indices.size (), indices[i].size ());
    EXPECT_EQ (k_distances.size (), dists[i].size ());
    for (size_t j = 0; j< no_of_neighbors; j++)
    {
      EXPECT_TRUE (k_indices[j] == indices[i][j] || k_distances[j] == dists[i][j]);
      //EXPECT_TRUE (k_indices[j] == k_indices_t[j]);
      //EXPECT_TRUE (k_distances[j] == k_distances_t[j]);
    }

  }
}

/* Test for NanoFlannSearch nearestKSearch with multiple query points */
TEST (PCL, NanoFlannSearch_multipointKnnSearch)
{

  unsigned int no_of_neighbors = 20;


  pcl::search::Search<PointXYZ>* NanoFlannSearch = new pcl::search::NanoFlannSearch<PointXYZ> ();
  //NanoFlannSearch->initSearchDS ();
  NanoFlannSearch->setInputCloud (cloud_big.makeShared ());

  std::vector< std::vector< float > > dists;
  std::vector< std::vector< int > > indices;
  NanoFlannSearch->nearestKSearch (cloud_big, std::vector<int>(),no_of_neighbors,indices,dists);

  vector<int> k_indices;
  k_indices.resize (no_of_neighbors);
  vector<float> k_distances;
  k_distances.resize (no_of_neighbors);

  for (size_t i = 0; i < cloud_big.points.size (); ++i)
  {
    NanoFlannSearch->nearestKSearch (cloud_big.points[i], no_of_neighbors, k_indices, k_distances);
    EXPECT_EQ (k_indices.size (), indices[i].size ());
    EXPECT_EQ (k_distances.size (), dists[i].size ());
    for (size_t j = 0; j< no_of_neighbors; j++ )
    {
      EXPECT_TRUE (k_indices[j] == indices[i][j] || k_distances[j] == dists[i][j]);
    }

  }
}

/* Test for NanoFlannSearch nearestKSearch with multiple query points */
TEST (PCL, NanoFlannSearch_knnByIndex)
{

  unsigned int no_of_neighbors = 3;


  pcl::search::Search<PointXYZ>* nanoflann_search = new pcl::search::NanoFlannSearch<PointXYZ> ();
  //NanoFlannSearch->initSearchDS ();
  nanoflann_search->setInputCloud (cloud_big.makeShared ());

  std::vector< std::vector< float > > dists;
  std::vector< std::vector< int > > indices;
  std::vector< int > query_indices;
  for (size_t i = 0; i<cloud_big.size (); i+=2)
  {
    query_indices.push_back (int (i));
  }
  nanoflann_search->nearestKSearch (cloud_big, query_indices,no_of_neighbors,indices,dists);

  vector<int> k_indices;
  k_indices.resize (no_of_neighbors);
  vector<float> k_distances;
  k_distances.resize (no_of_neighbors);

  for (size_t i = 0; i < query_indices.size (); ++i)
  {
    nanoflann_search->nearestKSearch (cloud_big[2*i], no_of_neighbors, k_indices, k_distances);
    EXPECT_EQ (k_indices.size (), indices[i].size ());
    EXPECT_EQ (k_distances.size (), dists[i].size ());
    for (size_t j = 0; j< no_of_neighbors; j++)
    {
      EXPECT_TRUE (k_indices[j] == indices[i][j] || k_distances[j] == dists[i][j]);
    }
    nanoflann_search->nearestKSearch (cloud_big,query_indices[i], no_of_neighbors, k_indices, k_distances);
    EXPECT_EQ (k_indices.size (), indices[i].size ());
    EXPECT_EQ (k_distances.size (), dists[i].size ());
    for (size_t j = 0; j< no_of_neighbors; j++)
    {
      EXPECT_TRUE (k_indices[j] == indices[i][j] || k_distances[j] == dists[i][j]);
    }

  }
}


/* Test for NanoFlannSearch nearestKSearch */
TEST (PCL, NanoFlannSearch_compareToKdTreeNanoFlann)
{

  int no_of_neighbors=3;
  vector<int> k_indices;
  k_indices.resize (no_of_neighbors);
  vector<float> k_distances;
  k_distances.resize (no_of_neighbors);

  pcl::search::Search<PointXYZ> *nanoflann_search, *kdtree_search;

  PointCloud<PointXYZ>::Ptr pc = cloud_big.makeShared();
  {
    ScopeTime scopeTime ("NANOFLANN build");
    nanoflann_search = new pcl::search::NanoFlannSearch<PointXYZ> ();
    nanoflann_search->setInputCloud (pc);
  }

  {
    ScopeTime scopeTime ("kdtree build");
    kdtree_search = new pcl::search::KdTree<PointXYZ> ();
    kdtree_search->setInputCloud (pc);
  }



  {
    ScopeTime scopeTime ("NANOFLANN nearestKSearch");
    for (size_t i = 0; i < cloud_big.points.size (); ++i)
      nanoflann_search->nearestKSearch (cloud_big.points[i], no_of_neighbors, k_indices, k_distances);
  }
  {
    ScopeTime scopeTime ("kd tree  nearestKSearch");
    for (size_t i = 0; i < cloud_big.points.size (); ++i)
      kdtree_search->nearestKSearch (cloud_big.points[i], no_of_neighbors, k_indices, k_distances);
  }

  vector<vector<int> > indices_nanoflann;
  vector<vector<float> > dists_nanoflann;
  vector<vector<int> > indices_tree;
  vector<vector<float> > dists_tree;
  indices_nanoflann.resize (cloud_big.size ());
  dists_nanoflann.resize (cloud_big.size ());
  indices_tree.resize (cloud_big.size ());
  dists_tree.resize (cloud_big.size ());
  for (size_t i = 0; i<cloud_big.size (); ++i)
  {
    indices_nanoflann[i].resize (no_of_neighbors);
    dists_nanoflann[i].resize (no_of_neighbors);
    indices_tree[i].resize (no_of_neighbors);
    dists_tree[i].resize (no_of_neighbors);
  }

  {
    ScopeTime scopeTime ("NANOFLANN multi nearestKSearch");
    nanoflann_search->nearestKSearch (cloud_big, std::vector<int> (), no_of_neighbors, indices_nanoflann,dists_nanoflann);
  }
  {
    ScopeTime scopeTime ("kd tree multi nearestKSearch");
    kdtree_search->nearestKSearch (cloud_big, std::vector<int> (), no_of_neighbors, indices_tree,dists_tree);
  }

  ASSERT_EQ (indices_nanoflann.size (), dists_nanoflann.size ());
  ASSERT_EQ (indices_nanoflann.size (), indices_tree.size ());
  ASSERT_EQ (indices_nanoflann.size (), dists_tree.size ());

  for (size_t i = 0; i<indices_nanoflann.size ();i++)
  {
    ASSERT_EQ (indices_nanoflann[i].size (), no_of_neighbors);
    ASSERT_EQ (indices_tree[i].size (), no_of_neighbors);
    ASSERT_EQ (dists_nanoflann[i].size (), no_of_neighbors);
    ASSERT_EQ (dists_tree[i].size (), no_of_neighbors);
    for (int j = 0; j<no_of_neighbors; j++)
    {

      ASSERT_TRUE( indices_nanoflann[i][j] == indices_tree[i][j] || dists_nanoflann[i][j]==dists_tree[i][j]);
    }
  }


  vector<int> query_indices;
  for (size_t i = 0; i<cloud_big.size (); i+=2)
    query_indices.push_back (int (i));

  {
    ScopeTime scopeTime ("NANOFLANN multi nearestKSearch with indices");
    nanoflann_search->nearestKSearch (cloud_big, query_indices, no_of_neighbors, indices_nanoflann,dists_nanoflann);
  }
  {
    ScopeTime scopeTime ("kd tree multi nearestKSearch with indices");
    kdtree_search->nearestKSearch (cloud_big, query_indices, no_of_neighbors, indices_tree,dists_tree);
  }
  ASSERT_EQ (indices_nanoflann.size (), dists_nanoflann.size ());
  ASSERT_EQ (indices_nanoflann.size (), indices_tree.size ());
  ASSERT_EQ (indices_nanoflann.size (), dists_tree.size ());

  for (size_t i = 0; i<indices_nanoflann.size ();i++)
  {
    ASSERT_EQ (indices_nanoflann[i].size (), no_of_neighbors);
    ASSERT_EQ (indices_tree[i].size (), no_of_neighbors);
    ASSERT_EQ (dists_nanoflann[i].size (), no_of_neighbors);
    ASSERT_EQ (dists_tree[i].size (), no_of_neighbors);
    for (int j = 0; j < no_of_neighbors; j++ )
    {
      ASSERT_TRUE( indices_nanoflann[i][j] == indices_tree[i][j] || dists_nanoflann[i][j]==dists_tree[i][j]);
    }
  }

  delete nanoflann_search;
  delete kdtree_search;
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  init ();

  // Testing using explicit instantiation of inherited class
  pcl::search::Search<PointXYZ>* NanoFlannSearch = new pcl::search::NanoFlannSearch<PointXYZ> ();
  NanoFlannSearch->setInputCloud (cloud.makeShared ());

  return (RUN_ALL_TESTS ());
}
