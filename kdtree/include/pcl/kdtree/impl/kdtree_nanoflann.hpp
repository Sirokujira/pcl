/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 */

#ifndef PCL_KDTREE_KDTREE_IMPL_NANOFLANN_H_
#define PCL_KDTREE_KDTREE_IMPL_NANOFLANN_H_

#include <cstdio>
#include <pcl/kdtree/kdtree_nanoflann.h>
#include <pcl/kdtree/nanoflann.h>
#include <pcl/console/print.h>

#include <Eigen/Dense>
using namespace Eigen;

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::KdTreeNANOFLANN<PointT>::KdTreeNANOFLANN (bool sorted)
  : pcl::KdTree<PointT> (sorted)
  , nanoflann_index_ (), cloud_ ()
  , index_mapping_ (), identity_mapping_ (false)
  , dim_ (0), total_nr_points_ (0)
  , param_k_ (nanoflann::SearchParams (-1 , epsilon_))
  , param_radius_ (nanoflann::SearchParams (-1, epsilon_, sorted))
{
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::KdTreeNANOFLANN<PointT>::KdTreeNANOFLANN (const KdTreeNANOFLANN<PointT> &k) 
  : pcl::KdTree<PointT> (false)
  , nanoflann_index_ (), cloud_ ()
  , index_mapping_ (), identity_mapping_ (false)
  , dim_ (0), total_nr_points_ (0)
  , param_k_ (nanoflann::SearchParams (-1 , epsilon_))
  , param_radius_ (nanoflann::SearchParams (-1, epsilon_, false))
{
  *this = k;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void pcl::KdTreeNANOFLANN<PointT>::setEpsilon (float eps)
{
  epsilon_ = eps;
  param_k_ =  nanoflann::SearchParams (-1 , epsilon_);
  param_radius_ = nanoflann::SearchParams (-1 , epsilon_, sorted_);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::KdTreeNANOFLANN<PointT>::setSortedResults (bool sorted)
{
  sorted_ = sorted;
  param_k_ = nanoflann::SearchParams (-1, epsilon_);
  param_radius_ = nanoflann::SearchParams (-1, epsilon_, sorted_);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::KdTreeNANOFLANN<PointT>::setInputCloud (const PointCloudConstPtr &cloud, const IndicesConstPtr &indices)
{
  cleanup ();   // Perform an automatic cleanup of structures

  epsilon_ = 0.0f;   // default error bound value
  dim_ = point_representation_->getNumberOfDimensions (); // Number of dimensions - default is 3 = xyz

  input_   = cloud;
  indices_ = indices;
  
  // Allocate enough data
  if (!input_)
  {
    PCL_ERROR ("[pcl::KdTreeNANOFLANN::setInputCloud] Invalid input!\n");
    return;
  }
  if (indices != NULL)
  {
    convertCloudToArray (*input_, *indices_);
  }
  else
  {
    convertCloudToArray (*input_);
  }
  total_nr_points_ = static_cast<int> (index_mapping_.size ());
  if (total_nr_points_ == 0)
  {
    PCL_ERROR ("[pcl::KdTreeNANOFLANN::setInputCloud] Cannot create a KDTree with an empty input cloud!\n");
    return;
  }

  // from float* to SearchPointCloud<float>
  // float* array = cloud_.get();
  size_t N = index_mapping_.size();
  // SearchPointCloud<float> cloud_pt;
  cloud_pt.pts.resize(N);
  for (size_t i = 0; i < N;i++) 
  {
    // cloud_pt.pts[i].x = array[i * 3 + 0];
    // cloud_pt.pts[i].y = array[i * 3 + 1];
    // cloud_pt.pts[i].z = array[i * 3 + 2];
    cloud_pt.pts[i].x = cloud_[i * 3 + 0];
    cloud_pt.pts[i].y = cloud_[i * 3 + 1];
    cloud_pt.pts[i].z = cloud_[i * 3 + 2];
  }

  // const PC2KD pc2kd(cloud_pt); // The adaptor

  // boost::shared
  // nanoflann_index_.reset(new NANOFLANNIndex(dim_, pc2kd, nanoflann::KDTreeSingleIndexAdaptorParams(15)));
  nanoflann_index_.reset(new NANOFLANNIndex(dim_, cloud_pt, nanoflann::KDTreeSingleIndexAdaptorParams(15)));
  // std::unique_ptr
  // nanoflann_index_(new NANOFLANNIndex(dim_, pc2kd, nanoflann::KDTreeSingleIndexAdaptorParams(15)));
  nanoflann_index_->buildIndex();
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int 
pcl::KdTreeNANOFLANN<PointT>::nearestKSearch (const PointT &point, int k, 
                                                std::vector<int> &k_indices, 
                                                std::vector<float> &k_distances) const
{
  assert (point_representation_->isValid (point) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");

  if (k > total_nr_points_)
    k = total_nr_points_;

  // std::vector<size_t>   ret_index(k);
  // std::vector<float> out_dist_sqr(k);

  // k_indices.resize (k);
  // k_distances.resize (k);

  std::vector<float> query (dim_);
  point_representation_->vectorize (static_cast<PointT> (point), query);

  // construct a kd-tree index: 
  // typedef nanoflann::KDTreeSingleIndexAdaptor<
  //   nanoflann::L2_Simple_Adaptor<float, SearchPointCloud<float> > ,
  //   SearchPointCloud<float>, 
  //   3
  //   > my_kd_tree_t;
  // // exe ok, test ng(loop)
  // // my_kd_tree_t flann_index3_(dim_, cloud_pt, nanoflann::KDTreeSingleIndexAdaptorParams(15));
  // // exe ok, test ok(call nearestKSearch once)
  // static my_kd_tree_t flann_index3_(dim_, cloud_pt, nanoflann::KDTreeSingleIndexAdaptorParams(15));
  // flann_index3_.buildIndex();

  nanoflann::KNNResultSet<float> resultSet(k);

  // 32 bit?
  // resultSet.init(&k_indices[0], &k_distances[0]);
  // 64 bit(size_t)
  resultSet.init((unsigned __int64 *)&k_indices[0], &k_distances[0]);
  // resultSet.init(&ret_index[0], &out_dist_sqr[0]);

  // findNeighbors
  // nanoflann_index_->findNeighbors(resultSet, &query[0], nanoflann::SearchParams(param_k_));
  nanoflann_index_->findNeighbors(resultSet, &query[0], nanoflann::SearchParams(15));
  // flann_index2_.findNeighbors(resultSet, &query[0], nanoflann::SearchParams(param_k_));

  // knnSearch
  // return : size_t
  // OK
  // k = nanoflann_index_->knnSearch(&query[0], k, &ret_index[0], &out_dist_sqr[0]);
  // OK(call once)(second call? generate SEH Exception)
  // k = flann_index3_.knnSearch(&query[0], k, &ret_index[0], &out_dist_sqr[0]);

  // ret_index.resize(k);
  // out_dist_sqr.resize(k);
  // k_indices.resize(k);
  // k_distances.resize(k);
  // std::cout << "NANOFLANN(idx3) - knnSearch(): k=" << k << "\n";
  // for (size_t i = 0; i < k; i++)
  //   std::cout << "idx["<< i << "]=" << ret_index[i] << " dist["<< i << "]=" << out_dist_sqr[i] << std::endl;
  // std::cout << "\n";
  // ret_matches
  // for (int i = 0; i < k; i++)
  // {
  //   k_indices[i] = ret_index[i];
  //   k_distances[i] = out_dist_sqr[i];
  // }

  // Do mapping to original point cloud
  if (!identity_mapping_) 
  {
    for (size_t i = 0; i < static_cast<size_t> (k); ++i)
    {
      int& neighbor_index = k_indices[i];
      neighbor_index = index_mapping_[neighbor_index];
    }
  }

  return (k);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int 
pcl::KdTreeNANOFLANN<PointT>::radiusSearch (const PointT &point, double radius, std::vector<int> &k_indices,
                                              std::vector<float> &k_sqr_dists, unsigned int max_nn) const
{
  assert (point_representation_->isValid (point) && "Invalid (NaN, Inf) point coordinates given to radiusSearch!");

  std::vector<float> query (dim_);
  point_representation_->vectorize (static_cast<PointT> (point), query);

  // Has max_nn been set properly?
  if (max_nn == 0 || max_nn > static_cast<unsigned int> (total_nr_points_))
    max_nn = total_nr_points_;

  // std::vector<std::vector<int> > indices(1);
  // std::vector<std::vector<float> > dists(1);
  std::vector<std::pair<size_t, float> > ret_matches;

  nanoflann::SearchParams params;
  // if (max_nn == static_cast<unsigned int>(total_nr_points_))
  //   params.max_neighbors = -1;  // return all neighbors in radius
  // else
  //   params.max_neighbors = max_nn;

  // static const PC2KD pc2kd(cloud_pt); // The adaptor
  // nanoflann_index_.reset(new NANOFLANNIndex(dim_, pc2kd, nanoflann::KDTreeSingleIndexAdaptorParams(15)));
  // nanoflann_index_->buildIndex();
  // NANOFLANNIndex flann_index2_(dim_, pc2kd, nanoflann::KDTreeSingleIndexAdaptorParams(15));
  // flann_index2_.buildIndex();

  const float search_radius = static_cast<float>(radius * radius);
  // const float search_radius = static_cast<float>(radius);

  nanoflann::RadiusResultSet<float, size_t> resultSet(search_radius, ret_matches);

  int neighbors_in_radius = nanoflann_index_->radiusSearch(&query[0], search_radius, ret_matches, params);
  // int neighbors_in_radius = nanoflann_index_->radiusSearch(resultSet, &query[0], params);
  // int neighbors_in_radius = flann_index2_.radiusSearch(&query[0], search_radius, ret_matches, params);
  // int neighbors_in_radius = flann_index2_.radiusSearch(resultSet, &query[0], params);

  // ret_matches
  for (int i = 0; i < neighbors_in_radius; i++)
  {
    k_indices.push_back(ret_matches[i].first);
    k_sqr_dists.push_back(ret_matches[i].second);
  }

  // Do mapping to original point cloud
  if (!identity_mapping_) 
  {
    for (int i = 0; i < neighbors_in_radius; ++i)
    {
      int& neighbor_index = k_indices[i];
      neighbor_index = index_mapping_[neighbor_index];
    }
  }

  return (neighbors_in_radius);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::KdTreeNANOFLANN<PointT>::cleanup ()
{
  // Data array cleanup
  index_mapping_.clear ();

  if (indices_)
    indices_.reset ();
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::KdTreeNANOFLANN<PointT>::convertCloudToArray (const PointCloud &cloud)
{
  // No point in doing anything if the array is empty
  if (cloud.points.empty ())
  {
    cloud_.reset ();
    return;
  }

  int original_no_of_points = static_cast<int> (cloud.points.size ());

  cloud_.reset (new float[original_no_of_points * dim_]);
  float* cloud_ptr = cloud_.get ();
  index_mapping_.reserve (original_no_of_points);
  identity_mapping_ = true;

  for (int cloud_index = 0; cloud_index < original_no_of_points; ++cloud_index)
  {
    // Check if the point is invalid
    if (!point_representation_->isValid (cloud.points[cloud_index]))
    {
      identity_mapping_ = false;
      continue;
    }

    index_mapping_.push_back (cloud_index);

    point_representation_->vectorize (cloud.points[cloud_index], cloud_ptr);
    cloud_ptr += dim_;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::KdTreeNANOFLANN<PointT>::convertCloudToArray (const PointCloud &cloud, const std::vector<int> &indices)
{
  // No point in doing anything if the array is empty
  if (cloud.points.empty ())
  {
    cloud_.reset ();
    return;
  }

  int original_no_of_points = static_cast<int> (indices.size ());

  cloud_.reset (new float[original_no_of_points * dim_]);
  float* cloud_ptr = cloud_.get ();
  index_mapping_.reserve (original_no_of_points);
  // its a subcloud -> false
  // true only identity: 
  //     - indices size equals cloud size
  //     - indices only contain values between 0 and cloud.size - 1
  //     - no index is multiple times in the list
  //     => index is complete
  // But we can not guarantee that => identity_mapping_ = false
  identity_mapping_ = false;
  
  for (std::vector<int>::const_iterator iIt = indices.begin (); iIt != indices.end (); ++iIt)
  {
    // Check if the point is invalid
    if (!point_representation_->isValid (cloud.points[*iIt]))
      continue;

    // map from 0 - N -> indices [0] - indices [N]
    index_mapping_.push_back (*iIt);  // If the returned index should be for the indices vector
    
    point_representation_->vectorize (cloud.points[*iIt], cloud_ptr);
    cloud_ptr += dim_;
  }
}

#define PCL_INSTANTIATE_KdTreeNANOFLANN(T) template class PCL_EXPORTS pcl::KdTreeNANOFLANN<T>;

#endif  //#ifndef _PCL_KDTREE_KDTREE_IMPL_NANOFLANN_H_

