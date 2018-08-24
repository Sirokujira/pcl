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
  , flann_index_ (), cloud_ ()
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
  , flann_index_ (), cloud_ ()
  , index_mapping_ (), identity_mapping_ (false)
  , dim_ (0), total_nr_points_ (0)
  , param_k_ (nanoflann::SearchParams (-1 , epsilon_))
  , param_radius_ (nanoflann::SearchParams (-1, epsilon_, false))
{
  *this = k;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::KdTreeNANOFLANN<PointT>::setEpsilon (float eps)
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

  // flann_index_.reset (new FLANNIndex (::flann::Matrix<float> (cloud_.get (), 
  //                                                             index_mapping_.size (), 
  //                                                             dim_),
  //                                     ::flann::KDTreeSingleIndexParams (15))); // max 15 points/leaf

  typedef PointCloudAdaptor<PointCloud<float> > PC2KD
  const PC2KD pc2kd(cloud); // The adaptor
  typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<float, PC2KD>, PC2KD, 3> FLANNIndex;

  flann_index_ = FLANNIndex(3, pc2kd, nanoflann::KDTreeSingleIndexAdaptorParams(15));
  flann_index_->buildIndex();
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

  k_indices.resize (k);
  k_distances.resize (k);

  std::vector<float> query (dim_);
  point_representation_->vectorize (static_cast<PointT> (point), query);

  ::flann::Matrix<int> k_indices_mat (&k_indices[0], 1, k);
  ::flann::Matrix<float> k_distances_mat (&k_distances[0], 1, k);
  // Wrap the k_indices and k_distances vectors (no data copy)
  // flann_index_->knnSearch (::flann::Matrix<float> (&query[0], 1, dim_), 
  //                         k_indices_mat, k_distances_mat,
  //                         k, param_k_);

  nanoflann::KNNResultSet<float> resultSet(k);
  resultSet.init(&k_indices, &k_distances);
  index.findNeighbors(resultSet, &query[0], nanoflann::SearchParams(10));
  // k = index.knnSearch(&query[0], k, &k_indices[0], &k_distances[0]);
  k_indices.resize(k)
  k_distances.resize(k)

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

  std::vector<std::pair<size_t, double> > ret_matches;
  nanoflann::SearchParams params;
  const size_t nMatches = index.radiusSearch(&query[0], radius, ret_matches, params);

  // std::cout << "radiusSearch(): radius=" << search_radius << " -> " << nMatches << " matches\n";
  // for (size_t i = 0; i < nMatches; i++)
  //   std::cout << "idx["<< i << "]=" << ret_matches[i].first << " dist["<< i << "]=" << ret_matches[i].second << std::endl;
  // std::cout << "\n";

  k_indices = ret_matches[0].first;
  k_sqr_dists = ret_matches[0].second;

  // Do mapping to original point cloud
  if (!identity_mapping_) 
  {
    for (int i = 0; i < nMatches; ++i)
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

