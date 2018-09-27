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
 * $Id$
 *
 */

#ifndef PCL_SEARCH_IMPL_NANOFLANN_SEARCH_H_
#define PCL_SEARCH_IMPL_NANOFLANN_SEARCH_H_

#include <pcl/search/nanoflann_search.h>
#include <pcl/kdtree/nanoflann.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
typename pcl::search::NanoFlannSearch<PointT>::IndexPtr
pcl::search::NanoFlannSearch<PointT>::KdTreeIndexCreator::createIndex (MatrixConstPtr data)
{
  std::vector<float> vec(data->size());
  Eigen::Map<Eigen::MatrixXf>(vec.data(), data->rows(), data->cols()) = *data;

  SearchPointCloud<float> cloud_pt;
  size_t N = data->rows();
  size_t dim = data->cols();
  cloud_pt.pts.resize(N);
  for (size_t i = 0; i < N;i++) 
  {
    cloud_pt.pts[i].x = vec[i * dim + 0];
    cloud_pt.pts[i].y = vec[i * dim + 1];
    cloud_pt.pts[i].z = vec[i * dim + 2];
  }

  const PC2KD pc2kd(cloud_pt); // The adaptor

  return (IndexPtr (new Index(3, pc2kd, nanoflann::KDTreeSingleIndexAdaptorParams (max_leaf_size_))));
  // return (IndexPtr (new Index(3, *data, nanoflann::KDTreeSingleIndexAdaptorParams (max_leaf_size_))));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
typename pcl::search::NanoFlannSearch<PointT>::IndexPtr
pcl::search::NanoFlannSearch<PointT>::KMeansIndexCreator::createIndex (MatrixConstPtr data)
{
  std::vector<float> vec(data->size());
  Eigen::Map<Eigen::MatrixXf>(vec.data(), data->rows(), data->cols()) = *data;

  SearchPointCloud<float> cloud_pt;
  size_t N = data->rows();
  size_t dim = data->cols();
  cloud_pt.pts.resize(N);
  for (size_t i = 0; i < N;i++) 
  {
    cloud_pt.pts[i].x = vec[i * dim + 0];
    cloud_pt.pts[i].y = vec[i * dim + 1];
    cloud_pt.pts[i].z = vec[i * dim + 2];
  }

  const PC2KD pc2kd(cloud_pt); // The adaptor
  // return (IndexPtr (new nanoflann::KMeansIndex<FlannDistance> (*data, flann::KMeansIndexParams ())));
  return (IndexPtr (new Index(3, pc2kd, nanoflann::KDTreeSingleIndexAdaptorParams (max_leaf_size_))));
  // return (IndexPtr (new Index(3, *data, nanoflann::KDTreeSingleIndexAdaptorParams (max_leaf_size_))));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
typename pcl::search::NanoFlannSearch<PointT>::IndexPtr
pcl::search::NanoFlannSearch<PointT>::KdTreeMultiIndexCreator::createIndex (MatrixConstPtr data)
{
  std::vector<float> vec(data->size());
  Eigen::Map<Eigen::MatrixXf>(vec.data(), data->rows(), data->cols()) = *data;

  SearchPointCloud<float> cloud_pt;
  size_t N = data->rows();
  size_t dim = data->cols();
  cloud_pt.pts.resize(N);
  for (size_t i = 0; i < N;i++)
  {
    cloud_pt.pts[i].x = vec[i * dim + 0];
    cloud_pt.pts[i].y = vec[i * dim + 1];
    cloud_pt.pts[i].z = vec[i * dim + 2];
  }

  const PC2KD pc2kd(cloud_pt); // The adaptor

  // return (IndexPtr (new nanoflann::KDTreeIndex<FlannDistance> (*data, nanoflann::KDTreeIndexParams (trees_))));
  return (IndexPtr (new Index(3, pc2kd, nanoflann::KDTreeSingleIndexAdaptorParams (max_leaf_size_))));
  // return (IndexPtr (new IndexIndex(3, *data, nanoflann::KDTreeSingleIndexAdaptorParams (max_leaf_size_))));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::search::NanoFlannSearch<PointT>::NanoFlannSearch(bool sorted, NanoFlannIndexCreatorPtr creator) : pcl::search::Search<PointT> ("NanoFlannSearch", sorted),
  // index_(), creator_ (creator), input_nanoflann_( MatrixPtr(new Eigen::MatrixXf(reinterpret_cast<const float &>(init_dummy_data), 1, 1)) ), eps_ (0), checks_ (32), input_copied_for_nanoflann_ (false), point_representation_ (new DefaultPointRepresentation<PointT>),
  index_(), creator_ (creator), eps_ (0), checks_ (32), input_copied_for_nanoflann_ (false), point_representation_ (new DefaultPointRepresentation<PointT>),
  dim_ (0), index_mapping_(), identity_mapping_()
{
  dim_ = point_representation_->getNumberOfDimensions ();
  // input_nanoflann_ = MatrixPtr (new Eigen::MatrixXf(new float[1], 1, 1));
  // input_nanoflann_ = MatrixPtr (new Eigen::MatrixXf(reinterpret_cast<const float&> (new float[1]), 1, 1));
  input_nanoflann_ = MatrixPtr (new Eigen::MatrixXf(reinterpret_cast<const float&> (init_dummy_data), 1, 1));
  
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::search::NanoFlannSearch<PointT>::~NanoFlannSearch()
{
  // if (input_copied_for_nanoflann_)
  //   delete [] input_nanoflann_.get();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::search::NanoFlannSearch<PointT>::setInputCloud (const PointCloudConstPtr& cloud, const IndicesConstPtr& indices)
{
  input_ = cloud;
  indices_ = indices;
  convertInputToNanoFlannMatrix ();
  index_ = creator_->createIndex (input_nanoflann_);
  index_->buildIndex();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::search::NanoFlannSearch<PointT>::nearestKSearch (const PointT &point, int k, std::vector<int> &indices, std::vector<float> &dists) const
{
  assert (point_representation_->isValid (point) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!"); // remove this check as soon as FLANN does NaN checks internally
  bool can_cast = point_representation_->isTrivial ();

  float* data = 0;
  if (!can_cast)
  {
    data = new float [point_representation_->getNumberOfDimensions ()];
    point_representation_->vectorize (point,data);
  }

  float* cdata = can_cast ? const_cast<float*> (reinterpret_cast<const float*> (&point)): data;
  // const nanoflann::Matrix<float> m (cdata ,1, point_representation_->getNumberOfDimensions ());

  nanoflann::SearchParams params;
  params.eps = eps_;
  params.sorted = sorted_results_;
  // p.checks = checks_;
  if (indices.size() != static_cast<unsigned int> (k))
    indices.resize (k,-1);
  if (dists.size() != static_cast<unsigned int> (k))
    dists.resize (k);

  // nanoflann::Matrix<int> i (&indices[0],1,k);
  // nanoflann::Matrix<float> d (&dists[0],1,k);
  // int result = index_->knnSearch (m,i,d,k, params);
  // do a knn search 
  // const size_t num_results = 1; 
  // size_t ret_index; 
  // PointT out_dist_sqr; 
  // nanoflann::KNNResultSet<PointT> resultSet(num_results); 
  // resultSet.init(&ret_index, &out_dist_sqr ); 
  // index.findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));
  // std::cout << "knnSearch(nn="<<num_results<<"): \n"; 
  // std::cout << "ret_index=" << ret_index << " out_dist_sqr=" << out_dist_sqr << endl; 

  int result = -1;
  delete [] data;

  if (!identity_mapping_)
  {
    for (size_t i = 0; i < static_cast<unsigned int> (k); ++i)
    {
      int& neighbor_index = indices[i];
      neighbor_index = index_mapping_[neighbor_index];
    }
  }
  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::search::NanoFlannSearch<PointT>::nearestKSearch (
    const PointCloud& cloud, const std::vector<int>& indices, int k, std::vector< std::vector<int> >& k_indices,
    std::vector< std::vector<float> >& k_sqr_distances) const
{
  std::vector<std::pair<size_t, float> > ret_matches;
  if (indices.empty ())
  {
    k_indices.resize (cloud.size ());
    k_sqr_distances.resize (cloud.size ());

    if (! cloud.is_dense) // remove this check as soon as FLANN does NaN checks internally
    {
      for (size_t i = 0; i < cloud.size(); i++)
      {
        assert (point_representation_->isValid (cloud[i]) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");
      }
    }

    bool can_cast = point_representation_->isTrivial ();

    // full point cloud + trivial copy operation = no need to do any conversion/copying to the flann matrix!
    float* data=0;
    if (!can_cast)
    {
      data = new float[dim_*cloud.size ()];
      for (size_t i = 0; i < cloud.size (); ++i)
      {
        float* out = data+i*dim_;
        point_representation_->vectorize (cloud[i],out);
      }
    }

    // const cast is evil, but the matrix constructor won't change the data, and the
    // search won't change the matrix
    float* cdata = can_cast ? const_cast<float*> (reinterpret_cast<const float*> (&cloud[0])): data;
    // const Eigen::MatrixXf m (cdata ,cloud.size (), dim_, can_cast ? sizeof (PointT) : dim_ * sizeof (float) );

    nanoflann::SearchParams params;
    params.sorted = sorted_results_;
    params.eps = eps_;
    // p.checks = checks_;
    // index_->knnSearch (m, k_indices, k_sqr_distances, k, params);
    // index_->ret_matches

    delete [] data;
  }
  else // if indices are present, the cloud has to be copied anyway. Only copy the relevant parts of the points here.
  {
    k_indices.resize (indices.size ());
    k_sqr_distances.resize (indices.size ());

    if (! cloud.is_dense) // remove this check as soon as FLANN does NaN checks internally
    {
      for (size_t i = 0; i < indices.size(); i++)
      {
        assert (point_representation_->isValid (cloud [indices[i]]) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");
      }
    }

    float* data=new float [dim_*indices.size ()];
    for (size_t i = 0; i < indices.size (); ++i)
    {
      float* out = data+i*dim_;
      point_representation_->vectorize (cloud[indices[i]],out);
    }
    const Eigen::MatrixXf m (reinterpret_cast<const float&>(data), indices.size (), point_representation_->getNumberOfDimensions ());

    nanoflann::SearchParams params;
    params.sorted = sorted_results_;
    params.eps = eps_;
    // p.checks = checks_;
    // index_->knnSearch (m, k_indices, k_sqr_distances, k, params);

    delete[] data;
  }
  if (!identity_mapping_)
  {
    for (size_t j = 0; j < k_indices.size (); ++j)
    {
      for (size_t i = 0; i < static_cast<unsigned int> (k); ++i)
      {
        int& neighbor_index = k_indices[j][i];
        neighbor_index = index_mapping_[neighbor_index];
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::search::NanoFlannSearch<PointT>::radiusSearch (
    const PointT& point,
    double radius,
    std::vector<int> &indices, std::vector<float> &distances,
    unsigned int max_nn) const
{
  assert (point_representation_->isValid (point) && "Invalid (NaN, Inf) point coordinates given to radiusSearch!"); // remove this check as soon as FLANN does NaN checks internally
  bool can_cast = point_representation_->isTrivial ();

  float* data = 0;
  if (!can_cast)
  {
    data = new float [point_representation_->getNumberOfDimensions ()];
    point_representation_->vectorize (point,data);
  }

  float* cdata = can_cast ? const_cast<float*> (reinterpret_cast<const float*> (&point)) : data;
  const Eigen::MatrixXf m (reinterpret_cast<const float&>(cdata) ,1, point_representation_->getNumberOfDimensions ());

  nanoflann::SearchParams params;
  params.sorted = sorted_results_;
  params.eps = eps_;
  // p.max_neighbors = max_nn > 0 ? max_nn : -1;
  // p.checks = checks_;
  // replace ret_matches? std::pair(i, d)
  std::vector<std::vector<size_t> > i (1);
  std::vector<std::vector<float> > d (1);
  std::vector<std::pair<size_t, float> > ret_matches;
  // const PointT search_radius = PointT(radius, radius, radius);
  const float search_radius = static_cast<float>(radius);

  std::vector<float> vec(m.size());
  Eigen::Map<Eigen::MatrixXf>(vec.data(), m.rows(), m.cols()) = m;

  // int result = index_->radiusSearch (m, i, d, static_cast<float> (radius * radius), params);
  // const size_t nMatches = index_->radiusSearch(&cdata[0], search_radius, ret_matches, params);
  const size_t nMatches = index_->radiusSearch(vec.data(), search_radius, ret_matches, params);
  cout << "radiusSearch(): radius=" << radius << " -> " << nMatches << " matches\n";

  for (size_t i = 0; i < nMatches; i++)
    cout << "idx["<< i << "]=" << ret_matches[i].first << " dist["<< i << "]=" << ret_matches[i].second << endl;

  cout << "\n"; 

  delete [] data;
  // indices = i [0];
  // distances = d [0];

  // return result;
  return nMatches;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void pcl::search::NanoFlannSearch<PointT>::radiusSearch (
    const PointCloud& cloud, const std::vector<int>& indices,
    double radius,
    std::vector< std::vector<int> >& k_indices, std::vector< std::vector<float> >& k_sqr_distances, unsigned int max_nn) const
{
  std::vector<std::pair<size_t, float> > ret_matches;

  if (indices.empty ()) // full point cloud + trivial copy operation = no need to do any conversion/copying to the flann matrix!
  {
    k_indices.resize (cloud.size ());
    k_sqr_distances.resize (cloud.size ());

    if (! cloud.is_dense) // remove this check as soon as FLANN does NaN checks internally
    {
      for (size_t i = 0; i < cloud.size(); i++)
      {
        assert (point_representation_->isValid (cloud[i]) && "Invalid (NaN, Inf) point coordinates given to radiusSearch!");
      }
    }

    bool can_cast = point_representation_->isTrivial ();

    float* data = 0;
    if (!can_cast)
    {
      data = new float[dim_*cloud.size ()];
      for (size_t i = 0; i < cloud.size (); ++i)
      {
        float* out = data + i * dim_;
        point_representation_->vectorize (cloud[i], out);
      }
    }

    float* cdata = can_cast ? const_cast<float*> (reinterpret_cast<const float*> (&cloud[0])) : data;
    const Eigen::MatrixXf m (reinterpret_cast<const float&>(cdata), cloud.size (), dim_, can_cast ? sizeof (PointT) : dim_ * sizeof (float));

    nanoflann::SearchParams params;
    params.sorted = sorted_results_;
    params.eps = eps_;
    const float search_radius = static_cast<float>(radius);

    // Note that I initialize vec with the matrix size to begin with:
    std::vector<float> vec(m.size());
  	Eigen::Map<Eigen::MatrixXf>(vec.data(), m.rows(), m.cols()) = m;

  	// index_->radiusSearch (m, k_indices, k_sqr_distances, static_cast<float> (radius * radius), params);
    // const size_t nMatches = index_->radiusSearch(m, search_radius, ret_matches, params);
  	const size_t nMatches = index_->radiusSearch(vec.data(), search_radius, ret_matches, params);
    cout << "radiusSearch(): radius=" << radius << " -> " << nMatches << " matches\n"; 

    for (size_t i = 0; i < nMatches; i++)
      cout << "idx["<< i << "]=" << ret_matches[i].first << " dist["<< i << "]=" << ret_matches[i].second << endl; 
    cout << "\n"; 

    delete [] data;
  }
  else // if indices are present, the cloud has to be copied anyway. Only copy the relevant parts of the points here.
  {
    k_indices.resize (indices.size ());
    k_sqr_distances.resize (indices.size ());

    if (! cloud.is_dense)  // remove this check as soon as FLANN does NaN checks internally
    {
      for (size_t i = 0; i < indices.size(); i++)
      {
        assert (point_representation_->isValid (cloud [indices[i]]) && "Invalid (NaN, Inf) point coordinates given to radiusSearch!");
      }
    }

    float* data = new float [dim_ * indices.size ()];
    for (size_t i = 0; i < indices.size (); ++i)
    {
      float* out = data + i * dim_;
      point_representation_->vectorize (cloud[indices[i]], out);
    }
    // const flann::Matrix<float> m (data, cloud.size (), point_representation_->getNumberOfDimensions ());
    // const Eigen::MatrixXf m (data, cloud.size (), point_representation_->getNumberOfDimensions ());
  	const Eigen::MatrixXf m (reinterpret_cast<const float&>(data), cloud.size (), point_representation_->getNumberOfDimensions ());

    nanoflann::SearchParams params;
    params.sorted = sorted_results_;
    params.eps = eps_;
    const float search_radius = static_cast<float>(radius);

  	std::vector<float> vec(m.size());
    Eigen::Map<Eigen::MatrixXf>(vec.data(), m.rows(), m.cols()) = m;

    // index_->radiusSearch (m, k_indices, k_sqr_distances, static_cast<float> (radius * radius), params);
    // const size_t nMatches = index_->radiusSearch(m, search_radius, ret_matches, params);
  	const size_t nMatches = index_->radiusSearch(vec.data(), search_radius, ret_matches, params);
    cout << "radiusSearch(): radius=" << search_radius << " -> " << nMatches << " matches\n"; 
    for (size_t i = 0; i < nMatches; i++) 
      cout << "idx["<< i << "]=" << ret_matches[i].first << " dist["<< i << "]=" << ret_matches[i].second << endl; 

    cout << "\n"; 

    delete[] data;
  }
  if (!identity_mapping_)
  {
    for (size_t j = 0; j < k_indices.size (); ++j )
    {
      for (size_t i = 0; i < k_indices[j].size (); ++i)
      {
        int& neighbor_index = k_indices[j][i];
        neighbor_index = index_mapping_[neighbor_index];
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void pcl::search::NanoFlannSearch<PointT>::convertInputToNanoFlannMatrix ()
{
  size_t original_no_of_points = indices_ && !indices_->empty () ? indices_->size () : input_->size ();

  // if (input_copied_for_nanoflann_)
  //   delete input_nanoflann_->get();
  input_copied_for_nanoflann_ = true;
  index_mapping_.clear();
  identity_mapping_ = true;

  //cloud_ = (float*)malloc (original_no_of_points * dim_ * sizeof (float));
  //index_mapping_.reserve(original_no_of_points);
  //identity_mapping_ = true;

  if (!indices_ || indices_->empty ())
  {
    // best case: all points can be passed to nanoflann without any conversions
    if (input_->is_dense && point_representation_->isTrivial ())
    {
      // const cast is evil, but nanoflann won't change the data
      input_nanoflann_ = MatrixPtr (new Eigen::MatrixXf (const_cast<float*>(reinterpret_cast<const float*>(&(*input_) [0])), original_no_of_points, point_representation_->getNumberOfDimensions (),sizeof (PointT)));
      input_copied_for_nanoflann_ = false;
    }
    else
    {
      input_nanoflann_ = MatrixPtr (new Eigen::MatrixXf (new float[original_no_of_points*point_representation_->getNumberOfDimensions ()], original_no_of_points, point_representation_->getNumberOfDimensions ()));
      float* cloud_ptr = input_nanoflann_->get();
      for (size_t i = 0; i < original_no_of_points; ++i)
      {
        const PointT& point = (*input_)[i];
        // Check if the point is invalid
        if (!point_representation_->isValid (point))
        {
          identity_mapping_ = false;
          continue;
        }

        index_mapping_.push_back (static_cast<int> (i));  // If the returned index should be for the indices vector

        point_representation_->vectorize (point, cloud_ptr);
        cloud_ptr += dim_;
      }
    }

  }
  else
  {
    input_nanoflann_ = MatrixPtr (new Eigen::MatrixXf (new float[original_no_of_points*point_representation_->getNumberOfDimensions ()], original_no_of_points, point_representation_->getNumberOfDimensions ()));
    float* cloud_ptr = input_nanoflann_->get();
    for (size_t indices_index = 0; indices_index < original_no_of_points; ++indices_index)
    {
      int cloud_index = (*indices_)[indices_index];
      const PointT&  point = (*input_)[cloud_index];
      // Check if the point is invalid
      if (!point_representation_->isValid (point))
      {
        identity_mapping_ = false;
        continue;
      }

      index_mapping_.push_back (static_cast<int> (indices_index));  // If the returned index should be for the indices vector

      point_representation_->vectorize (point, cloud_ptr);
      cloud_ptr += dim_;
    }
  }
  if (input_copied_for_nanoflann_)
    input_nanoflann_->rows = index_mapping_.size ();
}

#define PCL_INSTANTIATE_NanoFlannSearch(T) template class PCL_EXPORTS pcl::search::NanoFlannSearch<T>;

#endif
