/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id: kdtree_nanoflann.h 36261 2011-02-26 01:34:42Z ------- $
 *
 */

#ifndef PCL_KDTREE_KDTREE_NANOFLANN_H_
#define PCL_KDTREE_KDTREE_NANOFLANN_H_

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/nanoflann.h>

#include <boost/shared_array.hpp>


namespace pcl
{

  // And this is the "dataset to kd-tree" adaptor class: 
  template <typename Derived>
  struct PointCloudAdaptor
  { 
      typedef typename Derived::coord_t coord_t;

      const Derived &obj; //!< A const ref to the data set origin

      /// The constructor that sets the data set source
      PointCloudAdaptor(const Derived &obj_) : obj(obj_) { }

      /// CRTP helper method 
      inline const Derived& derived() const { return obj; }

      // Must return the number of data points 
      inline size_t kdtree_get_point_count() const { return derived().pts.size(); }

      // Returns the dim'th component of the idx'th point in the class: 
      // Since this is inlined and the "dim" argument is typically an immediate value, the 
      //  "if/else's" are actually solved at compile time. 
      // inline coord_t kdtree_get_pt(const size_t idx, int dim) const
      inline coord_t kdtree_get_pt(const size_t idx, int dim) const
      {
          if (dim == 0) return derived().pts[idx].x;
          else if (dim == 1) return derived().pts[idx].y;
          else return derived().pts[idx].z;
      }

      // Optional bounding-box computation: return false to default to a standard bbox computation loop. 
      //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again. 
      //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds) 
      template <class BBOX> 
      bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; } 
  }; // end of PointCloudAdaptor 

  // reference code : nanoflann/examples/util.h
  template <typename T>
  struct SearchPointCloud
  {
    typedef T coord_t;

    struct Point
    {
      T  x, y ,z;
    }; 

  public:
    std::vector<Point> pts;

    // Must return the number of data points 
    inline size_t kdtree_get_point_count() const { return pts.size(); } 

    // Returns the dim'th component of the idx'th point in the class: 
    // Since this is inlined and the "dim" argument is typically an immediate value, the 
    //  "if/else's" are actually solved at compile time. 
    inline T kdtree_get_pt(const size_t idx, const size_t dim) const 
    { 
      if (dim == 0) return pts[idx].x;
      else if (dim == 1) return pts[idx].y;
      else return pts[idx].z;
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop. 
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again. 
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds) 
    template <class BBOX> 
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; } 
  };

  // Forward declarations
  template <typename T> class PointRepresentation;

  /** \brief KdTreeNANOFLANN is a generic type of 3D spatial locator using kD-tree structures. 
    * The class is making use of the FLANN (Fast Library for Approximate Nearest Neighbor) project by jlblancoc.
    * 
    * \author 
    * \ingroup kdtree 
    */
  template <typename PointT>
  class KdTreeNANOFLANN : public pcl::KdTree<PointT>
  {
    public:
      using KdTree<PointT>::input_;
      using KdTree<PointT>::indices_;
      using KdTree<PointT>::epsilon_;
      using KdTree<PointT>::sorted_;
      using KdTree<PointT>::point_representation_;
      using KdTree<PointT>::nearestKSearch;
      using KdTree<PointT>::radiusSearch;

      typedef typename KdTree<PointT>::PointCloud PointCloud;
      typedef typename KdTree<PointT>::PointCloudConstPtr PointCloudConstPtr;

      typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
      typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

      // typedef PointCloudAdaptor<pcl::PointCloud<PointT>> PC2KD;
      typedef PointCloudAdaptor<SearchPointCloud<float>> PC2KD;
      // typedef SearchPointCloud<float> PC2KD;
      typedef nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, PC2KD>,
        PC2KD,
        3> NANOFLANNIndex;

      // typedef ::flann::Index<Dist> FLANNIndex;
      // typedef nanoflann::Index<Dist> FLANNIndex;


      // Boost shared pointers
      typedef boost::shared_ptr<KdTreeNANOFLANN<PointT> > Ptr;
      typedef boost::shared_ptr<const KdTreeNANOFLANN<PointT> > ConstPtr;

      /** \brief Default Constructor for KdTreeNANOFLANN.
        * \param[in] sorted set to true if the application that the tree will be used for requires sorted nearest neighbor indices (default). False otherwise. 
        *
        * By setting sorted to false, the \ref radiusSearch operations will be faster.
        */
      KdTreeNANOFLANN (bool sorted = true);

      /** \brief Copy constructor
        * \param[in] k the tree to copy into this
        */
      KdTreeNANOFLANN (const KdTreeNANOFLANN<PointT> &k);

      /** \brief Copy operator
        * \param[in] k the tree to copy into this
        */ 
      inline KdTreeNANOFLANN<PointT>&
      operator = (const KdTreeNANOFLANN<PointT>& k)
      {
        KdTree<PointT>::operator=(k);
        flann_index_ = k.flann_index_;
        cloud_ = k.cloud_;
        index_mapping_ = k.index_mapping_;
        identity_mapping_ = k.identity_mapping_;
        dim_ = k.dim_;
        total_nr_points_ = k.total_nr_points_;
        param_k_ = k.param_k_;
        param_radius_ = k.param_radius_;
        return (*this);
      }

      /** \brief Set the search epsilon precision (error bound) for nearest neighbors searches.
        * \param[in] eps precision (error bound) for nearest neighbors searches
        */
      void
      setEpsilon (float eps);

      void 
      setSortedResults (bool sorted);
      
      inline Ptr makeShared () { return Ptr (new KdTreeNANOFLANN<PointT> (*this)); } 

      /** \brief Destructor for KdTreeNANOFLANN. 
        * Deletes all allocated data arrays and destroys the kd-tree structures. 
        */
      virtual ~KdTreeNANOFLANN ()
      {
        cleanup ();
      }

      /** \brief Provide a pointer to the input dataset.
        * \param[in] cloud the const boost shared pointer to a PointCloud message
        * \param[in] indices the point indices subset that is to be used from \a cloud - if NULL the whole cloud is used
        */
      void 
      setInputCloud (const PointCloudConstPtr &cloud, const IndicesConstPtr &indices = IndicesConstPtr ());

      /** \brief Search for k-nearest neighbors for the given query point.
        * 
        * \attention This method does not do any bounds checking for the input index
        * (i.e., index >= cloud.points.size () || index < 0), and assumes valid (i.e., finite) data.
        * 
        * \param[in] point a given \a valid (i.e., finite) query point
        * \param[in] k the number of neighbors to search for
        * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
        * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k 
        * a priori!)
        * \return number of neighbors found
        * 
        * \exception asserts in debug mode if the index is not between 0 and the maximum number of points
        */
      int 
      nearestKSearch (const PointT &point, int k, 
                      std::vector<int> &k_indices, std::vector<float> &k_sqr_distances) const;

      /** \brief Search for all the nearest neighbors of the query point in a given radius.
        * 
        * \attention This method does not do any bounds checking for the input index
        * (i.e., index >= cloud.points.size () || index < 0), and assumes valid (i.e., finite) data.
        * 
        * \param[in] point a given \a valid (i.e., finite) query point
        * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
        * \param[out] k_indices the resultant indices of the neighboring points
        * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
        * \param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to
        * 0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be
        * returned.
        * \return number of neighbors found in radius
        *
        * \exception asserts in debug mode if the index is not between 0 and the maximum number of points
        */
      int 
      radiusSearch (const PointT &point, double radius, std::vector<int> &k_indices,
                    std::vector<float> &k_sqr_distances, unsigned int max_nn = 0) const;

    private:
      /** \brief Internal cleanup method. */
      void 
      cleanup ();

      /** \brief Converts a PointCloud to the internal NANOFLANN point array representation. Returns the number
        * of points.
        * \param cloud the PointCloud 
        */
      void 
      convertCloudToArray (const PointCloud &cloud);

      /** \brief Converts a PointCloud with a given set of indices to the internal NANOFLANN point array
        * representation. Returns the number of points.
        * \param[in] cloud the PointCloud data
        * \param[in] indices the point cloud indices
       */
      void 
      convertCloudToArray (const PointCloud &cloud, const std::vector<int> &indices);

    private:
      /** \brief Class getName method. */
      virtual std::string 
      getName () const { return ("KdTreeNANOFLANN"); }

      /** \brief A FLANN index object. */
      boost::shared_ptr<NANOFLANNIndex> flann_index_;

      /** \brief Internal pointer to data. */
      boost::shared_array<float> cloud_;
      
      /** \brief mapping between internal and external indices. */
      std::vector<int> index_mapping_;
      
      /** \brief whether the mapping bwwteen internal and external indices is identity */
      bool identity_mapping_;

      /** \brief Tree dimensionality (i.e. the number of dimensions per point). */
      int dim_;

      /** \brief The total size of the data (either equal to the number of points in the input cloud or to the number of indices - if passed). */
      int total_nr_points_;

      /** \brief The KdTree search parameters for K-nearest neighbors. */
      nanoflann::SearchParams param_k_;

      /** \brief The KdTree search parameters for radius search. */
      nanoflann::SearchParams param_radius_;

      SearchPointCloud<float> cloud_pt;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/kdtree/impl/kdtree_nanoflann.hpp>
#endif

#endif
