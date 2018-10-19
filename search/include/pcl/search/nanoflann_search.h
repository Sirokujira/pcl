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
 */

#ifndef PCL_SEARCH_NANOFLANN_SEARCH_H_
#define PCL_SEARCH_NANOFLANN_SEARCH_H_

#include <memory>

#include <pcl/search/search.h>
#include <pcl/common/time.h>
#include <pcl/point_representation.h>
#include <nanoflann.hpp>

#include <Eigen/Dense>

// #define EIGEN_STATIC_ASSERT
#define EIGEN_NO_STATIC_ASSERT

namespace pcl
{
  namespace search
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

    /** \brief @b search::NanoFlannSearch is a generic NANOFLANN wrapper class for the new search interface.
      * It is able to wrap any NANOFLANN index type, e.g. the kd tree as well as indices for high-dimensional
      * searches and intended as a more powerful and cleaner successor to KdTreeFlann.
      * 
      * By default, this class creates a single kd tree for indexing the input data. However, for high dimensions
      * (> 10), it is often better to use the multiple randomized kd tree index provided by NANOFLANN in combination with
      * the \ref flann::L2 distance functor. During search in this type of index, the number of checks to perform before
      * terminating the search can be controlled. Here is a code example if a high-dimensional 2-NN search:
      * 
      * \code
      * // Feature and distance type
      * typedef SHOT352 FeatureT;
      * 
      * // Search and index types
      * typedef search::NanoFlannSearch<FeatureT> SearchT;
      * typedef typename SearchT::NanoFlannIndexCreatorPtr CreatorPtrT;
      * typedef typename SearchT::KdTreeMultiIndexCreator IndexT;
      * typedef typename SearchT::PointRepresentationPtr RepresentationPtrT;
      * 
      * // Features
      * PointCloud<FeatureT>::Ptr query, target;
      * 
      * // Fill query and target with calculated features...
      * 
      * // Instantiate search object with 4 randomized trees and 256 checks
      * SearchT search (true, CreatorPtrT (new IndexT (4)));
      * search.setPointRepresentation (RepresentationPtrT (new DefaultFeatureRepresentation<FeatureT>));
      * search.setChecks (256);
      * search.setInputCloud (target);
      * 
      * // Do search
      * std::vector<std::vector<int> > k_indices;
      * std::vector<std::vector<float> > k_sqr_distances;
      * search.nearestKSearch (*query, std::vector<int> (), 2, k_indices, k_sqr_distances);
      * \endcode
      *
      * \author ---
      * \author --- (multiple randomized kd tree interface)
      * \ingroup search
      */
    template<typename PointT>
    class NanoFlannSearch : public Search<PointT>
    {
      using Search<PointT>::input_;
      using Search<PointT>::indices_;
      using Search<PointT>::sorted_results_;

      public:
    	// EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef boost::shared_ptr<NanoFlannSearch<PointT> > Ptr;
        typedef boost::shared_ptr<const NanoFlannSearch<PointT> > ConstPtr;

        typedef typename Search<PointT>::PointCloud PointCloud;
        typedef typename Search<PointT>::PointCloudConstPtr PointCloudConstPtr;

        typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
        typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

        // 代替えチェック(必要か確認)
        // typedef boost::shared_ptr<flann::Matrix <float> > MatrixPtr;
        // typedef boost::shared_ptr<const flann::Matrix <float> > MatrixConstPtr;
        // build NG(Eigen Initialize Error?)
        // typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Matrix;
        // typedef Eigen::MatrixXf Matrix;
        typedef boost::shared_ptr<Eigen::MatrixXf> MatrixPtr;
        typedef boost::shared_ptr<const Eigen::MatrixXf> MatrixConstPtr;
        // 初期化時のダミー用変数
        // float* init_dummy_data;

        // 索引の作成?
        // 代替えチェック?(必要か確認)
        // typedef flann::NNIndex< FlannDistance > Index;
        // typedef boost::shared_ptr<flann::NNIndex> IndexPtr;
        // nanoflann 内部での treeIndex がこれにあたる？
        // typedef nanoflann::NNIndex Index;
        // typedef std::unique_ptr<nanoflann::NNIndex> IndexPtr;
        // typedef PointCloudAdaptor<SearchPointCloud<float>> PC2KD;
        typedef SearchPointCloud<float> PC2KD;
        // typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, PC2KD>, PC2KD> Index;
        // typedef boost::shared_ptr< nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, PC2KD>, PC2KD> > IndexPtr;

        typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > Index;
        typedef boost::shared_ptr<Index> IndexPtr;
        // typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> >* IndexPtr;
        // typedef int Index;
        // typedef void* IndexPtr;
        // NG(need Set Dimension)
        // typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>, nanoflann::metric_L2> Index_L2;
        // typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>, nanoflann::metric_L2_Simple> Index_L2_Simple;
        // typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>, nanoflann::metric_L1> Index_L1;
        // typedef boost::shared_ptr< nanoflann::KDTreeEigenMatrixAdaptor<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>, nanoflann::metric_L2> > IndexPtr;

        typedef pcl::PointRepresentation<PointT> PointRepresentation;
        typedef boost::shared_ptr<PointRepresentation> PointRepresentationPtr;
        typedef boost::shared_ptr<const PointRepresentation> PointRepresentationConstPtr;

        /** \brief Helper class that creates a NanoFLANN index from a given NanoFLANN matrix. To
          * use a NanoFLANN index type with FlannSearch, implement this interface and
          * pass an object of the new type to the NanoFlannSearch constructor.
          * See the implementation of KdTreeIndexCreator for an example.
          */
        class NanoFlannIndexCreator
        {
          public:
          /** \brief Create a NanoFLANN Index from the input data.
            * \param[in] data The NanoFLANN matrix containing the input.
            * \return The NanoFLANN index.
            */
            // virtual IndexPtr createIndex (MatrixConstPtr data)=0;
            virtual IndexPtr createIndex (MatrixConstPtr data)=0;

          /** \brief destructor 
            */
            virtual ~NanoFlannIndexCreator () {}

          SearchPointCloud<float> cloud_pt;
        };
        typedef boost::shared_ptr<NanoFlannIndexCreator> NanoFlannIndexCreatorPtr;

        /** \brief Creates a FLANN KdTreeSingleIndex from the given input data.
          */
        class KdTreeIndexCreator: public NanoFlannIndexCreator
        {
          public:
          /** \param[in] max_leaf_size All NanoFLANN kd trees created by this class will have
            * a maximum of max_leaf_size points per leaf node. Higher values make index creation
            * cheaper, but search more costly (and the other way around).
            */
            KdTreeIndexCreator (unsigned int max_leaf_size=15) : max_leaf_size_ (max_leaf_size){}
      
            /** \brief Empty destructor */
            virtual ~KdTreeIndexCreator () {}

          /** \brief Create a NanoFLANN Index from the input data.
            * \param[in] data The NanoFLANN matrix containing the input.
            * \return The NanoFLANN index.
            */
            virtual IndexPtr createIndex (MatrixConstPtr data);
          private:
            unsigned int max_leaf_size_;
        };

        /** \brief Creates a NanoFLANN KdTreeSingleIndex from the given input data.
          */
        class KMeansIndexCreator: public NanoFlannIndexCreator
        {
          public:
          /** \brief All NanoFLANN kd trees created by this class will have
            * a maximum of max_leaf_size points per leaf node. Higher values make index creation
            * cheaper, but search more costly (and the other way around).
            */
            KMeansIndexCreator (){}
            
            /** \brief Empty destructor */
            virtual ~KMeansIndexCreator () {}

          /** \brief Create a NanoFLANN Index from the input data.
            * \param[in] data The NanoFLANN matrix containing the input.
            * \return The NanoFLANN index.
            */
            virtual IndexPtr createIndex (MatrixConstPtr data);
          private:
        };

        /** \brief Creates a NanoFLANN KdTreeIndex of multiple randomized trees from the given input data,
         *  suitable for feature matching. Note that in this case, it is often more efficient to use the
         *  \ref flann::L2 distance functor.
          */
        class KdTreeMultiIndexCreator: public NanoFlannIndexCreator
        {
          public:
          /** \param[in] trees Number of randomized trees to create.
            */
            KdTreeMultiIndexCreator (int trees = 4) : trees_ (trees) {}
      
            /** \brief Empty destructor */
            virtual ~KdTreeMultiIndexCreator () {}

          /** \brief Create a NanoFLANN Index from the input data.
            * \param[in] data The NanoFLANN matrix containing the input.
            * \return The NanoFLANN index.
            */
            virtual IndexPtr createIndex (MatrixConstPtr data);
          private:
            int trees_;
        };

        NanoFlannSearch (bool sorted = true, NanoFlannIndexCreatorPtr creator = NanoFlannIndexCreatorPtr (new KdTreeIndexCreator ()));

        /** \brief Destructor for NanoFlannSearch. */
        virtual
        ~NanoFlannSearch() = default;

        /** \brief Set the search epsilon precision (error bound) for nearest neighbors searches.
          * \param[in] eps precision (error bound) for nearest neighbors searches
          */
        inline void
        setEpsilon (double eps)
        {
          eps_ = eps;
        }

        /** \brief Get the search epsilon precision (error bound) for nearest neighbors searches. */
        inline double
        getEpsilon ()
        {
          return (eps_);
        }

        /** \brief Set the number of checks to perform during approximate searches in multiple randomized trees.
          * \param[in] checks number of checks to perform during approximate searches in multiple randomized trees.
          */
        inline void
        setChecks (int checks)
        {
          checks_ = checks;
        }

        /** \brief Get the number of checks to perform during approximate searches in multiple randomized trees. */
        inline int
        getChecks ()
        {
          return (checks_);
        }

        /** \brief Provide a pointer to the input dataset.
          * \param[in] cloud the const boost shared pointer to a PointCloud message
          * \param[in] indices the point indices subset that is to be used from \a cloud
          */
        virtual void
        setInputCloud (const PointCloudConstPtr& cloud, const IndicesConstPtr& indices = IndicesConstPtr ());

        /** \brief Search for the k-nearest neighbors for the given query point.
          * \param[in] point the given query point
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
          * a priori!)
          * \return number of neighbors found
          */
        int
        nearestKSearch (const PointT &point, int k, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances) const;


        /** \brief Search for the k-nearest neighbors for the given query point.
          * \param[in] cloud the point cloud data
          * \param[in] indices a vector of point cloud indices to query for nearest neighbors
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points, k_indices[i] corresponds to the neighbors of the query point i
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points, k_sqr_distances[i] corresponds to the neighbors of the query point i
          */
        virtual void
        nearestKSearch (const PointCloud& cloud, const std::vector<int>& indices, int k, 
                        std::vector< std::vector<int> >& k_indices, std::vector< std::vector<float> >& k_sqr_distances) const;

        /** \brief Search for all the nearest neighbors of the query point in a given radius.
          * \param[in] point the given query point
          * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
          * \param[out] k_indices the resultant indices of the neighboring points
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
          * \param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to
          * 0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be
          * returned.
          * \return number of neighbors found in radius
          */
        int
        radiusSearch (const PointT& point, double radius, 
                      std::vector<int> &k_indices, std::vector<float> &k_sqr_distances,
                      unsigned int max_nn = 0) const;

        /** \brief Search for the k-nearest neighbors for the given query point.
          * \param[in] cloud the point cloud data
          * \param[in] indices a vector of point cloud indices to query for nearest neighbors
          * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
          * \param[out] k_indices the resultant indices of the neighboring points, k_indices[i] corresponds to the neighbors of the query point i
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points, k_sqr_distances[i] corresponds to the neighbors of the query point i
          * \param[in] max_nn if given, bounds the maximum returned neighbors to this value
          */
        virtual void
        radiusSearch (const PointCloud& cloud, const std::vector<int>& indices, double radius, 
            std::vector< std::vector<int> >& k_indices, std::vector< std::vector<float> >& k_sqr_distances, unsigned int max_nn=0) const;

        /** \brief Provide a pointer to the point representation to use to convert points into k-D vectors.
          * \param[in] point_representation the const boost shared pointer to a PointRepresentation
          */
        inline void
        setPointRepresentation (const PointRepresentationConstPtr &point_representation)
        {
          point_representation_ = point_representation;
          dim_ = point_representation->getNumberOfDimensions ();
          if (input_) // re-create the tree, since point_representation might change things such as the scaling of the point clouds.
            setInputCloud (input_, indices_);
        }

        /** \brief Get a pointer to the point representation used when converting points into k-D vectors. */
        inline PointRepresentationConstPtr const
        getPointRepresentation ()
        {
          return (point_representation_);
        }

      protected:

        /** \brief converts the input data to a format usable by NanoFLANN
          */
        void convertInputToNanoFlannMatrix();

        /** The NanoFLANN index.
          */
        IndexPtr index_;

        /** The index creator, used to (re-) create the index when the search data is passed.
          */
        NanoFlannIndexCreatorPtr creator_;

        /** Input data in FLANN format.
          */
        MatrixPtr input_nanoflann_;

        /** Epsilon for approximate NN search.
          */
        float eps_;
        
        /** Number of checks to perform for approximate NN search using the multiple randomized tree index
         */
        int checks_;
        
        bool input_copied_for_nanoflann_;

        PointRepresentationConstPtr point_representation_;

        // PointCloud data dimension?
        int dim_;

        std::vector<int> index_mapping_;
        bool identity_mapping_;

    };
  }
}

#define PCL_INSTANTIATE_NanoFlannSearch(T) template class PCL_EXPORTS pcl::search::NanoFlannSearch<T>;

#endif    // PCL_SEARCH_NANOFLANN_SEARCH_H_

