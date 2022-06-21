

#pragma once

#include <pcl/pcl_base.h>
#include <pcl/search/search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/filters/morphological_filter.h>
#include <pcl/filters/extract_indices.h>

namespace pcl
{
  /** \brief
   * Implements the Progressive Morphological Filter for segmentation of ground points.
   * Description can be found in the article
   * "A Progressive Morphological Filter for Removing Nonground Measurements from
   * Airborne LIDAR Data"
   * by K. Zhang, S. Chen, D. Whitman, M. Shyu, J. Yan, and C. Zhang.
   */
  template <typename PointT>
  class Apmf : public pcl::PCLBase<PointT>
  {
  public:
    using PointCloud = pcl::PointCloud<PointT>;

    using PCLBase<PointT>::input_;
    using PCLBase<PointT>::indices_;
    using PCLBase<PointT>::initCompute;
    using PCLBase<PointT>::deinitCompute;

  public:
    ~Apmf(){};

    /** \brief Get the maximum window size to be used in filtering ground returns. */
    inline int
    getMaxWindowSize() const { return (max_window_size_); }
    /** \brief Set the maximum window size to be used in filtering ground returns. */
    inline void
    setMaxWindowSize(int max_window_size) { max_window_size_ = max_window_size; }

    /** \brief Get the slope value to be used in computing the height threshold. */
    inline float
    getSlope() const { return (slope_); }

    /** \brief Set the slope value to be used in computing the height threshold. */
    inline void
    setSlope(float slope) { slope_ = slope; }

    /** \brief Get the maximum height above the parameterized ground surface to be considered a ground return. */
    inline float
    getMaxDistance() const { return (max_distance_); }

    /** \brief Set the maximum height above the parameterized ground surface to be considered a ground return. */
    inline void
    setMaxDistance(float max_distance) { max_distance_ = max_distance; }

    /** \brief Get the initial height above the parameterized ground surface to be considered a ground return. */
    inline float
    getInitialDistance() const { return (initial_distance_); }

    /** \brief Set the initial height above the parameterized ground surface to be considered a ground return. */
    inline void
    setInitialDistance(float initial_distance) { initial_distance_ = initial_distance; }

    /** \brief Get the cell size. */
    inline float
    getCellSize() const { return (1.0f / cell_size_); }

    /** \brief Set the cell size. */
    inline void
    setCellSize(float cell_size) { cell_size_ = 1.0f / cell_size; }

    /** \brief Get the base to be used in computing progressive window sizes. */
    inline float
    getBase() const { return (base_); }

    /** \brief Set the base to be used in computing progressive window sizes. */
    inline void
    setBase(float base) { base_ = base; }

    /** \brief Get flag indicating whether or not to exponentially grow window sizes? */
    inline bool
    getExponential() const { return (exponential_); }

    /** \brief Set flag indicating whether or not to exponentially grow window sizes? */
    inline void
    setExponential(bool exponential) { exponential_ = exponential; }

    /** \brief Initialize the scheduler and set the number of threads to use.
     * \param nr_threads the number of hardware threads to use (0 sets the value back to automatic)
     */
    inline void
    setNumberOfThreads(unsigned int nr_threads = 0) { threads_ = nr_threads; }

    /** \brief This method launches the segmentation algorithm and returns indices of
     * points determined to be ground returns.
     * \param[out] ground indices of points determined to be ground returns.
     */
    virtual void
    extract(Indices &ground)
    {
      bool segmentation_is_possible = initCompute();
      if (!segmentation_is_possible)
      {
        deinitCompute();
        return;
      }

      // Compute the series of window sizes and height thresholds
      std::vector<float> height_thresholds;
      std::vector<float> window_sizes;
      std::vector<int> half_sizes;
      int iteration = 0;
      float window_size = 0.0f;

      while (window_size < max_window_size_)
      {
        // Determine the initial window size.
        int half_size = (exponential_) ? (static_cast<int>(std::pow(static_cast<float>(base_), iteration))) : ((iteration + 1) * base_);

        window_size = 2 * half_size + 1;
        // Calculate the height threshold to be used in the next iteration.
        float height_threshold = (iteration == 0) ? (initial_distance_) : (slope_ * (window_size - window_sizes[iteration - 1]) * cell_size_ + initial_distance_);
        // Enforce max distance on height threshold
        if (height_threshold > max_distance_)
          height_threshold = max_distance_;
        half_sizes.push_back(half_size);
        window_sizes.push_back(window_size);
        height_thresholds.push_back(height_threshold);
        iteration++;
      }
      // setup grid based on scale and extents
      Eigen::Vector4f global_max, global_min;
      //
      getMinMax3D<PointT>(*input_, global_min, global_max);
      // std::cout << global_min.matrix() << std::endl;
      // std::cout << global_max.matrix() << std::endl;

      float xextent = global_max.x() - global_min.x();
      float yextent = global_max.y() - global_min.y();

      int rows = static_cast<int>(std::floor(yextent * cell_size_) + 1);
      int cols = static_cast<int>(std::floor(xextent * cell_size_) + 1);

      Eigen::MatrixXf A(rows, cols);
      A.setConstant(std::numeric_limits<float>::quiet_NaN());

      Eigen::MatrixXf Z(rows, cols);
      Z.setConstant(std::numeric_limits<float>::quiet_NaN());

      Eigen::MatrixXf Zf(rows, cols);
      Zf.setConstant(std::numeric_limits<float>::quiet_NaN());

#pragma omp parallel for default(none) \
    shared(A, global_min)              \
        num_threads(threads_)
      for (int i = 0; i < (int)input_->size(); ++i)
      {
        // ...then test for lower points within the cell
        const PointT &p = (*input_)[i];
        if (!pcl::isFinite(p))
          continue;
        if (p.z > max_height_)
          continue;
        int row = std::floor((p.y - global_min.y()) * cell_size_);
        int col = std::floor((p.x - global_min.x()) * cell_size_);
        //
        if (p.z < A(row, col) || std::isnan(A(row, col)))
        {
          A(row, col) = p.z;
        }
      }

      // Ground indices are initially limited to those points in the input cloud we
      // wish to process
      ground = *indices_;
      // std::cout << __LINE__ << std::endl;
      // Progressively filter ground returns using morphological open
      for (std::size_t i = 0; i < window_sizes.size(); ++i)
      {
        PCL_DEBUG("      Iteration %d (height threshold = %f, window size = %f, half size = %d)...",
                  i, height_thresholds[i], window_sizes[i], half_sizes[i]);

        // Limit filtering to those points currently considered ground returns
        typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::copyPointCloud<PointT>(*input_, ground, *cloud);

        // Apply the morphological opening operation at the current window size.
#pragma omp parallel for default(none)      \
    shared(A, cols, half_sizes, i, rows, Z) \
        num_threads(threads_)
        for (int row = 0; row < rows; ++row)
        {
          int rs, re;
          rs = ((row - half_sizes[i]) < 0) ? 0 : row - half_sizes[i];
          re = ((row + half_sizes[i]) > (rows - 1)) ? (rows - 1) : row + half_sizes[i];

          for (int col = 0; col < cols; ++col)
          {
            int cs, ce;
            cs = ((col - half_sizes[i]) < 0) ? 0 : col - half_sizes[i];
            ce = ((col + half_sizes[i]) > (cols - 1)) ? (cols - 1) : col + half_sizes[i];

            float min_coeff = std::numeric_limits<float>::max();

            for (int j = rs; j < (re + 1); ++j)
            {
              for (int k = cs; k < (ce + 1); ++k)
              {
                if (A(j, k) != std::numeric_limits<float>::quiet_NaN())
                {
                  if (A(j, k) < min_coeff)
                    min_coeff = A(j, k);
                }
              }
            }

            if (min_coeff != std::numeric_limits<float>::max())
              Z(row, col) = min_coeff;
          }
        }
        // std::cout << __LINE__ << std::endl;
#pragma omp parallel for default(none)       \
    shared(cols, half_sizes, i, rows, Z, Zf) \
        num_threads(threads_)
        for (int row = 0; row < rows; ++row)
        {
          int rs, re;
          rs = ((row - half_sizes[i]) < 0) ? 0 : row - half_sizes[i];
          re = ((row + half_sizes[i]) > (rows - 1)) ? (rows - 1) : row + half_sizes[i];

          for (int col = 0; col < cols; ++col)
          {
            int cs, ce;
            cs = ((col - half_sizes[i]) < 0) ? 0 : col - half_sizes[i];
            ce = ((col + half_sizes[i]) > (cols - 1)) ? (cols - 1) : col + half_sizes[i];

            float max_coeff = -std::numeric_limits<float>::max();

            for (int j = rs; j < (re + 1); ++j)
            {
              for (int k = cs; k < (ce + 1); ++k)
              {
                if (Z(j, k) != std::numeric_limits<float>::quiet_NaN())
                {
                  if (Z(j, k) > max_coeff)
                    max_coeff = Z(j, k);
                }
              }
            }

            if (max_coeff != -std::numeric_limits<float>::max())
              Zf(row, col) = max_coeff;
          }
        }
        // std::cout << __LINE__ << std::endl;
        // Find indices of the points whose difference between the source and
        // filtered point clouds is less than the current height threshold.
        Indices pt_indices(230400, 0);
        for (std::size_t p_idx = 0; p_idx < ground.size(); ++p_idx)
        {
          const PointT &p = (*cloud)[p_idx];
          if (!pcl::isFinite(p))
            continue;
          if (p.z > max_height_)
            continue;
          int erow = static_cast<int>(std::floor((p.y - global_min.y()) * cell_size_));
          int ecol = static_cast<int>(std::floor((p.x - global_min.x()) * cell_size_));

          float diff = p.z - Zf(erow, ecol);
          if (diff < height_thresholds[i])
            pt_indices[p_idx] = 1; //.push_back(ground[p_idx]);
        }
        A.swap(Zf);
        // Ground is now limited to pt_indices
        ground.swap(pt_indices);
      }

      deinitCompute();
    };

  public:
    /** \brief Maximum window size to be used in filtering ground returns. */
    int max_window_size_;

    /** \brief Slope value to be used in computing the height threshold. */
    float slope_;

    /** \brief Maximum height above the parameterized ground surface to be considered a ground return. */
    float max_distance_;

    /** \brief Initial height above the parameterized ground surface to be considered a ground return. */
    float initial_distance_;

    /** \brief Cell size. */
    float cell_size_;

    /** \brief Base to be used in computing progressive window sizes. */
    float base_;

    /** \brief Exponentially grow window sizes? */
    bool exponential_;

    /** \brief Number of threads to be used. */
    unsigned int threads_;
    float max_height_;

  public:
    /** \brief Constructor that sets default values for member variables. */
    Apmf() : max_window_size_(33),
             slope_(0.7f),
             max_distance_(10.0f),
             initial_distance_(0.15f),
             cell_size_(1.0f / 1.0f),
             base_(2.0f),
             exponential_(true),
             threads_(0),
             max_height_(2.0f) {}
  };
}

#ifdef PCL_NO_PRECOMPILE
#endif
