
#include <vector>
#include <utility>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>


namespace cameras {

/**
 * \class GridCalibrationTarget
 * \brief a class that represents a calibration target with known geometry
 *
 * The class is a little limiting:
 * The target is supposed to be square such that each row has the same number of points.
 * Points along a row are supposed to be nominally colinear.
 * Points along a column are supposed to be nominally colinear.
 * Missing points are not allowed.
 *
 */
class GridCalibrationTargetBase {
 public:
  /// \brief initialize base class
  GridCalibrationTargetBase(size_t rows, size_t cols);

  ~GridCalibrationTargetBase() {};

 public:

  //typedef boost::shared_ptr<GridCalibrationTargetBase> Ptr;
  //typedef boost::shared_ptr<const GridCalibrationTargetBase> ConstPtr;

  /// \brief get the number of points of the full grid
  size_t size() const {
    return _rows * _cols;
  };

  /// \brief the number of rows in the calibration target
  size_t rows() const {
    return _rows;
  };

  /// \brief the number of columns in the calibration target
  size_t cols() const {
    return _cols;
  };

  /// \brief get a point from the target expressed in the target frame
  Eigen::Vector3d point(size_t i) const;

  /// \brief get all points from the target expressed in the target frame
  Eigen::MatrixXd points() const;

  /// \brief get the grid coordinates for a point
  std::pair<size_t, size_t> pointToGridCoordinates(size_t i) const;

  /// \brief get the point index from the grid coordinates
  size_t gridCoordinatesToPoint(size_t r, size_t c) const;

  /// \brief get a point from the target expressed in the target frame
  ///        by row and column
  Eigen::Vector3d gridPoint(size_t r, size_t c) const;

  /// \brief extract the calibration target points from an image
  ///        outCornerObserved flags wheter the corresponding point
  ///        in outImagePoints was observed
 bool computeObservation(const cv::Mat & /*image*/,
                                  Eigen::MatrixXd & /*outImagePoints*/,
                                  std::vector<bool> & /*outCornerObserved*/) const
  {
    return false;
  };

  /// \brief return pointer to the i-th grid point in target frame
  double * getPointDataPointer(size_t i);

 protected:
  /// \brief grid points stored in row-major order (idx = cols * r + c)
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> _points;

  /// \brief the number of point rows in the calibration target
  size_t _rows;

  /// \brief the number of point columns in the calibration target
  size_t _cols;

  ///////////////////////////////////////////////////
  // Serialization support
  ///////////////////////////////////////////////////
 public:
  //BOOST_SERIALIZATION_SPLIT_MEMBER()

  //serialization ctor
  GridCalibrationTargetBase() {};

 protected:
  //friend class boost::serialization::access;

}; //class GridCalibrationTargetBase

} // namespace cameras

