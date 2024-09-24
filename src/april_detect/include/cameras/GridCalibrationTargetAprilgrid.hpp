#include <vector>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <apriltags/TagDetector.h>
#include <apriltags/Tag36h11.h>
#include "cameras/GridCalibrationTargetBase.hpp"

namespace cameras {
class GridCalibrationTargetAprilgrid : public cameras::GridCalibrationTargetBase{
public:
    struct AprilgridOptions {
        AprilgridOptions() :
            doSubpixRefinement(true),
            maxSubpixDisplacement2(1.5),
            showExtractionVideo(true),
            minTagsForValidObs(4),
            minBorderDistance(4.0),
            blackTagBorder(2) {}
        
        bool doSubpixRefinement;
        double maxSubpixDisplacement2;
        bool showExtractionVideo;
        int minTagsForValidObs;
        double minBorderDistance;
        int blackTagBorder;
    };
    enum {CLASS_SERIALIZATION_VERSION = 1};

  GridCalibrationTargetAprilgrid(int tagRows, int tagCols, double tagSize,
                                 double tagSpacing, const AprilgridOptions &options = AprilgridOptions());

  //serialization ctor
  GridCalibrationTargetAprilgrid();

  ~GridCalibrationTargetAprilgrid() {};

  /// \brief extract the calibration target points from an image and write to an observation
  bool computeObservation(const cv::Mat & image,
                          Eigen::MatrixXd & outImagePoints,
                          std::vector<bool> &outCornerObserved) const;

 private:
  /// \brief initialize the object
  void initialize();

  /// \brief initialize the grid with the points
  void createGridPoints();

  /// \brief size of a tag [m]
  double _tagSize;

  /// \brief space between tags (tagSpacing [m] = tagSize * tagSpacing)
  double _tagSpacing;
  int _tagRows;
  int _tagCols;
  /// \brief target extraction options
  AprilgridOptions _options;

  // create a detector instance
  AprilTags::TagCodes _tagCodes;
  AprilTags::TagDetector* _tagDetector;


};
}//namespace