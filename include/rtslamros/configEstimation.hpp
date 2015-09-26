#ifndef RTSLAMROS_CONFIG_H_
#define RTSLAMROS_CONFIG_H_

#include <string>
#include <stdint.h>
#include <stdio.h>

namespace rtslamros {

using std::string;

/// Global configuration file of RTSLAM estimation.
/// Implements the Singleton design pattern to allow global access and to ensure
/// that only one instance exists.
class ConfigEstimation
{
public:
  static ConfigEstimation& getInstance();

  /// number of coefficients for the distortion correction polynomial
  static unsigned& correctionSize() { return getInstance().correction_size; }

  /// map size in # of states, robot + landmarks
  static unsigned& mapSize() { return getInstance().map_size; }

  /// measurement noise of a point
  static double& pixNoise() { return getInstance().pix_noise; }

  /// inverse depth mean initialization
  static double& dMin() { return getInstance().d_min; }

  /// reparametrization threshold
  static double& reparamTh() { return getInstance().reparam_th; }


  /// number of horizontal cells of the image grid for landmark density control
  static unsigned& gridHcells() { return getInstance().grid_hcells; }

  /// number of vertical cells
  static unsigned& gridVcells() { return getInstance().grid_vcells; }

  /// min margin of a cell that must be in the image when shifting the grid
  static unsigned& gridMargin() { return getInstance().grid_margin; }

  /// min separation between landmarks in the image for creation (margin with the border of the cell where landmarks can be initialized)
  static unsigned& gridSepar() { return getInstance().grid_separ; }


  /// relevance threshold to make an update (# of sigmas)
  static double& relevanceTh() { return getInstance().relevance_th; }

  /// mahalanobis distance for gating (# of sigmas)
  static double& mahalanobisTh() { return getInstance().mahalanobis_th; }

  /// max number of landmarks to update every frame
  static unsigned& nUpdatesTotal() { return getInstance().n_updates_total; }

  /// max number of landmarks to update with ransac every frame
  static unsigned& nUpdatesRansac() { return getInstance().n_updates_ransac; }

  /// maximum number of landmarks to try to initialize every frame
  static unsigned& nInit() { return getInstance().n_init; }

  /// how many times information gain is recomputed to resort observations in active search
  static unsigned& nRecompGains() { return getInstance().n_recomp_gains; }

  /// ransac low innovation threshold (pixels)
  static double& ransacLowInnov() { return getInstance().ransac_low_innov; }


  /// number of base observation used to initialize a ransac set
  static unsigned& ransacNtries() { return getInstance().ransac_ntries; }

  /// make multiple depth hypotheses when search ellipses are too big and distortion too strong
  static bool& multipleDepthHypos() { return getInstance().multiple_depth_hypos; }


  /// harris detector convolution size
  static unsigned& harrisConvSize() { return getInstance().harris_conv_size; }

  /// harris threshold
  static double& harrisTh() { return getInstance().harris_th; }

  /// harris symmetry factor
  static double& harrisEdge() { return getInstance().harris_edge; }


  /// descriptor patch size (odd value)
  static unsigned& descSize() { return getInstance().desc_size; }

  /// whether use or not the multiview descriptor
  static bool& multiviewDescriptor() { return getInstance().multiview_descriptor; }

  /// MultiviewDescriptor: min change of scale (ratio)
  static double& descScaleStep() { return getInstance().desc_scale_step; }

  /// MultiviewDescriptor: min change of point of view (deg)
  static double& descAngleStep() { return getInstance().desc_angle_step; }

  /// type of prediction from descriptor (0 = none, 1 = affine, 2 = homographic)
  static int& descPredictionType() { return getInstance().desc_prediction_type; }


  /// patch size used for matching
  static unsigned& patchSize() { return getInstance().patch_size; }

  /// if the search area is larger than this # of pixels, we bound it
  static unsigned& maxSearchSize() { return getInstance().max_search_size; }

  /// if the search area is larger than this # of pixels, we vote for killing the landmark
  static unsigned& killSearchSize() { return getInstance().kill_search_size; }

  /// ZNCC score threshold
  static double& matchTh() { return getInstance().match_th; }

  /// min ZNCC score under which we don't finish to compute the value of the score
  static double& minScore() { return getInstance().min_score; }

  /// higher ZNCC score threshold for landmarks with high depth uncertainty and high expectation uncertainty
  static double& hiMatchTh() { return getInstance().hi_match_th; }

  /// limit in pixels of the expectation uncertainty to use HI_MATCH_TH
  static double& hiLimit() { return getInstance().hi_limit; }

  /// position in the patch where we test if we finish the correlation computation
  static double& partialPosition() { return getInstance().partial_position; }

private:
  ConfigEstimation();
  ConfigEstimation(ConfigEstimation const&);
  void operator=(ConfigEstimation const&);

  // MISC
  unsigned correction_size; ///< number of coefficients for the distortion correction polynomial

  // FILTER
  unsigned map_size; ///< map size in # of states, robot + landmarks
  double pix_noise;  ///< measurement noise of a point

  // LANDMARKS
  double d_min;      ///< inverse depth mean initialization
  double reparam_th; ///< reparametrization threshold

  unsigned grid_hcells; ///< number of horizontal cells of the image grid for landmark density control
  unsigned grid_vcells; ///< number of vertical cells
  unsigned grid_margin; ///< min margin of a cell that must be in the image when shifting the grid
  unsigned grid_separ;  ///< min separation between landmarks in the image for creation (margin with the border of the cell where landmarks can be initialized)

  double relevance_th;       ///< relevance threshold to make an update (# of sigmas)
  double mahalanobis_th;     ///< mahalanobis distance for gating (# of sigmas)
  unsigned n_updates_total;  ///< max number of landmarks to update every frame
  unsigned n_updates_ransac; ///< max number of landmarks to update with ransac every frame
  unsigned n_init;           ///< maximum number of landmarks to try to initialize every frame
  unsigned n_recomp_gains;   ///< how many times information gain is recomputed to resort observations in active search
  double ransac_low_innov;   ///< ransac low innovation threshold (pixels)

  unsigned ransac_ntries;    ///< number of base observation used to initialize a ransac set
  bool multiple_depth_hypos; ///< make multiple depth hypotheses when search ellipses are too big and distortion too strong

  // RAW PROCESSING
  unsigned harris_conv_size; ///< harris detector convolution size
  double harris_th;          ///< harris threshold
  double harris_edge;       ///< harris symmetry factor

  unsigned desc_size;        ///< descriptor patch size (odd value)
  bool multiview_descriptor; ///< whether use or not the multiview descriptor
  double desc_scale_step;    ///< MultiviewDescriptor: min change of scale (ratio)
  double desc_angle_step;    ///< MultiviewDescriptor: min change of point of view (deg)
  int desc_prediction_type;  ///< type of prediction from descriptor (0 = none, 1 = affine, 2 = homographic)

  unsigned patch_size;       ///< patch size used for matching
  unsigned max_search_size;  ///< if the search area is larger than this # of pixels, we bound it
  unsigned kill_search_size; ///< if the search area is larger than this # of pixels, we vote for killing the landmark
  double match_th;           ///< ZNCC score threshold
  double min_score;          ///< min ZNCC score under which we don't finish to compute the value of the score
  double hi_match_th;        ///< higher ZNCC score threshold for landmarks with high depth uncertainty and high expectation uncertainty
  double hi_limit;           ///< limit in pixels of the expectation uncertainty to use HI_MATCH_TH
  double partial_position;   ///< position in the patch where we test if we finish the correlation computation

};

} // namespace rtslamros

#endif // RTSLAMROS_CONFIG_H_
