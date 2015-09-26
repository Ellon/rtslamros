
#include <vikit/params_helper.h>
#include <rtslamros/configEstimation.hpp>

namespace rtslamros {

ConfigEstimation::ConfigEstimation() :
   correction_size(vk::getParam<int>("rtslam/CORRECTION_SIZE", 4)),
   map_size(vk::getParam<int>("rtslam/MAP_SIZE", 500)),
   pix_noise(vk::getParam<double>("rtslam/PIX_NOISE", 1.0)),
   d_min(vk::getParam<double>("rtslam/D_MIN", 0.3)),
   reparam_th(vk::getParam<double>("rtslam/REPARAM_TH", 0.1)),
   grid_hcells(vk::getParam<int>("rtslam/GRID_HCELLS", 6)),
   grid_vcells(vk::getParam<int>("rtslam/GRID_VCELLS", 4)),
   grid_margin(vk::getParam<int>("rtslam/GRID_MARGIN", 5)),
   grid_separ(vk::getParam<int>("rtslam/GRID_SEPAR", 7)),
   relevance_th(vk::getParam<double>("rtslam/RELEVANCE_TH", 2.0)),
   mahalanobis_th(vk::getParam<double>("rtslam/MAHALANOBIS_TH", 3.0)),
   n_updates_total(vk::getParam<int>("rtslam/N_UPDATES_TOTAL", 50)),
   n_updates_ransac(vk::getParam<int>("rtslam/N_UPDATES_RANSAC", 22)),
   n_init(vk::getParam<int>("rtslam/N_INIT", 20)),
   n_recomp_gains(vk::getParam<int>("rtslam/N_RECOMP_GAINS", 2)),
   ransac_low_innov(vk::getParam<double>("rtslam/RANSAC_LOW_INNOV", 1.0)),
   ransac_ntries(vk::getParam<int>("rtslam/RANSAC_NTRIES", 6)),
   multiple_depth_hypos(vk::getParam<bool>("rtslam/MULTIPLE_DEPTH_HYPOS", false)),
   harris_conv_size(vk::getParam<int>("rtslam/HARRIS_CONV_SIZE", 5)),
   harris_th(vk::getParam<double>("rtslam/HARRIS_TH", 5.0)),
   harris_edge(vk::getParam<double>("rtslam/HARRIS_EDGE", 1.4)),
   desc_size(vk::getParam<int>("rtslam/DESC_SIZE", 21)),
   multiview_descriptor(vk::getParam<bool>("rtslam/MULTIVIEW_DESCRIPTOR", false)),
   desc_scale_step(vk::getParam<double>("rtslam/DESC_SCALE_STEP", 1.5)),
   desc_angle_step(vk::getParam<double>("rtslam/DESC_ANGLE_STEP", 5.0)),
   desc_prediction_type(vk::getParam<int>("rtslam/DESC_PREDICTION_TYPE", 1)),
   patch_size(vk::getParam<int>("rtslam/PATCH_SIZE", 11)),
   max_search_size(vk::getParam<int>("rtslam/MAX_SEARCH_SIZE", 10000)),
   kill_search_size(vk::getParam<int>("rtslam/KILL_SEARCH_SIZE", 100000)),
   match_th(vk::getParam<double>("rtslam/MATCH_TH", 0.75)),
   min_score(vk::getParam<double>("rtslam/MIN_SCORE", 0.70)),
   hi_match_th(vk::getParam<double>("rtslam/HI_MATCH_TH", 0.85)),
   hi_limit(vk::getParam<double>("rtslam/HI_LIMIT", 100)),
   partial_position(vk::getParam<double>("rtslam/PARTIAL_POSITION", 0.25))
{}

ConfigEstimation& ConfigEstimation::getInstance()
{
  static ConfigEstimation instance; // Instantiated on first use and guaranteed to be destroyed
  return instance;
}

} // namespace rtslamros

