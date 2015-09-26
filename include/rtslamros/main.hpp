/**
 * \file main.hpp
 *
 * Version from main.hpp which uses MTI and Camera sensors from ros
 * topics. Based on jafar/rtslam/main.hpp, as from it was in
 * 14/05/2014.
 *
 * \author Ellon P. Mendes <emendes@laas.fr>
 * \date 14/05/2014
 *
 */


/** ############################################################################
 * #############################################################################
 * features enable/disable
 * ###########################################################################*/

/*
 * STATUS: working fine, use it
 * Ransac ensures that we use correct observations for a few first updates,
 * allowing bad observations to be more easily detected by gating
 * You can disable it by setting N_UPDATES_RANSAC to 0 in config file
 */
#define RANSAC_FIRST 1

/*
 * STATUS: working fine, use it
 * This allows to use Dala "atrv" robot model instead of camera (default) model in the Gdhe view display
 */
#define ATRV 0

/*
 * STATUS: working fine, use it
 * This allows to have 0% cpu used for waiting/idle
 */
//#define EVENT_BASED_RAW 1 // always enabled now

/*
 * STATUS: in progress, do not use for now
 * This allows to track landmarks longer by updating the reference patch when
 * the landmark is not detected anymore and the point of view has changed
 * significantly enough
 * The problem is that the correlation is not robust enough in a matching
 * (opposed to tracking) context, and it can provoke matching errors with a
 * progressive appearance drift.
 * Also decreasing perfs by 10%, probably because we save a view at each obs,
 * or maybe it just because of the different random process
 */
//#define MULTIVIEW_DESCRIPTOR 1 // moved in config file

/*
 * STATUS: in progress, do not use for now
 * This allows to ignore some landmarks when we have some experience telling
 * us that we can't observe this landmarks from here (masking), in order to
 * save some time, and to allow creation of other observations in the
 * neighborhood to keep localizing with enough landmarks.
 * The problem is that sometimes it creates too many landmarks in the same area
 * (significantly slowing down slam), and sometimes doesn't create enough of
 * them when it is necessary.
 */
#define VISIBILITY_MAP 0


/*
 * STATUS: in progress, do not use for now
 * Only update if expectation uncertainty is significant wrt measurement uncertainty.
 *
 * Large updates are causing inconsistency because of linearization errors,
 * but too numerous updates are also causing inconsistency,
 * so we should avoid to do not significant updates.
 * An update is not significant if there are large odds that it is
 * only measurement noise and that there is not much information.
 *
 * When the camera is not moving at all, the landmarks are converging anyway
 * quite fast because of this, at very unconsistent positions of course,
 * so that when the camera moves it cannot recover them.
 *
 * Some work needs to be done yet to prevent search ellipses from growing
 * too much and integrate it better with the whole management, but this was
 * for first evaluation purpose.
 *
 * Unfortunately it doesn't seem to improve much the situation, even if
 * it is still working correctly with less computations.
 * The feature is disabled for now.
 */
#define RELEVANCE_TEST 0

/*
 * STATUS: seems to improve things, needs more testing but you can try it
 * Only update P if expectation uncertainty is significant wrt measurement uncertainty.
 *
 * This is similar to RELEVANCE_TEST except that we always update mean, and
 * update covariance only if innovation is relevant wrt measurement noise,
 * and it is more stable than RELEVANCE_TEST.
 *
 * Needs testing to see if it is stable enough and how to tune the relevance
 * threshold.
 */
#define RELEVANCE_TEST_P 0


/*
 * STATUS: in progress, do not use for now
 * This uses HDseg powered Segment based slam instead of the usual point based slam.
 * 0 use points
 * 1 use segments
 * 2 use both sgments and points
 */
#define SEGMENT_BASED 0
#if SEGMENT_BASED>1
	#define SEGMENT_NOISE_FACTOR 5
#else
	#define SEGMENT_NOISE_FACTOR 1
#endif

#if SEGMENT_BASED
	#ifndef HAVE_MODULE_DSEG
	#error "dseg module is required for segment based slam"
	#endif
#endif


/*
 * STATUS: seems to work ok, needs a bit more testing but you can try it
 * This option will allocate time to data managers to make them stop
 * updating observations when there is no time anymore, in order to avoid
 * missing frames.
 */

#define REAL_TIME_LIVE_RUN 0


/*
 * STATUS: does not work at all, do not use
 * This option stores a small sparse history of past positions of the robot
 * in the filter's state, but it does not change anything to the estimation
 * of the current position.
 */
#define STATE_HISTORY 0


/** ############################################################################
 * #############################################################################
 * Includes
 * ###########################################################################*/

#include <iostream>
#include <boost/shared_ptr.hpp>
//#include <boost/filesystem/operations.hpp>
#include <boost/filesystem.hpp>
#include <time.h>
#include <signal.h>
#include <map>
#include <getopt.h>
#include "kernel/keyValueFile.hpp"

// jafar debug include
#include "kernel/jafarDebug.hpp"
#include "kernel/timingTools.hpp"
#include "kernel/dataLog.hpp"
#include "kernel/threads.hpp"
#include "jmath/random.hpp"
#include "jmath/matlab.hpp"
#include "jmath/ublasExtra.hpp"
#include "jmath/angle.hpp"

#include "rtslam/rtSlam.hpp"
#include "rtslam/rawProcessors.hpp"
#include "rtslam/rawSegProcessors.hpp"
#include "rtslam/robotOdometry.hpp"
#include "rtslam/robotConstantVelocity.hpp"
#include "rtslam/robotConstantMotionModel.hpp"
#include "rtslam/robotInertial.hpp"
#include "rtslam/sensorPinhole.hpp"
#include "rtslam/sensorAbsloc.hpp"
#include "rtslam/landmarkAnchoredHomogeneousPoint.hpp"
#include "rtslam/landmarkAnchoredHomogeneousPointsLine.hpp"
//#include "rtslam/landmarkEuclideanPoint.hpp"
#include "rtslam/observationFactory.hpp"
#include "rtslam/observationMakers.hpp"
#include "rtslam/activeSearch.hpp"
#include "rtslam/activeSegmentSearch.hpp"
#include "rtslam/featureAbstract.hpp"
#include "rtslam/rawImage.hpp"
#include "rtslam/descriptorImagePoint.hpp"
#include "rtslam/descriptorSeg.hpp"
#include "rtslam/dataManagerOnePointRansac.hpp"
#include "rtslam/sensorManager.hpp"
#include "rtslam/historyManager.hpp"

#include "rtslam/hardwareSensorCameraFirewire.hpp"
#include "rtslam/hardwareSensorCameraUeye.hpp"
#include "rtslam/hardwareSensorMti.hpp"
#include "rtslam/hardwareSensorGpsGenom.hpp"
#include "rtslam/hardwareSensorMocap.hpp"
#include "rtslam/hardwareEstimatorOdo.hpp"
#include "rtslam/hardwareSensorExternalLoc.hpp"
#include "rtslam/hardwareSensorOdomRmp400Genom.hpp"

#include "rtslam/display_qt.hpp"
#include "rtslam/display_gdhe.hpp"

#include "rtslam/simuRawProcessors.hpp"
#include "rtslam/hardwareSensorAdhocSimulator.hpp"
#include "rtslam/hardwareSensorInertialAdhocSimulator.hpp"
#include "rtslam/exporterSocket.hpp"

#include <ros/ros.h>
#include <rtslamros/hardwareSensorMtiRos.hpp>
#include <rtslamros/hardwareSensorCameraRos.hpp>

/** ############################################################################
 * #############################################################################
 * types variables functions
 * ###########################################################################*/

using namespace jblas;
using namespace jafar;
using namespace jafar::jmath;
using namespace jafar::jmath::ublasExtra;
using namespace jafar::rtslam;
using namespace boost;


typedef ImagePointObservationMaker<ObservationPinHoleEuclideanPoint, SensorPinhole, LandmarkEuclideanPoint,
	 AppearanceImagePoint, SensorAbstract::PINHOLE, LandmarkAbstract::PNT_EUC> PinholeEucpObservationMaker;
typedef ImagePointObservationMaker<ObservationPinHoleEuclideanPoint, SensorPinhole, LandmarkEuclideanPoint,
	 simu::AppearanceSimu, SensorAbstract::PINHOLE, LandmarkAbstract::PNT_EUC> PinholeEucpSimuObservationMaker;
typedef ImagePointObservationMaker<ObservationPinHoleAnchoredHomogeneousPoint, SensorPinhole, LandmarkAnchoredHomogeneousPoint,
	AppearanceImagePoint, SensorAbstract::PINHOLE, LandmarkAbstract::PNT_AH> PinholeAhpObservationMaker;
typedef ImagePointObservationMaker<ObservationPinHoleAnchoredHomogeneousPoint, SensorPinhole, LandmarkAnchoredHomogeneousPoint,
	simu::AppearanceSimu, SensorAbstract::PINHOLE, LandmarkAbstract::PNT_AH> PinholeAhpSimuObservationMaker;

typedef DataManagerOnePointRansac<RawImage, SensorPinhole, FeatureImagePoint, image::ConvexRoi, ActiveSearchGrid, ImagePointHarrisDetector, ImagePointZnccMatcher> DataManager_ImagePoint_Ransac;
typedef DataManagerOnePointRansac<simu::RawSimu, SensorPinhole, simu::FeatureSimu, image::ConvexRoi, ActiveSearchGrid, simu::DetectorSimu<image::ConvexRoi>, simu::MatcherSimu<image::ConvexRoi> > DataManager_ImagePoint_Ransac_Simu;

#if SEGMENT_BASED
typedef SegmentObservationMaker<ObservationPinHoleAnchoredHomogeneousPointsLine, SensorPinhole, LandmarkAnchoredHomogeneousPointsLine,
   AppearanceImageSegment, SensorAbstract::PINHOLE, LandmarkAbstract::LINE_AHPL> PinholeAhplObservationMaker;
typedef SegmentObservationMaker<ObservationPinHoleAnchoredHomogeneousPointsLine, SensorPinhole, LandmarkAnchoredHomogeneousPointsLine,
	simu::AppearanceSimu, SensorAbstract::PINHOLE, LandmarkAbstract::LINE_AHPL> PinholeAhplSimuObservationMaker;

typedef DataManagerOnePointRansac<RawImage, SensorPinhole, FeatureImageSegment, image::ConvexRoi, ActiveSegmentSearchGrid, HDsegDetector, DsegMatcher> DataManager_ImageSeg_Test;
typedef DataManagerOnePointRansac<simu::RawSimu, SensorPinhole, simu::FeatureSimu, image::ConvexRoi, ActiveSegmentSearchGrid, simu::DetectorSimu<image::ConvexRoi>, simu::MatcherSimu<image::ConvexRoi> > DataManager_Segment_Ransac_Simu;
#endif

hardware::Mode mode = hardware::mOnline;
time_t rseed;


/** ############################################################################
 * #############################################################################
 * program parameters
 * ###########################################################################*/

enum { iDispQt = 0, iDispGdhe, iRenderAll, iReplay, iDump, iRandSeed, iPause, iVerbose, iMap, iRobot, iCamera, iTrigger, iGps, iOdom, iExtloc, iSimu, iExport, iCamsFilter, nIntOpts };
int intOpts[nIntOpts] = {0};
const int nFirstIntOpt = 0, nLastIntOpt = nIntOpts-1;

enum { fFreq = 0, fShutter, fHeading, fNoisySensors, nFloatOpts };
double floatOpts[nFloatOpts] = {0.0};
const int nFirstFloatOpt = nIntOpts, nLastFloatOpt = nIntOpts+nFloatOpts-1;

enum { sDataPath = 0, sConfigSetup, sConfigEstimation, sLog, nStrOpts };
std::string strOpts[nStrOpts];
const int nFirstStrOpt = nIntOpts+nFloatOpts, nLastStrOpt = nIntOpts+nFloatOpts+nStrOpts-1;

enum { bHelp = 0, bUsage, nBreakingOpts };
const int nFirstBreakingOpt = nIntOpts+nFloatOpts+nStrOpts, nLastBreakingOpt = nIntOpts+nFloatOpts+nStrOpts+nBreakingOpts-1;

/// !!WARNING!! be careful that options are in the same order above and below

struct option long_options[] = {
	// int options
	{"disp-2d", 2, 0, 0},
	{"disp-3d", 2, 0, 0},
	{"render-all", 2, 0, 0},
	{"replay", 2, 0, 0},
	{"dump", 2, 0, 0},
	{"rand-seed", 2, 0, 0},
	{"pause", 2, 0, 0},
	{"verbose", 2, 0, 0},
	{"map", 2, 0, 0},
	{"robot", 2, 0, 0}, // should be in config file
	{"camera", 2, 0, 0},
	{"trigger", 2, 0, 0}, // should be in config file
	{"gps", 2, 0, 0},
	{"odom", 2, 0, 0},
	{"extloc", 2, 0, 0},
	{"simu", 2, 0, 0},
	{"export", 2, 0, 0},
	{"cams-filter", 2, 0, 0},
	// double options
	{"freq", 2, 0, 0}, // should be in config file
	{"shutter", 2, 0, 0}, // should be in config file
	{"heading", 2, 0, 0},
	{"noisy-sensors", 2, 0, 0},
	// string options
	{"data-path", 1, 0, 0},
	{"config-setup", 1, 0, 0},
	{"config-estimation", 1, 0, 0},
	{"log", 1, 0, 0},
	// breaking options
	{"help",0,0,0},
	{"usage",0,0,0},
};

/** ############################################################################
 * #############################################################################
 * Config data
 * ###########################################################################*/

const int slam_sched = SCHED_RR;
const int slam_priority = 30; // >0 is higher priority (1;99), needs chown root;chmod u+s or started as root
const int display_niceness = 10; // >0 is lower priority (-20;+20)
const int display_period = 100; // ms


class ConfigSetup: public kernel::KeyValueFileSaveLoad
{
 public:
	/// SENSOR
	jblas::vec CAMERA_POSE_CONSTVEL[2]; ///< camera pose in SLAM frame for constant velocity (x,y,z,roll,pitch,yaw) (m,deg). If add std devs, will be filtered.
	jblas::vec CAMERA_POSE_INERTIAL[2]; ///< camera pose in SLAM frame (IMU frame) for inertial (x,y,z,roll,pitch,yaw) (m,deg). If add std devs, will be filtered.
	jblas::vec GPS_POSE; ///< GPS pose in SLAM frame (IMU frame in inertial) (x,y,z,roll,pitch,yaw) (m,deg). If add std devs, will be filtered.
	jblas::vec ROBOT_POSE; ///< real robot pose in SLAM frame (IMU frame in inertial) (for init and export)

	unsigned CAMERA_TYPE[2];      ///< camera type (0 = firewire, 1 = firewire format7, 2 = USB, 3 = UEYE)
	unsigned CAMERA_FORMAT[2];    ///< camera image format (0: GRAY8, 10: RGB24, 20: YUV411, 21: YUV422_UYVY, 22: YUV422_YUYV, 23: YUV422_YYUV, 24: YVU422_VYUY, 25: YVU422_YVYU, 26: YUV444, 30: BAYER_BGGR, 31: BAYER_GRBG, 32: BAYER_RGGB, 33: BAYER_GBRG)
	std::string CAMERA_DEVICE[2]; ///< camera device (firewire ID or device)
	unsigned CAMERA_IMG_WIDTH[2];     ///< image width
	unsigned CAMERA_IMG_HEIGHT[2];    ///< image height
	jblas::vec4 CAMERA_INTRINSIC[2];  ///< intrisic calibration parameters (u0,v0,alphaU,alphaV)
	jblas::vec3 CAMERA_DISTORTION[2]; ///< distortion calibration parameters (r1,r2,r3)
	std::string CAMERA_CALIB[2];      ///< calibration file if need to rectify

	/// SIMU SENSOR
	unsigned CAMERA_IMG_WIDTH_SIMU[2];
	unsigned CAMERA_IMG_HEIGHT_SIMU[2];
	jblas::vec4 CAMERA_INTRINSIC_SIMU[2];
	jblas::vec3 CAMERA_DISTORTION_SIMU[2];

	/// CONSTANT VELOCITY
	double UNCERT_VLIN; ///< initial uncertainty stdev on linear velocity (m/s)
	double UNCERT_VANG; ///< initial uncertainty stdev on angular velocity (rad/s)
	double PERT_VLIN;   ///< perturbation on linear velocity, ie non-constantness (m/s per sqrt(s))
	double PERT_VANG;   ///< perturbation on angular velocity, ie non-constantness (rad/s per sqrt(s))

	/// INERTIAL (also using UNCERT_VLIN)
	std::string MTI_DEVICE;    ///< IMU device
	double ACCELERO_FULLSCALE; ///< full scale of accelerometers (m/s2)  (MTI: 17)
	double ACCELERO_NOISE;     ///< noise stdev of accelerometers (m/s2) (MTI: 0.002*sqrt(30) )
	double GYRO_FULLSCALE;     ///< full scale of gyrometers (rad/s)     (MTI: rad(300) )
	double GYRO_NOISE;         ///< noise stdev of gyrometers (rad/s)    (MTI: rad(0.05)*sqrt(40) )

	double INITIAL_GRAVITY;  ///< initial value of gravity (default value 9.806, m/s2)
	double UNCERT_GRAVITY;   ///< initial gravity uncertainty (% of INITIAL_GRAVITY)
	double UNCERT_ABIAS;     ///< initial accelerometer bias uncertainty (% of ACCELERO_FULLSCALE, m/s2)
	double UNCERT_WBIAS;     ///< initial gyrometer bias uncertainty (% of GYRO_FULLSCALE, rad/s)
	double PERT_AERR;        ///< noise stdev coeff of accelerometers, for testing purpose (% of ACCELERO_NOISE)
	double PERT_WERR;        ///< noise stdev coeff of gyrometers, for testing purpose (% of GYRO_NOISE)
	double PERT_RANWALKACC;  ///< IMU a_bias random walk (m/s2 per sqrt(s))
	double PERT_RANWALKGYRO; ///< IMU w_bias random walk (rad/s per sqrt(s))

	double INITIAL_HEADING;  ///< initial heading of the real robot (0 is east, positive is toward north, rad)
	double UNCERT_HEADING;   ///< initial heading uncertainty of the real robot (rad)
	double UNCERT_ATTITUDE;  ///< initial attitude angles uncertainty (rad)

	double IMU_TIMESTAMP_CORRECTION; ///< correction to add to the IMU timestamp for synchronization (s)
	double GPS_TIMESTAMP_CORRECTION; ///< correction to add to the GPS timestamp for synchronization (s)

	/// Odometry noise variance to distance ratios
	double dxNDR;   ///< Odometry noise in position increment (m per sqrt(m))
	double dvNDR;   ///< Odometry noise in orientation increment (rad per sqrt(m))

	double ODO_TIMESTAMP_CORRECTION;  ///< correction to add to the odometry timestamp for synchronization (s)
	jblas::vec4 ODO_CALIB; ///< multiplicative correction for odometry wheel size (lf, rf, lb, rb)
	double GPS_MAX_CONSIST_SIG; ///< max sigma with guaranteed consistency (reflections can cause inconsistency)

	/// SIMU INERTIAL
	double SIMU_IMU_TIMESTAMP_CORRECTION;
	double SIMU_IMU_FREQ;
	double SIMU_IMU_GRAVITY;
	double SIMU_IMU_GYR_BIAS;
	double SIMU_IMU_GYR_BIAS_NOISESTD;
	double SIMU_IMU_GYR_GAIN;
	double SIMU_IMU_GYR_GAIN_NOISESTD;
	double SIMU_IMU_RANDWALKGYR_FACTOR;
	double SIMU_IMU_ACC_BIAS;
	double SIMU_IMU_ACC_BIAS_NOISESTD;
	double SIMU_IMU_ACC_GAIN;
	double SIMU_IMU_ACC_GAIN_NOISESTD;
	double SIMU_IMU_RANDWALKACC_FACTOR;

 private:
  void processKeyValueFile(jafar::kernel::KeyValueFile& keyValueFile, bool read);
 public:
	virtual void loadKeyValueFile(jafar::kernel::KeyValueFile const& keyValueFile);
	virtual void saveKeyValueFile(jafar::kernel::KeyValueFile& keyValueFile);
} configSetup;



class ConfigEstimation: public kernel::KeyValueFileSaveLoad
{
 public:
	/// MISC
	unsigned CORRECTION_SIZE; ///< number of coefficients for the distortion correction polynomial

	/// FILTER
	unsigned MAP_SIZE; ///< map size in # of states, robot + landmarks
	double PIX_NOISE;  ///< measurement noise of a point
	double PIX_NOISE_SIMUFACTOR; ///< factor of PIX_NOISE used to generate the actual simulation noise

	/// LANDMARKS
	double D_MIN;      ///< inverse depth mean initialization
	double REPARAM_TH; ///< reparametrization threshold

	unsigned GRID_HCELLS; ///< number of horizontal cells of the image grid for landmark density control
	unsigned GRID_VCELLS; ///< number of vertical cells
	unsigned GRID_MARGIN; ///< min margin of a cell that must be in the image when shifting the grid
	unsigned GRID_SEPAR;  ///< min separation between landmarks in the image for creation (margin with the border of the cell where landmarks can be initialized)

	double RELEVANCE_TH;       ///< relevance threshold to make an update (# of sigmas)
	double MAHALANOBIS_TH;     ///< mahalanobis distance for gating (# of sigmas)
	unsigned N_UPDATES_TOTAL;  ///< max number of landmarks to update every frame
	unsigned N_UPDATES_RANSAC; ///< max number of landmarks to update with ransac every frame
	unsigned N_INIT;           ///< maximum number of landmarks to try to initialize every frame
	unsigned N_RECOMP_GAINS;   ///< how many times information gain is recomputed to resort observations in active search
	double RANSAC_LOW_INNOV;   ///< ransac low innovation threshold (pixels)

	unsigned RANSAC_NTRIES;    ///< number of base observation used to initialize a ransac set
	bool MULTIPLE_DEPTH_HYPOS; ///< make multiple depth hypotheses when search ellipses are too big and distortion too strong

	/// RAW PROCESSING
	unsigned HARRIS_CONV_SIZE; ///< harris detector convolution size
	double HARRIS_TH;          ///< harris threshold
	double HARRIS_EDDGE;       ///< harris symmetry factor

	unsigned DESC_SIZE;        ///< descriptor patch size (odd value)
	bool MULTIVIEW_DESCRIPTOR; ///< whether use or not the multiview descriptor
	double DESC_SCALE_STEP;    ///< MultiviewDescriptor: min change of scale (ratio)
	double DESC_ANGLE_STEP;    ///< MultiviewDescriptor: min change of point of view (deg)
	int DESC_PREDICTION_TYPE;  ///< type of prediction from descriptor (0 = none, 1 = affine, 2 = homographic)

	unsigned PATCH_SIZE;       ///< patch size used for matching
	unsigned MAX_SEARCH_SIZE;  ///< if the search area is larger than this # of pixels, we bound it
	unsigned KILL_SEARCH_SIZE; ///< if the search area is larger than this # of pixels, we vote for killing the landmark
	double MATCH_TH;           ///< ZNCC score threshold
	double MIN_SCORE;          ///< min ZNCC score under which we don't finish to compute the value of the score
	double HI_MATCH_TH;        ///< higher ZNCC score threshold for landmarks with high depth uncertainty and high expectation uncertainty
	double HI_LIMIT;           ///< limit in pixels of the expectation uncertainty to use HI_MATCH_TH
	double PARTIAL_POSITION;   ///< position in the patch where we test if we finish the correlation computation

 private:
  void processKeyValueFile(jafar::kernel::KeyValueFile& keyValueFile, bool read);
 public:
	virtual void loadKeyValueFile(jafar::kernel::KeyValueFile const& keyValueFile);
	virtual void saveKeyValueFile(jafar::kernel::KeyValueFile& keyValueFile);
} configEstimation;



/** ############################################################################
 * #############################################################################
 * Slam function
 * ###########################################################################*/

// FIXME remove this when kernel is released
	template<typename T>
	class VariableConditionPlus: public kernel::VariableCondition<T>
	{
		public:
			VariableConditionPlus(const T &val_init): kernel::VariableCondition<T>(val_init) {}
			template<typename Pred, typename duration_type>
			bool timed_wait(Pred pred, duration_type const & rel_time, bool unlock = true)
			{
				boost::unique_lock<boost::mutex> l(kernel::VariableMutex<T>::m);
				while(!pred(kernel::VariableMutex<T>::var))
					if(!kernel::VariableCondition<T>::c.timed_wait(l, rel_time)) return pred(kernel::VariableMutex<T>::var);
				return true;
			}
	};



boost::scoped_ptr<kernel::LoggerTask> loggerTask;
boost::scoped_ptr<kernel::DataLogger> dataLogger;
sensor_manager_ptr_t sensorManager;
boost::shared_ptr<ExporterAbstract> exporter;
world_ptr_t worldPtr;
#ifdef HAVE_MODULE_QDISPLAY
display::ViewerQt *viewerQt = NULL;
#endif
#ifdef HAVE_MODULE_GDHE
display::ViewerGdhe *viewerGdhe = NULL;
#endif
VariableConditionPlus<int> rawdata_condition(0);
kernel::VariableCondition<int> estimatordata_condition(0);
bool ready = false;
boost::shared_ptr<hardware::HardwareSensorMti> trigger;


bool demo_slam_init()
{ JFR_GLOBAL_TRY

	/// INIT ORDER : config, map and map-manager, create robot, sensors and data-managers, init robot

	/// ---------------------------------------------------------------------------
	/// --- INIT CONFIG -----------------------------------------------------------
	/// ---------------------------------------------------------------------------

	ready = false;

	/// preprocess options
	if (strOpts[sLog].size() == 1)
	{
		if (strOpts[sLog][0] == '0') strOpts[sLog] = ""; else
		if (strOpts[sLog][0] == '1') strOpts[sLog] = "rtslam.log";
	}

	if (!(intOpts[iReplay] & 1) && (intOpts[iDump] & 2))
		{ std::cerr << "Warning: dump==2 only valid with replay==1 or 3" << std::endl; intOpts[iDump] &= (~2); }

	if (!(intOpts[iReplay] & 1) && ((intOpts[iDump] & 1) || strOpts[sLog].size() != 0))
	{
		if (!boost::filesystem::exists(strOpts[sDataPath]))
		{
			if (!boost::filesystem::create_directory(strOpts[sDataPath]))
				std::cerr << "Error: failed to create directory data-path \"" << strOpts[sDataPath] << "\"" << std::endl;
		} else
		{
			if (!boost::filesystem::is_directory(strOpts[sDataPath]))
				std::cerr << "Error: data-path \"" << strOpts[sDataPath] << "\" is not a directory" << std::endl;
			else
			{
				//boost::filesystem::remove_all(strOpts[sDataPath]); // too dangerous
				std::ostringstream oss; oss << "cd " << strOpts[sDataPath] << " ; rm -f *.log ; rm -f *.dat ; rm -f *.pgm ; rm -f *.time ;" << std::endl;
				int r = std::system(oss.str().c_str());
				if (!r) std::cerr << "Failed to clean data path with error " << r << std::endl;
			}
		}
	}

	if (intOpts[iReplay] & 1) mode = hardware::mOffline; else
		if (intOpts[iDump] & 1) mode = hardware::mOnlineDump; else
			mode = hardware::mOnline;
	if (strOpts[sConfigSetup] == "#!@")
	{
		if (intOpts[iReplay] & 1)
			strOpts[sConfigSetup] = strOpts[sDataPath] + "/setup.cfg";
		else
			strOpts[sConfigSetup] = "data/setup.cfg";
	}
	if (intOpts[iReplay] & 1) intOpts[iExport] = 0;
	if (strOpts[sConfigSetup][0] == '@' && strOpts[sConfigSetup][1] == '/')
		strOpts[sConfigSetup] = strOpts[sDataPath] + strOpts[sConfigSetup].substr(1);
	if (strOpts[sConfigEstimation][0] == '@' && strOpts[sConfigEstimation][1] == '/')
		strOpts[sConfigEstimation] = strOpts[sDataPath] + strOpts[sConfigEstimation].substr(1);
	if (!(intOpts[iReplay] & 1) && (intOpts[iDump] & 1))
	{
		boost::filesystem::remove(strOpts[sDataPath] + "/setup.cfg");
		boost::filesystem::remove(strOpts[sDataPath] + "/setup.cfg.maybe");
		boost::filesystem::remove(strOpts[sDataPath] + "/estimation.cfg");
		boost::filesystem::remove(strOpts[sDataPath] + "/estimation.cfg.maybe");
		if (intOpts[iReplay] == 2)
		{
			boost::filesystem::copy_file(strOpts[sConfigSetup], strOpts[sDataPath] + "/setup.cfg.maybe"/*, boost::filesystem::copy_option::overwrite_if_exists*/);
			boost::filesystem::copy_file(strOpts[sConfigEstimation], strOpts[sDataPath] + "/estimation.cfg.maybe"/*, boost::filesystem::copy_option::overwrite_if_exists*/);
		}
		else
		{
			boost::filesystem::copy_file(strOpts[sConfigSetup], strOpts[sDataPath] + "/setup.cfg"/*, boost::filesystem::copy_option::overwrite_if_exists*/);
			boost::filesystem::copy_file(strOpts[sConfigEstimation], strOpts[sDataPath] + "/estimation.cfg"/*, boost::filesystem::copy_option::overwrite_if_exists*/);
		}
	}
	if (intOpts[iCamera] < 10 && intOpts[iCamera] > 0) intOpts[iCamera] += 9; // backward compatibility, anyway it does not mean anything with the new convention
	#ifndef HAVE_MODULE_QDISPLAY
	intOpts[iDispQt] = 0;
	#endif
	#ifndef HAVE_MODULE_GDHE
	intOpts[iDispGdhe] = 0;
	#endif

	std::cout << "Loading config files " << strOpts[sConfigSetup] << " and " << strOpts[sConfigEstimation] << std::endl;
	configSetup.load(strOpts[sConfigSetup]);
	configEstimation.load(strOpts[sConfigEstimation]);

	/// deal with the random seed
	rseed = jmath::get_srand();
	if (intOpts[iRandSeed] != 0 && intOpts[iRandSeed] != 1)
		rseed = intOpts[iRandSeed];
	if (!(intOpts[iReplay] & 1) && (intOpts[iDump] & 1)) {
		std::fstream f((strOpts[sDataPath] + std::string("/rseed.log")).c_str(), std::ios_base::out);
		f << rseed << std::endl;
		f.close();
	}
	else if ((intOpts[iReplay] & 1) && intOpts[iRandSeed] == 1) {
		std::fstream f((strOpts[sDataPath] + std::string("/rseed.log")).c_str(), std::ios_base::in);
		f >> rseed;
		f.close();
	}
	intOpts[iRandSeed] = rseed;
	std::cout << "Random seed " << rseed << std::endl;
	rtslam::srand(rseed);

	/// init world
	worldPtr.reset(new WorldAbstract());

	/// init display
	#ifdef HAVE_MODULE_QDISPLAY
	if (intOpts[iDispQt])
	{
		display::ViewerQt *viewerQt = new display::ViewerQt(8, configEstimation.MAHALANOBIS_TH, false, "data/rendered2D_%02d-%06d.png");
		worldPtr->addDisplayViewer(viewerQt, display::ViewerQt::id());
	}
	#endif
	#ifdef HAVE_MODULE_GDHE
	if (intOpts[iDispGdhe])
	{
		#if ATRV
		display::ViewerGdhe *viewerGdhe = new display::ViewerGdhe("atrv", configEstimation.MAHALANOBIS_TH, "localhost");
		#else
		display::ViewerGdhe *viewerGdhe = new display::ViewerGdhe("camera", configEstimation.MAHALANOBIS_TH, "localhost");
		#endif
		boost::filesystem::path ram_path("/mnt/ram");
		if (boost::filesystem::exists(ram_path) && boost::filesystem::is_directory(ram_path))
			viewerGdhe->setConvertTempPath("/mnt/ram");
		worldPtr->addDisplayViewer(viewerGdhe, display::ViewerGdhe::id());
	}
	#endif

	/// init logger
    if (!strOpts[sLog].empty() || intOpts[iDump])
		loggerTask.reset(new kernel::LoggerTask(display_niceness));

	if (!strOpts[sLog].empty())
	{
		dataLogger.reset(new kernel::DataLogger(strOpts[sDataPath] + "/" + strOpts[sLog]));
		dataLogger->setLoggerTask(loggerTask.get());
		dataLogger->writeCurrentDate();
		dataLogger->writeNewLine();

		// write options to log
		std::ostringstream oss;
		for(int i = 0; i < nIntOpts; ++i)
			{ oss << long_options[i+nFirstIntOpt].name << " = " << intOpts[i]; dataLogger->writeComment(oss.str()); oss.str(""); }
		for(int i = 0; i < nFloatOpts; ++i)
			{ oss << long_options[i+nFirstFloatOpt].name << " = " << floatOpts[i]; dataLogger->writeComment(oss.str()); oss.str(""); }
		for(int i = 0; i < nStrOpts; ++i)
			{ oss << long_options[i+nFirstStrOpt].name << " = " << strOpts[i]; dataLogger->writeComment(oss.str()); oss.str(""); }
		dataLogger->writeNewLine();
	}

	/// init verbosity
	switch (intOpts[iVerbose])
	{
		case 0: debug::DebugStream::setLevel("rtslam", debug::DebugStream::Off); break;
		case 1: debug::DebugStream::setLevel("rtslam", debug::DebugStream::Trace); break;
		case 2: debug::DebugStream::setLevel("rtslam", debug::DebugStream::Warning); break;
		case 3: debug::DebugStream::setLevel("rtslam", debug::DebugStream::Debug); break;
		case 4: debug::DebugStream::setLevel("rtslam", debug::DebugStream::VerboseDebug); break;
		default: debug::DebugStream::setLevel("rtslam", debug::DebugStream::VeryVerboseDebug); break;
	}


	/// ---------------------------------------------------------------------------
	/// --- INIT MAPS -------------------------------------------------------------
	/// ---------------------------------------------------------------------------


	// init some params for the cameras
	int camsi = intOpts[iCamera]/10;
	const int ncam = 3;
	int ncams = 0; bool cams[ncam]; for(int i = 0; i < ncam; i++) { cams[i] = camsi&(1<<i); if (cams[i]) ncams++; }
	bool load_calib = ((intOpts[iCamera]%10) > 0);

	int filter_mods[ncam]; for(int i = 0; i < ncam; ++i) filter_mods[i] = 0;
	int filter_div = 1;
	if (intOpts[iCamsFilter] != 100000)
	{
		int p = 1; for(int i = 0; i < ncams; i++) p *= 10;
		filter_div = intOpts[iCamsFilter] / p;
		if (filter_div == 0) { std::cerr << "not enough digits for --cams-filter, expecting " << ncams+1 << std::endl; worldPtr->error = eParam; return false; }
		for(int i = 0; i < ncam; ++i) if (cams[i]) { p /= 10; filter_mods[i] = (intOpts[iCamsFilter] / p) % 10; }
	}

	if (ncams > 1) { configEstimation.GRID_HCELLS--; }

	vec intrinsic[2], distortion[2];
	int img_width[2], img_height[2];
	for(int c = 0; c < ncam; ++c)
	{
		if (!cams[c]) continue;
		if (intOpts[iSimu] != 0)
		{
			img_width[c] = configSetup.CAMERA_IMG_WIDTH_SIMU[c];
			img_height[c] = configSetup.CAMERA_IMG_HEIGHT_SIMU[c];
			intrinsic[c] = configSetup.CAMERA_INTRINSIC_SIMU[c];
			distortion[c] = configSetup.CAMERA_DISTORTION_SIMU[c];
		} else
		{
			img_width[c] = configSetup.CAMERA_IMG_WIDTH[c];
			img_height[c] = configSetup.CAMERA_IMG_HEIGHT[c];
			intrinsic[c] = configSetup.CAMERA_INTRINSIC[c];
			distortion[c] = configSetup.CAMERA_DISTORTION[c];
		}
	}

	// compute min_cell_fov, min over all camera sensors
	double cell_fov, min_cell_fov = 1e10;
	for(int c = 0; c < ncam; ++c)
	{
		if (!cams[c]) continue;
		cell_fov = 2. * atan(img_width[c] / (2. * intrinsic[c](2))) / configEstimation.GRID_HCELLS;
		if (cell_fov < min_cell_fov) min_cell_fov = cell_fov;
		cell_fov = 2. * atan(img_height[c] / (2. * intrinsic[c](3))) / configEstimation.GRID_VCELLS;
		if (cell_fov < min_cell_fov) min_cell_fov = cell_fov;
	}
	min_cell_fov /= 2.; // security factor
	min_cell_fov *= 180./M_PI;

	// create map
	map_ptr_t mapPtr(new MapAbstract(configEstimation.MAP_SIZE));
	mapPtr->linkToParentWorld(worldPtr);

	 // 1b. Create map manager.
	landmark_factory_ptr_t pointLmkFactory;
	landmark_factory_ptr_t segLmkFactory;
#if SEGMENT_BASED
	 segLmkFactory.reset(new LandmarkFactory<LandmarkAnchoredHomogeneousPointsLine, LandmarkAnchoredHomogeneousPointsLine>());
#endif
#if SEGMENT_BASED != 1
	 pointLmkFactory.reset(new LandmarkFactory<LandmarkAnchoredHomogeneousPoint, LandmarkEuclideanPoint>());
#endif
	map_manager_ptr_t mmPoint;
	map_manager_ptr_t mmSeg;
	const double gridDistInit = 0.5;
	const double gridDistFactor = 2.0;
	const int gridNDist = 5;
	const double gridPhiFactor = 1.2;

	switch(intOpts[iMap])
	{
		case 0: { // odometry
			if(pointLmkFactory != NULL)
				mmPoint.reset(new MapManagerOdometry(pointLmkFactory, configEstimation.REPARAM_TH, configEstimation.KILL_SEARCH_SIZE,
					min_cell_fov, gridDistInit, gridDistFactor, gridNDist, gridPhiFactor));
			if(segLmkFactory != NULL)
				mmSeg.reset(new MapManagerOdometry(segLmkFactory, configEstimation.REPARAM_TH, configEstimation.KILL_SEARCH_SIZE,
					min_cell_fov, gridDistInit, gridDistFactor, gridNDist, gridPhiFactor));
			break;
		}
		case 1: { // global
			const int killSearchTh = 20;
			const double killMatchTh = 0.5;
			const double killConsistencyTh = 0.5;
			const double killUncertaintyTh = 0.5;

			if(pointLmkFactory != NULL)
				mmPoint.reset(new MapManagerGlobal(pointLmkFactory, configEstimation.REPARAM_TH, configEstimation.KILL_SEARCH_SIZE,
					killSearchTh, killMatchTh, killConsistencyTh, killUncertaintyTh, min_cell_fov, gridDistInit, gridDistFactor, gridNDist, gridPhiFactor));
			if(segLmkFactory != NULL)
				mmSeg.reset(new MapManagerGlobal(segLmkFactory, configEstimation.REPARAM_TH, configEstimation.KILL_SEARCH_SIZE,
					killSearchTh, killMatchTh, killConsistencyTh, killUncertaintyTh, min_cell_fov, gridDistInit, gridDistFactor, gridNDist, gridPhiFactor));
			break;
		}
		case 2: { // local/multimap
			if(pointLmkFactory != NULL)
				mmPoint.reset(new MapManagerLocal(pointLmkFactory, configEstimation.REPARAM_TH, configEstimation.KILL_SEARCH_SIZE));
			if(segLmkFactory != NULL)
				mmSeg.reset(new MapManagerLocal(segLmkFactory, configEstimation.REPARAM_TH, configEstimation.KILL_SEARCH_SIZE));
			break;
		}
	}
	if(mmPoint != NULL)
		mmPoint->linkToParentMap(mapPtr);
	if(mmSeg != NULL)
		mmSeg->linkToParentMap(mapPtr);

	// simulation environment
	boost::shared_ptr<simu::AdhocSimulator> simulator;
	if (intOpts[iSimu] != 0)
	{
		simulator.reset(new simu::AdhocSimulator());
		#if SEGMENT_BASED
			jblas::vec11 pose;
		#else
			jblas::vec3 pose;
		#endif

		const int maxnpoints = 1000;
		int npoints = 0;
		#if SEGMENT_BASED
			double points[maxnpoints][11];
		#else
			double points[maxnpoints][3];
		#endif

		switch (intOpts[iSimu]/10)
		{
		#if SEGMENT_BASED
			case 1: {
				// 3D cube
				const int npoints_ = 36; npoints = npoints_;
				double tmp[npoints_][11] = {
					{0,0,0,5,-1,-1,1,5,-1, 1,1}, {0,0,0,5,1,1,1,5,1,-1,1}, {0,0,0,5,-1,1,1,5, 1,1,1}, {0,0,0,5,-1,-1,1,5,1,-1,1},
					{0,0,0,3,-1,-1,1,3,-1, 1,1}, {0,0,0,3,1,1,1,3,1,-1,1}, {0,0,0,3,-1,1,1,3, 1,1,1}, {0,0,0,3,-1,-1,1,3,1,-1,1},
					{0,0,0,5,-1,-1,1,3,-1,-1,1}, {0,0,0,5,1,1,1,3,1, 1,1}, {0,0,0,5,-1,1,1,3,-1,1,1}, {0,0,0,5, 1,-1,1,3,1,-1,1},
					{0,0,0,5,-4,-1,1,5,-4, 1,1}, {0,0,0,5,-2,1,1,5,-2,-1,1}, {0,0,0,5,-4,1,1,5,-2,1,1}, {0,0,0,5,-4,-1,1,5,-2,-1,1},
					{0,0,0,3,-4,-1,1,3,-4, 1,1}, {0,0,0,3,-2,1,1,3,-2,-1,1}, {0,0,0,3,-4,1,1,3,-2,1,1}, {0,0,0,3,-4,-1,1,3,-2,-1,1},
					{0,0,0,5,-4,-1,1,3,-4,-1,1}, {0,0,0,5,-2,1,1,3,-2, 1,1}, {0,0,0,5,-4,1,1,3,-4,1,1}, {0,0,0,5,-2,-1,1,3,-2,-1,1},
					{0,0,0,5, 2,-1,1,5, 2, 1,1}, {0,0,0,5,4,1,1,5,4,-1,1}, {0,0,0,5,2,1,1,5, 4,1,1}, {0,0,0,5, 2,-1,1,5,4,-1,1},
					{0,0,0,3, 2,-1,1,3, 2, 1,1}, {0,0,0,3,4,1,1,3,4,-1,1}, {0,0,0,3,2,1,1,3, 4,1,1}, {0,0,0,3, 2,-1,1,3,4,-1,1},
					{0,0,0,5, 2,-1,1,3, 2,-1,1}, {0,0,0,5,4,1,1,3,4, 1,1}, {0,0,0,5,2,1,1,3, 2,1,1}, {0,0,0,5, 4,-1,1,3,4,-1,1}
				};
				memcpy(points, tmp, npoints*11*sizeof(double));
				break;
			}
			case 2: {
				// 2D square
				const int npoints_ = 4; npoints = npoints_;
				double tmp[npoints_][11] = {
					{0,0,0,5,-1,-1,1,5,-1,1,1}, {0,0,0,5,1,1,1,5,1,-1,1}, {0,0,0,5,-1,1,1,5,1,1,1}, {0,0,0,5,-1,-1,1,5,1,-1,1}
				};
				memcpy(points, tmp, npoints*11*sizeof(double));
				break;
			}
			case 3: {
				// almost 2D square
				const int npoints_ = 4; npoints = npoints_;
				double tmp[npoints_][11] = {
					{0,0,0,4,-1,-1,1,4,-1,1,1}, {0,0,0,4,1,1,1,4,1,-1,1}, {0,0,0,5,-1,1,1,5,1,1,1}, {0,0,0,5,-1,-1,1,5,1,-1,1}
				};
				memcpy(points, tmp, npoints*11*sizeof(double));
				break;
			}
			default: npoints = 0;
		#else
			case 1: {
				// 3D regular grid
				const int npoints_ = 3*11*13; npoints = npoints_;
				for(int i = 0, z = -1; z <= 1; ++z) for(int y = -3; y <= 7; ++y) for(int x = -6; x <= 6; ++x, ++i)
					{ points[i][0] = x*1.0; points[i][1] = y*1.0; points[i][2] = z*1.0; }
				break;
			}
			case 2: {
				// 2D square
				const int npoints_ = 5; npoints = npoints_;
				double tmp[npoints_][3] = { {5,-1,-1}, {5,-1,1}, {5,1,1}, {5,1,-1}, {5,0,0} };
				memcpy(points, tmp, npoints*3*sizeof(double));
				break;
			}
			case 3: {
				// almost 2D square
				const int npoints_ = 5; npoints = npoints_;
				double tmp[npoints_][3] = { {5,-1,-1}, {5,-1,1}, {5,1,1}, {5,1,-1}, {4,0,0} };
				memcpy(points, tmp, npoints*3*sizeof(double));
				break;
			}
		case 4: {
			// far 3D regular grid
			const int npoints_ = 3*11*13; npoints = npoints_;
			for(int i = 0, z = -1; z <= 1; ++z) for(int y = -3; y <= 7; ++y) for(int x = -6; x <= 6; ++x, ++i)
				{ points[i][0] = x*1.0+100; points[i][1] = y*10.0; points[i][2] = z*10.0; }
			break;
		}

			default: npoints = 0;
		#endif
		}

		// add landmarks
		for(int i = 0; i < npoints; ++i)
		{
			pose(0) = points[i][0]; pose(1) = points[i][1]; pose(2) = points[i][2];
		#if !SEGMENT_BASED
			simu::Landmark *lmk = new simu::Landmark(LandmarkAbstract::POINT, pose);
		#else
			pose(3) = points[i][3]; pose(4) = points[i][4]; pose(5) = points[i][5];
			pose(6) = points[i][6]; pose(7) = points[i][7]; pose(8) = points[i][8];
			pose(9) = points[i][9]; pose(10) = points[i][10];
			simu::Landmark *lmk = new simu::Landmark(LandmarkAbstract::LINE, pose);
		#endif
			simulator->addLandmark(lmk);
		}
	}


	/// ---------------------------------------------------------------------------
	/// --- CREATE ROBOTS ---------------------------------------------------------
	/// ---------------------------------------------------------------------------

	double trigger_construction_date = -1;
	robot_ptr_t robPtr1;
	switch (intOpts[iRobot])
	{
		case 0: {
			boost::shared_ptr<RobotConstantVelocity> robPtr(new RobotConstantVelocity(mapPtr));
			robPtr->setVelocityStd(configSetup.UNCERT_VLIN, configSetup.UNCERT_VANG); robPtr1 = robPtr; break; }
		case 10: case 100: {
			boost::shared_ptr<RobotConstantMotionModel<0,0> > robPtr(new RobotConstantMotionModel<0,0>(mapPtr));
			robPtr->setVelocityStd(configSetup.UNCERT_VLIN, configSetup.UNCERT_VANG); robPtr1 = robPtr; break; }
//		case 0:
		case 11: case 111: {
			boost::shared_ptr<RobotConstantMotionModel<1,1> > robPtr(new RobotConstantMotionModel<1,1>(mapPtr));
			robPtr->setVelocityStd(configSetup.UNCERT_VLIN, configSetup.UNCERT_VANG); robPtr1 = robPtr; break; }
		case 12: case 122: {
			boost::shared_ptr<RobotConstantMotionModel<2,2> > robPtr(new RobotConstantMotionModel<2,2>(mapPtr));
			robPtr->setVelocityStd(configSetup.UNCERT_VLIN, configSetup.UNCERT_VANG); robPtr1 = robPtr; break; }
		case 121: {
			boost::shared_ptr<RobotConstantMotionModel<2,1> > robPtr(new RobotConstantMotionModel<2,1>(mapPtr));
			robPtr->setVelocityStd(configSetup.UNCERT_VLIN, configSetup.UNCERT_VANG); robPtr1 = robPtr; break; }
		case 13: case 133: {
			boost::shared_ptr<RobotConstantMotionModel<3,3> > robPtr(new RobotConstantMotionModel<3,3>(mapPtr));
			robPtr->setVelocityStd(configSetup.UNCERT_VLIN, configSetup.UNCERT_VANG); robPtr1 = robPtr; break; }
	}

	if (intOpts[iRobot] == 0 || intOpts[iRobot] >= 10) // constant model
	{
		double _v[6] = {
				configSetup.PERT_VLIN, configSetup.PERT_VLIN, configSetup.PERT_VLIN,
				configSetup.PERT_VANG, configSetup.PERT_VANG, configSetup.PERT_VANG };
		vec pertStd = createVector<6>(_v);
		robPtr1->perturbation.set_std_continuous(pertStd);

		if (intOpts[iTrigger] != 0)
		{
			// just to initialize the MTI as an external trigger controlling shutter time
			trigger.reset(new hardware::HardwareSensorMti(
				NULL, configSetup.MTI_DEVICE, intOpts[iTrigger], floatOpts[fFreq], floatOpts[fShutter], 1, mode, strOpts[sDataPath], loggerTask.get()));
			trigger_construction_date = kernel::Clock::getTime();
			floatOpts[fFreq] = trigger->getFreq();
		}
	}
	else
	if (intOpts[iRobot] == 1) // inertial
	{
		robinertial_ptr_t robPtr1_(new RobotInertial(mapPtr));
		robPtr1_->setInitialStd(
			configSetup.UNCERT_VLIN,
			configSetup.UNCERT_ABIAS*configSetup.ACCELERO_FULLSCALE,
			configSetup.UNCERT_WBIAS*configSetup.GYRO_FULLSCALE,
			configSetup.UNCERT_GRAVITY*configSetup.INITIAL_GRAVITY);
		robPtr1_->setInitialParams(configSetup.INITIAL_GRAVITY);

		double aerr = configSetup.PERT_AERR * configSetup.ACCELERO_NOISE;
		double werr = configSetup.PERT_WERR * configSetup.GYRO_NOISE;
		double _v[12] = {
				aerr, aerr, aerr, werr, werr, werr,
				configSetup.PERT_RANWALKACC, configSetup.PERT_RANWALKACC, configSetup.PERT_RANWALKACC,
				configSetup.PERT_RANWALKGYRO, configSetup.PERT_RANWALKGYRO, configSetup.PERT_RANWALKGYRO};
		vec pertStd = createVector<12>(_v);
		#if ESTIMATE_BIASES
		robPtr1_->perturbation.set_std_continuous(pertStd);
		#else
		robPtr1_->perturbation.set_std_continuous(ublas::subrange(pertStd,0,6));
		#endif

		hardware::hardware_sensorprop_ptr_t hardEst1;
		if (intOpts[iSimu] != 0)
		{
/*			boost::shared_ptr<hardware::HardwareEstimatorInertialAdhocSimulator> hardEst1_(
				new hardware::HardwareEstimatorInertialAdhocSimulator(configSetup.SIMU_IMU_FREQ, 50, simulator, robPtr1_->id()));
			hardEst1_->setSyncConfig(configSetup.SIMU_IMU_TIMESTAMP_CORRECTION);

			hardEst1_->setErrors(configSetup.SIMU_IMU_GRAVITY,
				configSetup.SIMU_IMU_GYR_BIAS, configSetup.SIMU_IMU_GYR_BIAS_NOISESTD,
				configSetup.SIMU_IMU_GYR_GAIN, configSetup.SIMU_IMU_GYR_GAIN_NOISESTD,
				configSetup.SIMU_IMU_RANDWALKGYR_FACTOR * configSetup.PERT_RANWALKGYRO,
				configSetup.SIMU_IMU_ACC_BIAS, configSetup.SIMU_IMU_ACC_BIAS_NOISESTD,
				configSetup.SIMU_IMU_ACC_GAIN, configSetup.SIMU_IMU_ACC_GAIN_NOISESTD,
				configSetup.SIMU_IMU_RANDWALKACC_FACTOR * configSetup.PERT_RANWALKACC);

			hardEst1 = hardEst1_;
*/		} else
		{
		  /** CHANGE_HERE_TO_ROS
		   */
			boost::shared_ptr<rtslamros::hardware::HardwareSensorMtiRos> hardEst1_(new rtslamros::hardware::HardwareSensorMtiRos(
				&estimatordata_condition, intOpts[iTrigger], floatOpts[fFreq], floatOpts[fShutter], 1024, mode, strOpts[sDataPath], loggerTask.get()));
			trigger_construction_date = kernel::Clock::getTime();
			if (intOpts[iTrigger] != 0) floatOpts[fFreq] = hardEst1_->getFreq();
			hardEst1_->setSyncConfig(configSetup.IMU_TIMESTAMP_CORRECTION);
			//hardEst1_->setUseForInit(true);
			//hardEst1_->setNeedInit(true);
			//hardEst1_->start();
			hardEst1 = hardEst1_;
		}
		robPtr1_->setHardwareEstimator(hardEst1);

		robPtr1 = robPtr1_;
	} else
	if (intOpts[iRobot] == 2) // odometry
	{
/*		robodo_ptr_t robPtr1_(new RobotOdometry(mapPtr));
		std::cout<<"configSetup.dxNDR "<<configSetup.dxNDR<<std::endl;
		std::cout<<"configSetup.dvNDR "<<configSetup.dvNDR<<std::endl;
		double _v[6] = {configSetup.dxNDR, configSetup.dxNDR, configSetup.dxNDR,
						configSetup.dvNDR, configSetup.dvNDR, configSetup.dvNDR};
		vec pertStd = createVector<6>(_v);
		robPtr1_->perturbation.set_std_continuous(pertStd);

		hardware::hardware_estimator_ptr_t hardEst2;
		boost::shared_ptr<hardware::HardwareEstimatorOdo> hardEst2_(new hardware::HardwareEstimatorOdo(
				intOpts[iTrigger], floatOpts[fFreq], floatOpts[fShutter], 1024, mode, strOpts[sDataPath]));
		if (intOpts[iTrigger] != 0) floatOpts[fFreq] = hardEst2_->getFreq();
		hardEst2_->setSyncConfig(configSetup.POS_TIMESTAMP_CORRECTION);
		hardEst2 = hardEst2_;
		robPtr1_->setHardwareEstimator(hardEst2);
		robPtr1 = robPtr1_;
		*/
	}

	robPtr1->linkToParentMap(mapPtr);
	robPtr1->setRobotPose(ublas::subrange(configSetup.ROBOT_POSE,0,6), true);
	double heading = (floatOpts[fHeading] < 1e4 ? floatOpts[fHeading] : configSetup.INITIAL_HEADING);
	robPtr1->setOrientationStd(0,0,heading,
		configSetup.UNCERT_ATTITUDE,configSetup.UNCERT_ATTITUDE,configSetup.UNCERT_HEADING, false);
	if (dataLogger) dataLogger->addLoggable(*robPtr1.get());

  // history manager
  #if STATE_HISTORY
  history_manager_ptr_t hm(new HistoryManagerSparse(mapPtr, robPtr1->pose.ia(), 1.0, 5.0, 3));
  //history_manager_ptr_t hm(new HistoryManagerSparse(mapPtr, robPtr1->state.ia(), 1.0, 5.0, 3));
  robPtr1->setHistoryManager(hm);
  #endif


	if (intOpts[iSimu] != 0)
	{
		simu::Robot *rob = new simu::Robot(robPtr1->id(), 6);
		if (dataLogger) dataLogger->addLoggable(*rob);

		switch (intOpts[iSimu]%10)
		{
			// horiz loop, no rotation
			case 1: {
				double VEL = 0.5;
				rob->addWaypoint(0,0,0, 0,0,0, 0,0,0, 0,0,0);
				rob->addWaypoint(1,0,0, 0,0,0, VEL,0,0, 0,0,0);
				rob->addWaypoint(3,2,0, 0,0,0, 0,VEL,0, 0,0,0);
				rob->addWaypoint(1,4,0, 0,0,0, -VEL,0,0, 0,0,0);
				rob->addWaypoint(-1,4,0, 0,0,0, -VEL,0,0, 0,0,0);
				rob->addWaypoint(-3,2,0, 0,0,0, 0,-VEL,0, 0,0,0);
				rob->addWaypoint(-1,0,0, 0,0,0, VEL,0,0, 0,0,0);
				rob->addWaypoint(0,0,0, 0,0,0, 0,0,0, 0,0,0);
				break;
			}
			// two coplanar circles
			case 2: {
				double VEL = 0.5;
				rob->addWaypoint(0 ,+1,0 , 0,0,0, 0,0   ,+VEL, 0,0,0);
				rob->addWaypoint(0 ,0 ,+1, 0,0,0, 0,-VEL,0   , 0,0,0);
				rob->addWaypoint(0 ,-1,0 , 0,0,0, 0,0   ,-VEL, 0,0,0);
				rob->addWaypoint(0 ,0 ,-1, 0,0,0, 0,+VEL,0   , 0,0,0);

				rob->addWaypoint(0 ,+1,0 , 0,0,0, 0,0   ,+VEL, 0,0,0);
				rob->addWaypoint(0 ,0 ,+1, 0,0,0, 0,-VEL,0   , 0,0,0);
				rob->addWaypoint(0 ,-1,0 , 0,0,0, 0,0   ,-VEL, 0,0,0);
				rob->addWaypoint(0 ,0 ,-1, 0,0,0, 0,+VEL,0   , 0,0,0);

				rob->addWaypoint(0 ,+1,0 , 0,0,0, 0,0   ,+VEL, 0,0,0);
				break;
			}

			// two non-coplanar circles at constant velocity
			case 3: {
				double VEL = 0.5;
				rob->addWaypoint(0   ,+1,0 , 0,0,0, 0,0   ,+VEL, 0,0,0);
				rob->addWaypoint(0.25,0 ,+1, 0,0,0, VEL/4,-VEL,0   , 0,0,0);
				rob->addWaypoint(0.5 ,-1,0 , 0,0,0, 0,0   ,-VEL, 0,0,0);
				rob->addWaypoint(0.25,0 ,-1, 0,0,0, -VEL/4,+VEL,0   , 0,0,0);

				rob->addWaypoint(0    ,+1,0 , 0,0,0, 0,0   ,+VEL, 0,0,0);
				rob->addWaypoint(-0.25,0 ,+1, 0,0,0, -VEL/4,-VEL,0   , 0,0,0);
				rob->addWaypoint(-0.5 ,-1,0 , 0,0,0, 0,0   ,-VEL, 0,0,0);
				rob->addWaypoint(-0.25,0 ,-1, 0,0,0, VEL/4,+VEL,0   , 0,0,0);

				rob->addWaypoint(0 ,+1,0 , 0,0,0, 0,0   ,+VEL, 0,0,0);
				break;
			}

			// two non-coplanar circles with start and stop from/to null speed
			case 4: {
				double VEL = 0.5;
				rob->addWaypoint(0   ,+1,0 , 0,0,0, 0,0   ,0, 0,0,0);
				rob->addWaypoint(0   ,+1,0.1 , 0,0,0, 0,0   ,+VEL/2, 0,0,0);
				rob->addWaypoint(0   ,+1,0.5 , 0,0,0, 0,0   ,+VEL/2, 0,0,0);
				rob->addWaypoint(0.25,0 ,+1, 0,0,0, VEL/4,-VEL,0   , 0,0,0);
				rob->addWaypoint(0.5 ,-1,0 , 0,0,0, 0,0   ,-VEL, 0,0,0);
				rob->addWaypoint(0.25,0 ,-1, 0,0,0, -VEL/4,+VEL,0   , 0,0,0);

				rob->addWaypoint(0    ,+1,0 , 0,0,0, 0,0   ,+VEL, 0,0,0);
				rob->addWaypoint(-0.25,0 ,+1, 0,0,0, -VEL/4,-VEL,0   , 0,0,0);
				rob->addWaypoint(-0.5 ,-1,0 , 0,0,0, 0,0   ,-VEL, 0,0,0);
				rob->addWaypoint(-0.25,0 ,-1, 0,0,0, VEL/4,+VEL,0   , 0,0,0);

				rob->addWaypoint(0 ,+1,-0.5 , 0,0,0, 0,0   ,+VEL/2, 0,0,0);
				rob->addWaypoint(0 ,+1,-0.1 , 0,0,0, 0,0   ,+VEL/2, 0,0,0);
				rob->addWaypoint(0 ,+1,0 , 0,0,0, 0,0   ,0, 0,0,0);
				break;
			}

			// horiz loop with rotation (always goes forward)
			case 5: {
				double VEL = 0.5;
				rob->addWaypoint(0,0,0, 0,0,0, VEL/5,0,0, 0,0,0);
				rob->addWaypoint(1,0,0, 0,0,0, VEL,0,0, 0,0,0);
				rob->addWaypoint(3,2,0, 1*M_PI/2,0,0, 0,VEL,0, 100,0,0);
				rob->addWaypoint(1,4,0, 2*M_PI/2,0,0, -VEL,0,0, 0,0,0);
				rob->addWaypoint(-1,4,0, 2*M_PI/2,0,0, -VEL,0,0, 0,0,0);
				rob->addWaypoint(-3,2,0, 3*M_PI/2,0,0, 0,-VEL,0, 100,0,0);
				rob->addWaypoint(-1,0,0, 4*M_PI/2,0,0, VEL,0,0, 0,0,0);
				rob->addWaypoint(0,0,0, 4*M_PI/2,0,0, 0,0,0, 0,0,0);
				break;
			}

			// straight line
			case 6: {
				double VEL = 0.5;
				rob->addWaypoint(0,0,0, 0,0,0, 0,0,0, 0,0,0);
				rob->addWaypoint(1,0,0, 0,0,0, VEL,0,0, 0,0,0);
				rob->addWaypoint(20,0,0, 0,0,0, VEL,0,0, 0,0,0);
				break;
			}
		}

		simulator->addRobot(rob);

	}

//robPtr1->setPoseStd(0, 0, 0, 0, 0, 0, 20, 20, 20, 10, 10, 10, true);

	/// ---------------------------------------------------------------------------
	/// --- INIT SENSORS ----------------------------------------------------------
	/// ---------------------------------------------------------------------------


	/// pin-hole parameters in BOOST format
	boost::shared_ptr<ObservationFactory> obsFact(new ObservationFactory());
#if SEGMENT_BASED
	if (intOpts[iSimu] != 0)
	{
		obsFact->addMaker(boost::shared_ptr<ObservationMakerAbstract>(new PinholeAhplSimuObservationMaker(
			configEstimation.REPARAM_TH, configEstimation.KILL_SEARCH_SIZE, 30, 0.5, 0.5, configEstimation.D_MIN, configEstimation.PATCH_SIZE)));
	} else
	{
		obsFact->addMaker(boost::shared_ptr<ObservationMakerAbstract>(new PinholeAhplObservationMaker(
			configEstimation.REPARAM_TH, configEstimation.KILL_SEARCH_SIZE, 30, 0.5, 0.5, configEstimation.D_MIN, configEstimation.PATCH_SIZE)));
	}
#endif
#if SEGMENT_BASED != 1
	if (intOpts[iSimu] != 0)
	{
		obsFact->addMaker(boost::shared_ptr<ObservationMakerAbstract>(new PinholeEucpSimuObservationMaker(
		  configEstimation.D_MIN, configEstimation.PATCH_SIZE)));
		obsFact->addMaker(boost::shared_ptr<ObservationMakerAbstract>(new PinholeAhpSimuObservationMaker(
		  configEstimation.D_MIN, configEstimation.PATCH_SIZE)));
	} else
	{
		obsFact->addMaker(boost::shared_ptr<ObservationMakerAbstract>(new PinholeEucpObservationMaker(
		  configEstimation.D_MIN, configEstimation.PATCH_SIZE)));
		obsFact->addMaker(boost::shared_ptr<ObservationMakerAbstract>(new PinholeAhpObservationMaker(
		  configEstimation.D_MIN, configEstimation.PATCH_SIZE)));
	}
#endif

	int ncam_built = 0;
	pinhole_ptr_t cams_built[ncam];
	bool initialized_cameras ;
	const int ntrya = 4;
	for(int itrya = 0; itrya < ntrya; ++itrya) { initialized_cameras = true;
	for(int c = 0; c < ncam && initialized_cameras; ++c)
	{
		if (!cams[c]) continue;
		pinhole_ptr_t senPtr11;
		if (c < ncam_built) senPtr11 = cams_built[c]; else {
		// a. Create camera
		jblas::vec camera_pose = (intOpts[iRobot] == 1 ? configSetup.CAMERA_POSE_INERTIAL[c] : configSetup.CAMERA_POSE_CONSTVEL[c]);
		int cp_size = camera_pose.size();
		if (cp_size != 6 && cp_size != 12) { std::cerr << "Camera pose must have size 6 or 12 (with uncertainties), not " << cp_size << std::endl; worldPtr->error = eConfig; return false; }
		senPtr11 = pinhole_ptr_t(new SensorPinhole(robPtr1, (cp_size == 6 ? MapObject::UNFILTERED : MapObject::FILTERED)));
		senPtr11->linkToParentRobot(robPtr1);
		if (ncams == 1) senPtr11->name("cam"); else { std::ostringstream oss; oss << "cam" << c; senPtr11->name(oss.str()); }
		if (cp_size == 6)
		{
			senPtr11->setPose(camera_pose(0), camera_pose(1), camera_pose(2), camera_pose(3), camera_pose(4), camera_pose(5)); // x,y,z,roll,pitch,yaw
		} else
		{
			senPtr11->setPoseStd(camera_pose(0), camera_pose(1), camera_pose(2), camera_pose(3), camera_pose(4), camera_pose(5),
				camera_pose(6), camera_pose(7), camera_pose(8), camera_pose(9), camera_pose(10), camera_pose(11)); // x,y,z,roll,pitch,yaw + std_dev
		}
		//senPtr11->pose.x(quaternion::originFrame());
		senPtr11->params.setIntrinsicCalibration(img_width[c], img_height[c], intrinsic[c], distortion[c], configEstimation.CORRECTION_SIZE);
		//JFR_DEBUG("Correction params: " << senPtr11->params.correction);
		senPtr11->params.setMiscellaneous(configEstimation.PIX_NOISE, configEstimation.D_MIN);

		if (dataLogger) dataLogger->addLoggable(*senPtr11.get());

		if (intOpts[iSimu] != 0)
		{
			jblas::vec6 pose;
			subrange(pose, 0, 3) = subrange(senPtr11->pose.x(), 0, 3);
			subrange(pose, 3, 6) = quaternion::q2e(subrange(senPtr11->pose.x(), 3, 7));
			std::swap(pose(3), pose(5)); // FIXME-EULER-CONVENTION
			simu::Sensor *sen = new simu::Sensor(senPtr11->id(), pose, senPtr11);
			simulator->addSensor(robPtr1->id(), sen);
			simulator->addObservationModel(robPtr1->id(), senPtr11->id(), LandmarkAbstract::POINT, new ObservationModelPinHoleEuclideanPoint(senPtr11));
			#if SEGMENT_BASED
				simulator->addObservationModel(robPtr1->id(), senPtr11->id(), LandmarkAbstract::LINE, new ObservationModelPinHoleAnchoredHomogeneousPointsLine(senPtr11));
			#endif
		} else
		{
			senPtr11->setIntegrationPolicy(false);
			senPtr11->setUseForInit(false);
			senPtr11->setNeedInit(true); // for auto exposure
		}

		// b. Create data manager.
		boost::shared_ptr<ActiveSearchGrid> asGrid(new ActiveSearchGrid(img_width[c], img_height[c], configEstimation.GRID_HCELLS, configEstimation.GRID_VCELLS, configEstimation.GRID_MARGIN, configEstimation.GRID_SEPAR));
		 boost::shared_ptr<ActiveSegmentSearchGrid> assGrid(new ActiveSegmentSearchGrid(img_width[c], img_height[c], configEstimation.GRID_HCELLS, configEstimation.GRID_VCELLS, configEstimation.GRID_MARGIN, configEstimation.GRID_SEPAR));

		#if RANSAC_FIRST
		 int ransac_ntries = configEstimation.RANSAC_NTRIES;
		#else
		int ransac_ntries = 0;
		#endif

		if (intOpts[iSimu] != 0)
		{
			#if SEGMENT_BASED
				boost::shared_ptr<simu::DetectorSimu<image::ConvexRoi> > detector(new simu::DetectorSimu<image::ConvexRoi>(LandmarkAbstract::LINE, 4, configEstimation.PATCH_SIZE, configEstimation.PIX_NOISE, configEstimation.PIX_NOISE*configEstimation.PIX_NOISE_SIMUFACTOR));
				boost::shared_ptr<simu::MatcherSimu<image::ConvexRoi> > matcher(new simu::MatcherSimu<image::ConvexRoi>(LandmarkAbstract::LINE, 4, configEstimation.PATCH_SIZE, configEstimation.MAX_SEARCH_SIZE, configEstimation.RANSAC_LOW_INNOV, configEstimation.MATCH_TH, configEstimation.MAHALANOBIS_TH, configEstimation.RELEVANCE_TH, configEstimation.PIX_NOISE, configEstimation.PIX_NOISE*configEstimation.PIX_NOISE_SIMUFACTOR));

				boost::shared_ptr<DataManager_Segment_Ransac_Simu> dmPt11(new DataManager_Segment_Ransac_Simu(detector, matcher, assGrid, configEstimation.N_UPDATES_TOTAL, configEstimation.N_UPDATES_RANSAC, ransac_ntries, configEstimation.N_INIT, configEstimation.N_RECOMP_GAINS, configEstimation.MULTIPLE_DEPTH_HYPOS, (intOpts[iDump]&2) ? loggerTask.get() : NULL));

				dmPt11->linkToParentSensorSpec(senPtr11);
				dmPt11->linkToParentMapManager(mmPoint);
				dmPt11->setObservationFactory(obsFact);

				hardware::hardware_sensorext_ptr_t hardSen11(new hardware::HardwareSensorAdhocSimulator(&rawdata_condition, floatOpts[fFreq], simulator, robPtr1->id(), senPtr11->id()));
				senPtr11->setHardwareSensor(hardSen11);
			#else
				boost::shared_ptr<simu::DetectorSimu<image::ConvexRoi> > detector(new simu::DetectorSimu<image::ConvexRoi>(LandmarkAbstract::POINT, 2, configEstimation.PATCH_SIZE, configEstimation.PIX_NOISE, configEstimation.PIX_NOISE*configEstimation.PIX_NOISE_SIMUFACTOR));
				boost::shared_ptr<simu::MatcherSimu<image::ConvexRoi> > matcher(new simu::MatcherSimu<image::ConvexRoi>(LandmarkAbstract::POINT, 2, configEstimation.PATCH_SIZE, configEstimation.MAX_SEARCH_SIZE, configEstimation.RANSAC_LOW_INNOV, configEstimation.MATCH_TH, configEstimation.HI_MATCH_TH, configEstimation.HI_LIMIT, configEstimation.MAHALANOBIS_TH, configEstimation.RELEVANCE_TH, configEstimation.PIX_NOISE, configEstimation.PIX_NOISE*configEstimation.PIX_NOISE_SIMUFACTOR));

				boost::shared_ptr<DataManager_ImagePoint_Ransac_Simu> dmPt11(new DataManager_ImagePoint_Ransac_Simu(detector, matcher, asGrid, configEstimation.N_UPDATES_TOTAL, configEstimation.N_UPDATES_RANSAC, ransac_ntries, configEstimation.N_INIT, configEstimation.N_RECOMP_GAINS, configEstimation.MULTIPLE_DEPTH_HYPOS, (intOpts[iDump]&2) ? loggerTask.get() : NULL));

				dmPt11->linkToParentSensorSpec(senPtr11);
				dmPt11->linkToParentMapManager(mmPoint);
				dmPt11->setObservationFactory(obsFact);

				hardware::hardware_sensorext_ptr_t hardSen11(new hardware::HardwareSensorAdhocSimulator(&rawdata_condition, floatOpts[fFreq], simulator, robPtr1->id(), senPtr11->id()));
				senPtr11->setHardwareSensor(hardSen11);
			#endif
		} else
		{
			boost::shared_ptr<DescriptorFactoryAbstract> pointDescFactory;
			boost::shared_ptr<DescriptorFactoryAbstract> segDescFactory;
			#if SEGMENT_BASED
				if (configEstimation.MULTIVIEW_DESCRIPTOR)
					segDescFactory.reset(new DescriptorImageSegMultiViewFactory(configEstimation.DESC_SIZE, configEstimation.DESC_SCALE_STEP, jmath::degToRad(configEstimation.DESC_ANGLE_STEP), (DescriptorImageSegMultiView::PredictionType)configEstimation.DESC_PREDICTION_TYPE));
				else
					segDescFactory.reset(new DescriptorImageSegFirstViewFactory(configEstimation.DESC_SIZE));

				boost::shared_ptr<HDsegDetector> hdsegDetector(new HDsegDetector(configEstimation.PATCH_SIZE, 3,configEstimation.PIX_NOISE*SEGMENT_NOISE_FACTOR,segDescFactory));
				boost::shared_ptr<DsegMatcher> dsegMatcher(new DsegMatcher(configEstimation.RANSAC_LOW_INNOV, configEstimation.MATCH_TH, configEstimation.MAHALANOBIS_TH, configEstimation.RELEVANCE_TH, configEstimation.PIX_NOISE*SEGMENT_NOISE_FACTOR));
				boost::shared_ptr<DataManager_ImageSeg_Test> dmSeg(new DataManager_ImageSeg_Test(hdsegDetector, dsegMatcher, assGrid, configEstimation.N_UPDATES_TOTAL, configEstimation.N_UPDATES_RANSAC, ransac_ntries, configEstimation.N_INIT, configEstimation.N_RECOMP_GAINS, configEstimation.MULTIPLE_DEPTH_HYPOS, (intOpts[iDump]&2) ? loggerTask.get() : NULL));

				dmSeg->linkToParentSensorSpec(senPtr11);
				dmSeg->linkToParentMapManager(mmSeg);
				dmSeg->setObservationFactory(obsFact);
			#endif
			#if SEGMENT_BASED != 1
					 if (configEstimation.MULTIVIEW_DESCRIPTOR)
							pointDescFactory.reset(new DescriptorImagePointMultiViewFactory(configEstimation.DESC_SIZE, configEstimation.DESC_SCALE_STEP, jmath::degToRad(configEstimation.DESC_ANGLE_STEP), (DescriptorImagePointMultiView::PredictionType)configEstimation.DESC_PREDICTION_TYPE));
					 else
							pointDescFactory.reset(new DescriptorImagePointFirstViewFactory(configEstimation.DESC_SIZE));

					 boost::shared_ptr<ImagePointHarrisDetector> harrisDetector(new ImagePointHarrisDetector(configEstimation.HARRIS_CONV_SIZE, configEstimation.HARRIS_TH, configEstimation.HARRIS_EDDGE, configEstimation.PATCH_SIZE, configEstimation.PIX_NOISE, pointDescFactory));
					 boost::shared_ptr<ImagePointZnccMatcher> znccMatcher(new ImagePointZnccMatcher(configEstimation.MIN_SCORE, configEstimation.PARTIAL_POSITION, configEstimation.PATCH_SIZE, configEstimation.MAX_SEARCH_SIZE, configEstimation.RANSAC_LOW_INNOV, configEstimation.MATCH_TH, configEstimation.HI_MATCH_TH, configEstimation.HI_LIMIT, configEstimation.MAHALANOBIS_TH, configEstimation.RELEVANCE_TH, configEstimation.PIX_NOISE));
					 boost::shared_ptr<DataManager_ImagePoint_Ransac> dmPt11(new DataManager_ImagePoint_Ransac(harrisDetector, znccMatcher, asGrid, configEstimation.N_UPDATES_TOTAL, configEstimation.N_UPDATES_RANSAC, ransac_ntries, configEstimation.N_INIT, configEstimation.N_RECOMP_GAINS, configEstimation.MULTIPLE_DEPTH_HYPOS, (intOpts[iDump]&2) ? loggerTask.get() : NULL));

					 dmPt11->linkToParentSensorSpec(senPtr11);
					 dmPt11->linkToParentMapManager(mmPoint);
					 dmPt11->setObservationFactory(obsFact);
			#endif
			if (dataLogger) dataLogger->addLoggable(*dmPt11.get());
		}
		cams_built[c] = senPtr11;
		ncam_built = c+1;
		} // ncam_built


		// c. Create hardware sensor
		if (intOpts[iSimu] == 0)
		{
			int cam_id = (ncams==1 && mode==hardware::mOnline ? 0 : c+1);
			if (configSetup.CAMERA_TYPE[c] == 0 || configSetup.CAMERA_TYPE[c] == 1)
			{ // VIAM
				#ifdef HAVE_VIAM
				viam_hwcrop_t crop;
				switch (configSetup.CAMERA_TYPE[c])
				{
					case 0: crop = VIAM_HW_FIXED; break;
					case 1: crop = VIAM_HW_CROP; break;
					default: crop = VIAM_HW_FIXED; break;
				}
				rtslamros::hardware::hardware_sensor_camera_ros_ptr_t hardSen11;
				for(int itry = 0; itry < 2; ++itry)
				{
					hardSen11 = rtslamros::hardware::hardware_sensor_camera_ros_ptr_t(new rtslamros::hardware::HardwareSensorCameraRos(&rawdata_condition, mode, c+1, cv::Size(img_width[c],img_height[c]),15.0,500,loggerTask.get(),strOpts[sDataPath]));
					if (hardSen11->initialized()) break; else std::cerr << "!HardwareSensorCameraRos " << hardSen11->id() << " failed to initialize" << (itry != 1 ? ", reset sensor and retry in 1 second." : ".") << std::endl;
					hardSen11.reset();
					if (itry != 1) sleep(1);
				}
				if (!hardSen11)
				{
					std::cerr << "!!HardwareSensorCameraRos " << cam_id << " failed to start" << (itrya != ntrya-1 ? ", resetting bus and retrying." : ", abandoning.") << std::endl ;
					initialized_cameras = false;
					for(int cc = 0; cc < c; cc++) if (cams[cc]) cams_built[cc]->setHardwareSensor(hardware::hardware_sensorext_ptr_t());
					sleep(1);
					int r = std::system("dc1394_reset_bus || dc1394_reset_bus2");
					if (r) std::cerr << "dc1394_reset_bus failed with error " << r << std::endl;
					sleep(1);
					break;
				}

//				if (!(intOpts[iReplay] & 1)) hardSen11->assessFirstImage(trigger_construction_date);
				hardSen11->setTimingInfos(1.0/hardSen11->getFreq(), 1.0/hardSen11->getFreq());
				hardSen11->setFilter(filter_div, filter_mods[c]);
				senPtr11->setHardwareSensor(hardSen11);
				#else
				if (intOpts[iReplay] & 1)
				{
					hardware::hardware_sensorext_ptr_t hardSen11(new rtslamros::hardware::HardwareSensorCameraRos(&rawdata_condition, mode, c+1, cv::Size(img_width[c],img_height[c]),500,loggerTask.get(),strOpts[sDataPath]));
					senPtr11->setHardwareSensor(hardSen11);
				} else
				{
					std::cerr << "You need to install the library viam to use a firewire camera" << std::endl;
					worldPtr->error = eDependency; return false;
				}
				#endif
			} else if (configSetup.CAMERA_TYPE[c] == 2)
			{ // V4L or VIAM ?
				std::cerr << "Generic USB cameras not supported yet ; use firewire of ueye camera" << std::endl;
				worldPtr->error = eNotSupported; return false;
			} else if (configSetup.CAMERA_TYPE[c] == 3)
			{ // UEYE
				#ifdef HAVE_UEYE
				hardware::hardware_sensor_ueye_ptr_t hardSen11(new hardware::HardwareSensorCameraUeye(&rawdata_condition, 500,
					configSetup.CAMERA_DEVICE[c], cv::Size(img_width[c],img_height[c]), floatOpts[fFreq], intOpts[iTrigger],
					floatOpts[fShutter], mode, cam_id, strOpts[sDataPath], loggerTask.get()));
				hardSen11->setTimingInfos(1.0/hardSen11->getFreq(), 1.0/hardSen11->getFreq());
				senPtr11->setHardwareSensor(hardSen11);
				#else
				if (intOpts[iReplay] & 1)
				{
					hardware::hardware_sensorext_ptr_t hardSen11(new hardware::HardwareSensorCameraUeye(&rawdata_condition, c+1, cv::Size(img_width[c],img_height[c]),strOpts[sDataPath]));
					senPtr11->setHardwareSensor(hardSen11);
				} else
				{
					std::cerr << "You need to install the ueye driver to use a ueye camera" << std::endl;
					worldPtr->error = eDependency; return false;
				}
				#endif
			}
		}
	} if (initialized_cameras) break; } // for each camera
	if (!initialized_cameras)
	{
		std::cerr << "Could not start all cameras." << std::endl;
		worldPtr->error = eNoSensorData;
		return false;
	}

	if (intOpts[iGps])
	{
		int cp_size = configSetup.GPS_POSE.size();
		if (cp_size != 6 && cp_size != 12) { std::cerr << "Gps pose must have size 6 or 12 (with uncertainties), not " << cp_size << std::endl; worldPtr->error = eConfig; return false; }
		absloc_ptr_t senPtr13(new SensorAbsloc(robPtr1, (cp_size == 6 ? MapObject::UNFILTERED : MapObject::FILTERED), -1.0, configSetup.GPS_MAX_CONSIST_SIG, true, intOpts[iGps] == 2));
		senPtr13->linkToParentRobot(robPtr1);
		senPtr13->name("GPS");
		hardware::hardware_sensorprop_ptr_t hardGps;
		bool init = true;
		switch (intOpts[iGps])
		{
			case 1:
				hardGps.reset(new hardware::HardwareSensorGpsGenom(&rawdata_condition, 200, mode, strOpts[sDataPath], loggerTask.get()));
				break;
			case 2:
				hardGps.reset(new hardware::HardwareSensorGpsGenom(&rawdata_condition, 200, mode, strOpts[sDataPath], loggerTask.get())); // TODO ask to ignore vel
				break;
			case 3:
				hardGps.reset(new hardware::HardwareSensorMocap(&rawdata_condition, 200, mode, strOpts[sDataPath], loggerTask.get()));
				init = false;
				break;
		}

		hardGps->setSyncConfig(configSetup.GPS_TIMESTAMP_CORRECTION);
		hardGps->setTimingInfos(1.0/20.0, 1.5/20.0);
		senPtr13->setHardwareSensor(hardGps);
		senPtr13->setIntegrationPolicy(true);
		senPtr13->setUseForInit(true);
		senPtr13->setNeedInit(init);
		if (cp_size == 6)
		{
			senPtr13->setPose(configSetup.GPS_POSE[0], configSetup.GPS_POSE[1], configSetup.GPS_POSE[2],
												configSetup.GPS_POSE[3], configSetup.GPS_POSE[4], configSetup.GPS_POSE[5]); // x,y,z,roll,pitch,yaw
		} else
		{
			senPtr13->setPoseStd(configSetup.GPS_POSE[0], configSetup.GPS_POSE[1], configSetup.GPS_POSE[2],
												configSetup.GPS_POSE[3], configSetup.GPS_POSE[4], configSetup.GPS_POSE[5],
												configSetup.GPS_POSE[6], configSetup.GPS_POSE[7], configSetup.GPS_POSE[8],
												configSetup.GPS_POSE[9], configSetup.GPS_POSE[10], configSetup.GPS_POSE[11]); // x,y,z,roll,pitch,yaw + std_dev
		}
	}

	if (intOpts[iOdom])
	{
		int cp_size = configSetup.ROBOT_POSE.size();
		if (cp_size != 6 && cp_size != 12) { std::cerr << "Robot pose must have size 6 or 12 (with uncertainties), not " << cp_size << std::endl; worldPtr->error = eConfig; return false; }
		absloc_ptr_t senPtr14(new SensorAbsloc(robPtr1, (cp_size == 6 ? MapObject::UNFILTERED : MapObject::FILTERED), 3.0, 1e9, false, true, false));
		senPtr14->linkToParentRobot(robPtr1);
		senPtr14->name("odom");
//		robPtr1->registerRobotQuantity(RobotAbstract::qAngVel);
		hardware::HardwareSensorOdomRmp400Genom *odomRmp = new hardware::HardwareSensorOdomRmp400Genom(&rawdata_condition, 200, mode, strOpts[sDataPath], loggerTask.get());
		odomRmp->setCalib(configSetup.ODO_CALIB);
		hardware::hardware_sensorprop_ptr_t hardOdom(odomRmp);

		hardOdom->setSyncConfig(configSetup.ODO_TIMESTAMP_CORRECTION);
		hardOdom->setTimingInfos(1.0/20.0, 1.5/20.0);
		senPtr14->setHardwareSensor(hardOdom);
		senPtr14->setIntegrationPolicy(true);
		senPtr14->setUseForInit(false);
		senPtr14->setNeedInit(false);
		senPtr14->setPose(configSetup.ROBOT_POSE[0], configSetup.ROBOT_POSE[1], configSetup.ROBOT_POSE[2],
											configSetup.ROBOT_POSE[3], configSetup.ROBOT_POSE[4], configSetup.ROBOT_POSE[5]); // x,y,z,roll,pitch,yaw
		if (cp_size == 6)
		{
			senPtr14->setPose(configSetup.ROBOT_POSE[0], configSetup.ROBOT_POSE[1], configSetup.ROBOT_POSE[2],
												configSetup.ROBOT_POSE[3], configSetup.ROBOT_POSE[4], configSetup.ROBOT_POSE[5]); // x,y,z,roll,pitch,yaw
		} else
		{
			senPtr14->setPoseStd(configSetup.ROBOT_POSE[0], configSetup.ROBOT_POSE[1], configSetup.ROBOT_POSE[2],
												configSetup.ROBOT_POSE[3], configSetup.ROBOT_POSE[4], configSetup.ROBOT_POSE[5],
												configSetup.ROBOT_POSE[6], configSetup.ROBOT_POSE[7], configSetup.ROBOT_POSE[8],
												configSetup.ROBOT_POSE[9], configSetup.ROBOT_POSE[10], configSetup.ROBOT_POSE[11]); // x,y,z,roll,pitch,yaw + std_dev
		}
	}

	if (intOpts[iExtloc]/10)
	{
		absloc_ptr_t senPtr15(new SensorAbsloc(robPtr1, MapObject::UNFILTERED, -1.0, 1e9, true, false, false));
		senPtr15->linkToParentRobot(robPtr1);
		senPtr15->name("extloc");
		int source = intOpts[iExtloc]/10-1; // source online poster or manual file
		hardware::ExtLocType type = (hardware::ExtLocType)(intOpts[iExtloc]%10);
		hardware::HardwareSensorExternalLoc *extloc = new hardware::HardwareSensorExternalLoc(&rawdata_condition, 200, type, source, mode, strOpts[sDataPath], loggerTask.get());
		hardware::hardware_sensorprop_ptr_t hardExtloc(extloc);

		hardExtloc->setTimingInfos(1.0/2.0, (source ? -0.05 : 0.5));
		senPtr15->setHardwareSensor(hardExtloc);
		senPtr15->setIntegrationPolicy(true);
		senPtr15->setUseForInit(false);
		senPtr15->setNeedInit(false);
		senPtr15->setPose(0,0,0,0,0,0); // x,y,z,roll,pitch,yaw
	}


	/// Sensor Manager
	if (intOpts[iReplay] & 1)
		sensorManager.reset(new SensorManagerOffline(mapPtr, (intOpts[iReplay] == 3 ? strOpts[sDataPath] : "")));
	else
		sensorManager.reset(new SensorManagerOnline(mapPtr, ((intOpts[iDump] & 1) ? strOpts[sDataPath] : ""), loggerTask.get()));

	/// ---------------------------------------------------------------------------
	/// --- INIT DISPLAY ----------------------------------------------------------
	/// ---------------------------------------------------------------------------

	//--- force a first display with empty slam to ensure that all windows are loaded
	#ifdef HAVE_MODULE_QDISPLAY
	if (intOpts[iDispQt])
	{
		viewerQt = PTR_CAST<display::ViewerQt*> (worldPtr->getDisplayViewer(display::ViewerQt::id()));
		viewerQt->bufferize(worldPtr);

		// initializing stuff for controlling run/pause from viewer
		boost::unique_lock<boost::mutex> runStatus_lock(viewerQt->runStatus.mutex);
		viewerQt->runStatus.pause = intOpts[iPause];
		viewerQt->runStatus.render_all = intOpts[iRenderAll];
		runStatus_lock.unlock();
	}
	#endif
	#ifdef HAVE_MODULE_GDHE
	if (intOpts[iDispGdhe])
	{
		viewerGdhe = PTR_CAST<display::ViewerGdhe*> (worldPtr->getDisplayViewer(display::ViewerGdhe::id()));
		viewerGdhe->bufferize(worldPtr);
	}
	#endif

	//worldPtr->display_mutex.unlock();

	switch (intOpts[iExport])
	{
		case 1: exporter.reset(new ExporterSocket(robPtr1, 30000)); break;
		case 2: exporter.reset(new ExporterPoster(robPtr1)); break;
	}

	return true;
JFR_GLOBAL_CATCH
} // demo_slam_init


/** ############################################################################
 * #############################################################################
 * Exit function
 * ###########################################################################*/

void demo_slam_exit(world_ptr_t *world, boost::thread *thread_main) {
	(*world)->exit(true);
	(*world)->display_condition.notify_all();

#ifdef HAVE_MODULE_QDISPLAY
	if (intOpts[iDispQt])
	{
		viewerQt->runStatus.pause = 0;
		viewerQt->runStatus.condition.notify_all();
	}
#endif

// 	std::cout << "EXITING !!!" << std::endl;
	//fputc('\n', stdin);
	thread_main->join();
	//thread_main->timed_join(boost::posix_time::milliseconds(500));
}

void demo_slam_stop(world_ptr_t *world)
{
	sensorManager->printStats();

	static int stopped = false;
	if (stopped) return;

	if (exporter) exporter->stop();
	(*world)->slam_blocked(true);

	// stop all sensors
	map_ptr_t mapPtr = (*world)->mapList().front();
	ready = false;
	for (MapAbstract::RobotList::iterator robIter = mapPtr->robotList().begin();
		robIter != mapPtr->robotList().end(); ++robIter)
	{
		if ((*robIter)->hardwareEstimatorPtr)
		{
			std::cout << "Stopping robot " << (*robIter)->id() << " estimator..."; std::cout.flush();
			(*robIter)->hardwareEstimatorPtr->stop();
			std::cout << " OK." << std::endl;
		}
		for (RobotAbstract::SensorList::iterator senIter = (*robIter)->sensorList().begin();
			senIter != (*robIter)->sensorList().end(); ++senIter)
		{
			std::cout << "Stopping sensor " << (*senIter)->id() << " " << (*senIter)->name() << "..."; std::cout.flush();
			(*senIter)->stop();
			std::cout << " OK." << std::endl;
		}
	}

	std::cout << "Stopping sensor manager..."; std::cout.flush();
	sensorManager->stop();
	std::cout << " OK." << std::endl;

	if (loggerTask)
	{
		std::cout << "Stopping and joining logger..."; std::cout.flush();
		loggerTask->stop(true);
		std::cout << " OK." << std::endl;
	}

	for (MapAbstract::RobotList::iterator robIter = mapPtr->robotList().begin();
		robIter != mapPtr->robotList().end(); ++robIter)
	{
		if ((*robIter)->hardwareEstimatorPtr)
		{
			std::cout << "Joining robot " << (*robIter)->id() << " estimator..."; std::cout.flush();
			(*robIter)->hardwareEstimatorPtr->join();
			std::cout << " OK." << std::endl;
		}
		for (RobotAbstract::SensorList::iterator senIter = (*robIter)->sensorList().begin();
			senIter != (*robIter)->sensorList().end(); ++senIter)
		{
			std::cout << "Joining sensor " << (*senIter)->id() << " " << (*senIter)->name() << "..."; std::cout.flush();
			if ((*senIter)->join(1000)) std::cout << " OK." << std::endl; else std::cout << " FAILED." << std::endl;
		}
	}

	std::cout << "Quitting" << std::endl;
	stopped = true;
}


void set_signals(sig_t catcher)
{
	signal(SIGQUIT, catcher);
	signal(SIGTERM, catcher);
	signal(SIGINT, catcher); // ctrl-c

	signal(SIGABRT, catcher);
	signal(SIGPIPE, catcher);
	signal(SIGFPE, catcher);
	signal(SIGSEGV, catcher);
	//signal(SIGBUS, catcher);
}


void signal_catcher(int sig __attribute__((unused)))
{
	if (worldPtr->error == eNoError) worldPtr->error = eCrashed;
	static bool first = true;
	if (first)
	{
		first = false;
		std::cerr << "RT-SLAM is stopping because it received signal " << sig << " \"" << strsignal(sig) << "\"" << std::endl;
		demo_slam_stop(&worldPtr);

		// force deleting sensors object to call their destructor
		map_ptr_t mapPtr = worldPtr->mapList().front();
		for (MapAbstract::RobotList::iterator robIter = mapPtr->robotList().begin();
			robIter != mapPtr->robotList().end(); ++robIter)
		{
			for (RobotAbstract::SensorList::iterator senIter = (*robIter)->sensorList().begin();
				senIter != (*robIter)->sensorList().end(); ++senIter)
				delete (*senIter).get();
			if ((*robIter)->hardwareEstimatorPtr.get() != trigger.get()) delete (*robIter).get();
		}
		delete trigger.get();
	}
	else std::cerr << "RT-SLAM failed to stop because it received signal " << sig << " \"" << strsignal(sig) << "\"" << std::endl;

	signal(sig, SIG_DFL);
	raise(sig);
}


/** ############################################################################
 * #############################################################################
 * Main function
 * ###########################################################################*/

#define SHOW_PERIODIC_SENSORS_INFOS 1

void showSensorsInfos(map_ptr_t mapPtr)
{
	if (!(intOpts[iReplay] & 1))
	{
		#if SHOW_PERIODIC_SENSORS_INFOS
		std::cout << std::setprecision(16) << kernel::Clock::getTime() << std::endl;
		#endif
		for (MapAbstract::RobotList::iterator robIter = mapPtr->robotList().begin();
			robIter != mapPtr->robotList().end(); ++robIter)
		{
			if ((*robIter)->hardwareEstimatorPtr) (*robIter)->hardwareEstimatorPtr->showInfos();
			for (RobotAbstract::SensorList::iterator senIter = (*robIter)->sensorList().begin();
				senIter != (*robIter)->sensorList().end(); ++senIter)
				(*senIter)->showInfos();
		}
	}
}

void demo_slam_main(world_ptr_t *world)
{ JFR_GLOBAL_TRY

	if (!(intOpts[iReplay] & 1))
		kernel::setCurrentThreadScheduler(slam_sched, slam_priority);

	robot_ptr_t robotPtr;

	// wait for display to be ready if enabled
	if (intOpts[iDispQt] || intOpts[iDispGdhe])
	{
		boost::unique_lock<boost::mutex> display_lock(worldPtr->display_mutex);
		worldPtr->display_rendered = false;
		display_lock.unlock();
		worldPtr->display_condition.notify_all();
// std::cout << "SLAM: now waiting for this display to finish" << std::endl;
		display_lock.lock();
		while(!worldPtr->display_rendered) worldPtr->display_condition.wait(display_lock);
		display_lock.unlock();
	}

	set_signals(signal_catcher);

	// start hardware sensors that need long init
	map_ptr_t mapPtr = (*world)->mapList().front();
	bool has_init = false;
	for (MapAbstract::RobotList::iterator robIter = mapPtr->robotList().begin();
		robIter != mapPtr->robotList().end(); ++robIter)
	{
		if ((*robIter)->hardwareEstimatorPtr)  { has_init = true; (*robIter)->hardwareEstimatorPtr->start(); }
		for (RobotAbstract::SensorList::iterator senIter = (*robIter)->sensorList().begin();
			senIter != (*robIter)->sensorList().end(); ++senIter)
		{
			if ((*senIter)->getNeedInit())
				{ has_init = true; (*senIter)->start(); }
		}
		robotPtr = *robIter;
	}

	// wait for their init
	if (has_init && !(intOpts[iReplay] & 1))
	{
		std::cout << "Sensors are calibrating... DON'T MOVE THE SYSTEM!!" << std::flush;
		sleep(2);
		std::cout << " done." << std::endl;

		bool abort = false;
		for (MapAbstract::RobotList::iterator robIter = mapPtr->robotList().begin();
			robIter != mapPtr->robotList().end(); ++robIter)
		{
			for (RobotAbstract::SensorList::iterator senIter = (*robIter)->sensorList().begin();
				senIter != (*robIter)->sensorList().end(); ++senIter)
			{
				if ((*senIter)->getNeedInit())
				{
					RawInfos infos;
					(*senIter)->queryAvailableRaws(infos);
					if (infos.available.size() == 0)
					{
						std::cerr << "Sensor " << (*senIter)->name() << " has no data, stopping." << std::endl;
						abort = true;
					}
				}
			}
		}
		if (abort)
		{
			worldPtr->error = eNoSensorData;
			JFR_ERROR(RtslamException, RtslamException::NO_SENSOR_DATA, "Some sensors have no data.");
		}
	}


	// set the start date
	double start_date = kernel::Clock::getTime();
	if (intOpts[iSimu]) start_date = 0.0;
	if (!(intOpts[iReplay] & 1) && (intOpts[iDump] & 1)) {
		std::fstream f((strOpts[sDataPath] + std::string("/sdate.log")).c_str(), std::ios_base::out);
		f << std::setprecision(19) << start_date << std::endl;
		f.close();
	}
	else if (intOpts[iReplay] & 1) {
		std::fstream f((strOpts[sDataPath] + std::string("/sdate.log")).c_str(), std::ios_base::in);
		if (!f.is_open()) { std::cout << "Missing sdate.log file. Please copy the .time file of the first image to sdate.log" << std::endl; return; }
		f >> start_date;
		f.close();
	}
	std::cout << "slam start date: " << std::setprecision(16) << start_date << std::endl;
	sensorManager->setStartDate(start_date);

	// start other hardware sensors
	for (MapAbstract::RobotList::iterator robIter = mapPtr->robotList().begin();
		robIter != mapPtr->robotList().end(); ++robIter)
	{
		for (RobotAbstract::SensorList::iterator senIter = (*robIter)->sensorList().begin();
			senIter != (*robIter)->sensorList().end(); ++senIter)
		{
			if (!(*senIter)->getNeedInit())
				(*senIter)->start();
		}
	}

	#if !SHOW_PERIODIC_SENSORS_INFOS
	showSensorsInfos(mapPtr);
	#endif

	#if STATS
	jblas::vec robot_prediction;
	double average_robot_innovation_pos = 0.;
	double average_robot_innovation_ori = 0.;
	int n_innovation = 0;
	#endif

	// ---------------------------------------------------------------------------
	// --- LOOP ------------------------------------------------------------------
	// ---------------------------------------------------------------------------
	// INIT : complete observations set
	// loop all sensors
	// loop all lmks
	// create sen--lmk observation
	// Temporal loop
	//if (dataLogger) dataLogger->log();
	double filterTime = 0.0;
	kernel::Chrono chrono;
	double next_show_infos = -1;

	while (!(*world)->exit())
	{
		bool had_data = false;
		chrono.reset();

		SensorManagerAbstract::ProcessInfo pinfo = sensorManager->getNextDataToUse(filterTime);
		bool no_more_data = pinfo.no_more_data;

		if (pinfo.sen)
		{
			had_data = true;
			robot_ptr_t robPtr = pinfo.sen->robotPtr();
			if (intOpts[iReplay] == 2)
				pinfo.sen->process_fake(pinfo.id, true); // just to release data
			else
			{
				double newt = pinfo.date;
				#if SHOW_PERIODIC_SENSORS_INFOS
				if (newt > next_show_infos) { showSensorsInfos(mapPtr); next_show_infos = newt+2.; }
				#endif

				JFR_DEBUG("************** FRAME : " << (*world)->t << " (" << std::setprecision(16) << newt << std::setprecision(6) << ") sensor " << pinfo.sen->id());

//std::cout << "Frame " << (*world)->t << " using sen " << pinfo.sen->id() << " at time " << std::setprecision(16) << newt << std::endl;

				// wait to have all the estimator data (ie one after newt) to do this move,
				// or it can cause trouble if there are two many missing data,
				// and it ensures offline repeatability, and quality will be better
				// TODO be smarter and choose an older data if possible
				bool waited = false;
				double wait_time;
				estimatordata_condition.set(0);
				double start_date = kernel::Clock::getTime();
				double waitedmove_date = start_date;
				bool stop = false;
				while (!robPtr->move(newt))
				{
					if (!waited) wait_time = kernel::Clock::getTime();
					waited = true;
					if (robPtr->hardwareEstimatorPtr->stopped()) { stop = true; break; }
					estimatordata_condition.wait(boost::lambda::_1 != 0);
					estimatordata_condition.set(0);
					waitedmove_date = kernel::Clock::getTime();
				}
				double moved_date = kernel::Clock::getTime();
				if (stop)
				{
					std::cout << "No more estimator data, stopping." << std::endl;
					break;
				}
				if (waited)
				{
					wait_time = kernel::Clock::getTime() - wait_time;
					/*if (wait_time > 0.001)*/ std::cout << "wa(i|s)ted " << wait_time << " for estimator data" << std::endl;
				}


				if (!ready && sensorManager->allInit())
				{ // here to ensure that at least one move has been done (to init estimator)
					robPtr->reinit_extrapolate();
					ready = true;
				}

				JFR_DEBUG("Robot " << robPtr->id() << " state after move " << robPtr->state.x() << " ; euler " << quaternion::q2e(ublas::subrange(robPtr->state.x(), 3, 7)));
				JFR_DEBUG("Robot state stdev after move " << stdevFromCov(robPtr->state.P()));
				#if STATS
				robot_prediction = robPtr->pose.x();
				#endif

				#if REAL_TIME_LIVE_RUN
				pinfo.sen->process(pinfo.id, pinfo.date_next);
				#else
				pinfo.sen->process(pinfo.id, -1.);
				#endif
				pinfo.sen->robotPtr()->last_updated = pinfo.sen;

				#if STATE_HISTORY
				pinfo.sen->robotPtr()->historyManager->process(newt);
				//pinfo.sen->robotPtr()->historyManager->print();
				#endif

				JFR_DEBUG("Robot state after corrections of sensor " << pinfo.sen->id() << " : " << robPtr->state.x() << " ; euler " << quaternion::q2e(ublasExtra::normalized(ublas::subrange(robPtr->state.x(), 3, 7))));
				JFR_DEBUG("Robot state stdev after corrections " << stdevFromCov(robPtr->state.P()));
				#if STATS
				average_robot_innovation_pos += ublas::norm_2(ublas::subrange(robPtr->pose.x(),0,3) - ublas::subrange(robot_prediction,0,3));
				average_robot_innovation_ori += ublas::norm_2(ublas::subrange(robPtr->pose.x(),3,7) - ublas::subrange(robot_prediction,3,7));
				n_innovation++;
				#endif

				robPtr->reinit_extrapolate();
				double processed_date = kernel::Clock::getTime();
				if (exporter && ready) exporter->exportCurrentState();

				sensorManager->logData(pinfo.sen, start_date, waitedmove_date, moved_date, processed_date);
			}
			filterTime = robPtr->self_time;
		}


		// wait that display has finished if render all
		if (had_data)
		{
			// get render all status
			bool renderAll;
			#ifdef HAVE_MODULE_QDISPLAY
			if (intOpts[iDispQt])
			{
				boost::unique_lock<boost::mutex> runStatus_lock(viewerQt->runStatus.mutex);
				renderAll = viewerQt->runStatus.render_all;
				runStatus_lock.unlock();
			} else
			#endif
			renderAll = (intOpts[iRenderAll] != 0);

			// if render all, wait display has finished
			if ((intOpts[iDispQt] || intOpts[iDispGdhe]) && renderAll)
			{
				boost::unique_lock<boost::mutex> display_lock((*world)->display_mutex);
				while(!(*world)->display_rendered && !(*world)->exit()) (*world)->display_condition.wait(display_lock);
				display_lock.unlock();
			}
		}


		// asking for display if display has finished
		unsigned processed_t = (had_data ? (*world)->t : (*world)->t-1);
		if ((*world)->display_t+1 < processed_t+1)
		{
			boost::unique_lock<boost::mutex> display_lock((*world)->display_mutex);
			if ((*world)->display_rendered)
			{
				#ifdef HAVE_MODULE_QDISPLAY
				display::ViewerQt *viewerQt = NULL;
				if (intOpts[iDispQt]) viewerQt = PTR_CAST<display::ViewerQt*> ((*world)->getDisplayViewer(display::ViewerQt::id()));
				if (intOpts[iDispQt]) viewerQt->bufferize(*world);
				#endif
				#ifdef HAVE_MODULE_GDHE
				display::ViewerGdhe *viewerGdhe = NULL;
				if (intOpts[iDispGdhe]) viewerGdhe = PTR_CAST<display::ViewerGdhe*> ((*world)->getDisplayViewer(display::ViewerGdhe::id()));
				if (intOpts[iDispGdhe]) viewerGdhe->bufferize(*world);
				#endif

				(*world)->display_t = (*world)->t;
				(*world)->display_rendered = false;
				display_lock.unlock();
				(*world)->display_condition.notify_all();
			} else
			display_lock.unlock();
		}

		if (no_more_data) break;

		if (!had_data)
		{
			if (pinfo.date_next < 0) rawdata_condition.wait(boost::lambda::_1 != 0); else
				rawdata_condition.timed_wait(boost::lambda::_1 != 0, boost::posix_time::microseconds((pinfo.date_next-kernel::Clock::getTime())*1e6));
		}
		rawdata_condition.set(0);

		bool doPause;
		#ifdef HAVE_MODULE_QDISPLAY
		if (intOpts[iDispQt])
		{
			boost::unique_lock<boost::mutex> runStatus_lock(viewerQt->runStatus.mutex);
			doPause = viewerQt->runStatus.pause;
			runStatus_lock.unlock();
		} else
		#endif
		doPause = (intOpts[iPause] != 0);
		if (doPause && had_data && !(*world)->exit())
		{
			(*world)->slam_blocked(true);
			#ifdef HAVE_MODULE_QDISPLAY
			if (intOpts[iDispQt])
			{
				boost::unique_lock<boost::mutex> runStatus_lock(viewerQt->runStatus.mutex);
				do {
					viewerQt->runStatus.condition.wait(runStatus_lock);
				} while (viewerQt->runStatus.pause && !viewerQt->runStatus.next);
				viewerQt->runStatus.next = 0;
				runStatus_lock.unlock();
			} else
			#endif
			getchar(); // wait for key in replay mode
			(*world)->slam_blocked(false);
		}

		if (had_data)
		{
			(*world)->t++;
			if (dataLogger) dataLogger->log();
		}
	} // temporal loop

	std::cout << "Stopping RT-SLAM" << std::endl;
	#if STATS
	average_robot_innovation_pos /= n_innovation;
	average_robot_innovation_ori /= n_innovation;
	std::cout << "innovation" <<
		" avg " <<  (*world)->mapList().front()->filterPtr->avg_innov() <<
		" max " <<  (*world)->mapList().front()->filterPtr->max_innov() <<
		" robot_pos_avg " << average_robot_innovation_pos <<
		" robot_ori_avg " << average_robot_innovation_ori << std::endl;
	std::cout << "LandmarkCounts EP " << LandmarkEuclideanPoint::ncreated << " AHP " << LandmarkAnchoredHomogeneousPoint::ncreated << std::endl;
	#endif
	robot_ptr_t robPtr = (*world)->mapList().front()->robotList().front();
	std::cout << "final_robot_position " << robPtr->state.x(0) << " " << robPtr->state.x(1) << " " << robPtr->state.x(2) << std::endl;

	demo_slam_stop(world);
//	std::cout << "\nFINISHED ! Press a key to terminate." << std::endl;
//	getchar();

JFR_GLOBAL_CATCH
} // demo_slam_main


/** ############################################################################
 * #############################################################################
 * Display function
 * ###########################################################################*/

bool demo_slam_display_first = true;

void demo_slam_display(world_ptr_t *world)
{ JFR_GLOBAL_TRY

	if (demo_slam_display_first && !(intOpts[iReplay] & 1))
	{
		kernel::setCurrentThreadPriority(display_niceness);
		demo_slam_display_first = false;
	}

//	static unsigned prev_t = 0;
	kernel::Timer timer(display_period*1000);
	while(!(*world)->exit())
	{
		/*
		if (intOpts[iDispQt])
		{
			#ifdef HAVE_MODULE_QDISPLAY
			qdisplay::qtMutexLock<kernel::FifoMutex>((*world)->display_mutex);
			#endif
		}
		else
		{
			(*world)->display_mutex.lock();
		}
		*/
		// just to display the last frame if slam is blocked or has finished
		boost::unique_lock<kernel::VariableMutex<bool> > blocked_lock((*world)->slam_blocked);
		if ((*world)->slam_blocked.var)
		{
			if ((*world)->display_t+1 < (*world)->t+1 && (*world)->display_rendered)
			{
				#ifdef HAVE_MODULE_QDISPLAY
				display::ViewerQt *viewerQt = NULL;
				if (intOpts[iDispQt]) viewerQt = PTR_CAST<display::ViewerQt*> ((*world)->getDisplayViewer(display::ViewerQt::id()));
				if (intOpts[iDispQt]) viewerQt->bufferize(*world);
				#endif
				#ifdef HAVE_MODULE_GDHE
				display::ViewerGdhe *viewerGdhe = NULL;
				if (intOpts[iDispGdhe]) viewerGdhe = PTR_CAST<display::ViewerGdhe*> ((*world)->getDisplayViewer(display::ViewerGdhe::id()));
				if (intOpts[iDispGdhe]) viewerGdhe->bufferize(*world);
				#endif

				(*world)->display_t = (*world)->t;
				(*world)->display_rendered = false;
			}
		}
		blocked_lock.unlock();

		// waiting that display is ready
// std::cout << "DISPLAY: waiting for data" << std::endl;
		boost::unique_lock<boost::mutex> display_lock((*world)->display_mutex);
		if (intOpts[iDispQt] == 0)
		{
			while((*world)->display_rendered)
				(*world)->display_condition.wait(display_lock);
		} else
		{
			#ifdef HAVE_MODULE_QDISPLAY
			int nwait = std::max(1,display_period/10-1);
			for(int i = 0; (*world)->display_rendered && i < nwait; ++i)
			{
				(*world)->display_condition.timed_wait(display_lock, boost::posix_time::milliseconds(10));
				display_lock.unlock();
				QApplication::instance()->processEvents();
				display_lock.lock();
			}
			if ((*world)->display_rendered) break;
			#endif
		}
		display_lock.unlock();
// std::cout << "DISPLAY: ok data here, let's start!" << std::endl;

//		if ((*world)->t != prev_t)
//		{
//			prev_t = (*world)->t;
//			(*world)->display_rendered = (*world)->t;

//		!bufferize!

//			if (!intOpts[iRenderAll]) // strange: if we always unlock here, qt.dump takes much more time...
//				(*world)->display_mutex.unlock();

			#ifdef HAVE_MODULE_QDISPLAY
			display::ViewerQt *viewerQt = NULL;
			if (intOpts[iDispQt]) viewerQt = PTR_CAST<display::ViewerQt*> ((*world)->getDisplayViewer(display::ViewerQt::id()));
			if (intOpts[iDispQt]) viewerQt->render();
			#endif
			#ifdef HAVE_MODULE_GDHE
			display::ViewerGdhe *viewerGdhe = NULL;
			if (intOpts[iDispGdhe]) viewerGdhe = PTR_CAST<display::ViewerGdhe*> ((*world)->getDisplayViewer(display::ViewerGdhe::id()));
			if (intOpts[iDispGdhe]) viewerGdhe->render();
			#endif

			if (((intOpts[iReplay] & 1) || intOpts[iSimu]) && (intOpts[iDump] & 1) && (*world)->display_t+1 != 0)
			{
				#ifdef HAVE_MODULE_QDISPLAY
				if (intOpts[iDispQt])
				{
					std::ostringstream oss; oss << strOpts[sDataPath] << "/rendered-2D_%d-" << std::setw(6) << std::setfill('0') << (*world)->display_t << ".png";
					viewerQt->dump(oss.str());
				}
				#endif
				#ifdef HAVE_MODULE_GDHE
				if (intOpts[iDispGdhe])
				{
					std::ostringstream oss; oss << strOpts[sDataPath] << "/rendered-3D_" << std::setw(6) << std::setfill('0') << (*world)->display_t << ".png";
					viewerGdhe->dump(oss.str());
				}
				#endif
//				if (intOpts[iRenderAll])
//					(*world)->display_mutex.unlock();
			}
//		} else
//		{
//			(*world)->display_mutex.unlock();
//			boost::this_thread::yield();
//		}
// std::cout << "DISPLAY: finished display, marking rendered" << std::endl;
		display_lock.lock();
		(*world)->display_rendered = true;
		display_lock.unlock();
		(*world)->display_condition.notify_all();

		if (intOpts[iDispQt]) break; else timer.wait();
	}

JFR_GLOBAL_CATCH
} // demo_slam_display


/** ############################################################################
 * #############################################################################
 * Demo function
 * ###########################################################################*/


void demo_slam_run() {

	//kernel::setProcessScheduler(slam_sched, 5); // for whole process, including display thread
	demo_slam_display_first = true;

	// to start with qt display
	if (intOpts[iDispQt]) // at least 2d
	{
		#ifdef HAVE_MODULE_QDISPLAY
		qdisplay::QtAppStart((qdisplay::FUNC)&demo_slam_display,0,(qdisplay::FUNC)&demo_slam_main,0,display_period,&worldPtr,(qdisplay::EXIT_FUNC)&demo_slam_exit);
		#else
		std::cout << "Please install qdisplay module if you want 2D display" << std::endl;
		#endif
	} else
	if (intOpts[iDispGdhe]) // only 3d
	{
		#ifdef HAVE_MODULE_GDHE
		boost::thread *thread_disp = new boost::thread(boost::bind(demo_slam_display,&worldPtr));
		demo_slam_main(&worldPtr);
		delete thread_disp;
		#else
		std::cout << "Please install gdhe module if you want 3D display" << std::endl;
		#endif
	} else // none
	{
		demo_slam_main(&worldPtr);
	}

	JFR_DEBUG("Terminated");
}


/** ############################################################################
 * #############################################################################
 * Config file loading
 * ###########################################################################*/

#define TOKENPASTE(x, y, z) x ## y ## z
#define TOKENPASTE2(x, y, z) TOKENPASTE(x, y, z)
#define TOKENSTRING(x) #x


#define KeyValueFile_processItem(k) { read ? keyValueFile.getItem(#k, k) : keyValueFile.setItem(#k, k); }
#define KeyValueFile_processItem_def(k, def) { read ? keyValueFile.getItem(#k, k, def) : keyValueFile.setItem(#k, k); }
#define KeyValueFile_processItem_ind(pref, i, k) { std::ostringstream oss; oss << #pref << i << #k; read ? keyValueFile.getItem(oss.str(), pref##k[i-1]) : keyValueFile.setItem(oss.str(), pref##k[i-1]); }
#define KeyValueFile_processItem_ind_def(pref, i, k, def) { std::ostringstream oss; oss << #pref << i << #k; read ? keyValueFile.getItem(oss.str(), pref##k[i-1], def) : keyValueFile.setItem(oss.str(), pref##k[i-1]); }


void ConfigSetup::loadKeyValueFile(jafar::kernel::KeyValueFile const& keyValueFile)
{
  jafar::kernel::KeyValueFile &keyValueFile2 = const_cast<jafar::kernel::KeyValueFile&>(keyValueFile);
  processKeyValueFile(keyValueFile2, true);
}
void ConfigSetup::saveKeyValueFile(jafar::kernel::KeyValueFile& keyValueFile)
{
  processKeyValueFile(keyValueFile, false);
}

void ConfigSetup::processKeyValueFile(jafar::kernel::KeyValueFile& keyValueFile, bool read)
{
	int camsi = intOpts[iCamera]/10;
	const int ncam = 3;
	int ncams = 0; bool cams[ncam]; for(int i = 0; i < ncam; i++) { cams[i] = camsi&(1<<i); if (cams[i]) ncams++; }
	bool load_calib = ((intOpts[iCamera]%10) > 0);

	if (intOpts[iRobot] == 1)
	{
		try { for(int i = 1; i <= ncam; ++i) if (cams[i-1]) KeyValueFile_processItem_ind(CAMERA,i,_POSE_INERTIAL) }
		catch (kernel::KernelException &e) { if (ncams == 1 && read) keyValueFile.getItem("SENSOR_POSE_INERTIAL", CAMERA_POSE_INERTIAL[0]); else throw e; }
	}
	else
	{
		try { for(int i = 1; i <= ncam; ++i) if (cams[i-1]) KeyValueFile_processItem_ind(CAMERA,i,_POSE_CONSTVEL) }
		catch (kernel::KernelException &e) { if (ncams == 1 && read) keyValueFile.getItem("SENSOR_POSE_CONSTVEL", CAMERA_POSE_CONSTVEL[0]); else throw e; }
	}
	if (!read || intOpts[iGps])
		KeyValueFile_processItem(GPS_POSE);
	KeyValueFile_processItem(ROBOT_POSE);

	if (intOpts[iSimu])
	{
		try {
			for(int i = 1; i <= ncam; ++i)
			{
				if (!cams[i-1]) continue;
				KeyValueFile_processItem_ind(CAMERA,i,_IMG_WIDTH_SIMU);
				KeyValueFile_processItem_ind(CAMERA,i,_IMG_HEIGHT_SIMU);
				KeyValueFile_processItem_ind(CAMERA,i,_INTRINSIC_SIMU);
				KeyValueFile_processItem_ind(CAMERA,i,_DISTORTION_SIMU);
			}
		} catch (kernel::KernelException &e)
		{
			if (read && ncams == 1)
			{
				keyValueFile.getItem("IMG_WIDTH_SIMU", CAMERA_IMG_WIDTH_SIMU[0]);
				keyValueFile.getItem("IMG_HEIGHT_SIMU", CAMERA_IMG_HEIGHT_SIMU[0]);
				keyValueFile.getItem("INTRINSIC_SIMU", CAMERA_INTRINSIC_SIMU[0]);
				keyValueFile.getItem("DISTORTION_SIMU", CAMERA_DISTORTION_SIMU[0]);
			} else throw e;
		}
	} else
	{
		try {
			for(int i = 1; i <= ncam; ++i)
			{
				if (!cams[i-1]) continue;
				KeyValueFile_processItem_ind(CAMERA,i,_TYPE);
				KeyValueFile_processItem_ind_def(CAMERA,i,_FORMAT, "0");
				KeyValueFile_processItem_ind(CAMERA,i,_DEVICE);
				KeyValueFile_processItem_ind(CAMERA,i,_IMG_WIDTH);
				KeyValueFile_processItem_ind(CAMERA,i,_IMG_HEIGHT);
				KeyValueFile_processItem_ind(CAMERA,i,_INTRINSIC);
				KeyValueFile_processItem_ind(CAMERA,i,_DISTORTION);
				if (load_calib) KeyValueFile_processItem_ind(CAMERA,i,_CALIB);
			}
		} catch (kernel::KernelException &e)
		{
			if (read && ncams == 1)
			{
				keyValueFile.getItem("CAMERA_TYPE", CAMERA_TYPE[0]);
				keyValueFile.getItem("CAMERA_FORMAT", CAMERA_FORMAT[0], "0");
				keyValueFile.getItem("CAMERA_DEVICE", CAMERA_DEVICE[0]);
				keyValueFile.getItem("IMG_WIDTH", CAMERA_IMG_WIDTH[0]);
				keyValueFile.getItem("IMG_HEIGHT", CAMERA_IMG_HEIGHT[0]);
				keyValueFile.getItem("INTRINSIC", CAMERA_INTRINSIC[0]);
				keyValueFile.getItem("DISTORTION", CAMERA_DISTORTION[0]);
			} else throw e;
		}
	}

	KeyValueFile_processItem(UNCERT_VLIN);
	KeyValueFile_processItem(UNCERT_VANG);
	KeyValueFile_processItem(PERT_VLIN);
	KeyValueFile_processItem(PERT_VANG);

	if (!read || intOpts[iRobot] == 1 || intOpts[iTrigger] != 0)
		KeyValueFile_processItem(MTI_DEVICE);

	if (!read || intOpts[iRobot] == 1)
	{
		KeyValueFile_processItem(ACCELERO_FULLSCALE);
		KeyValueFile_processItem(ACCELERO_NOISE);
		KeyValueFile_processItem(GYRO_FULLSCALE);
		KeyValueFile_processItem(GYRO_NOISE);

		try { KeyValueFile_processItem(INITIAL_GRAVITY); }
		catch(kernel::KernelException &e) { INITIAL_GRAVITY = 9.806; }
		KeyValueFile_processItem(UNCERT_GRAVITY);
		KeyValueFile_processItem(UNCERT_ABIAS);
		KeyValueFile_processItem(UNCERT_WBIAS);
		KeyValueFile_processItem(PERT_AERR);
		KeyValueFile_processItem(PERT_WERR);
		KeyValueFile_processItem(PERT_RANWALKACC);
		KeyValueFile_processItem(PERT_RANWALKGYRO);
	}

	try { KeyValueFile_processItem(INITIAL_HEADING); }
	catch(kernel::KernelException &e) { INITIAL_HEADING = 0.0; }
	KeyValueFile_processItem(UNCERT_HEADING);
	KeyValueFile_processItem(UNCERT_ATTITUDE);

	if (!read || intOpts[iRobot] == 1)
		KeyValueFile_processItem(IMU_TIMESTAMP_CORRECTION);
	if (!read || intOpts[iGps])
		KeyValueFile_processItem(GPS_TIMESTAMP_CORRECTION);

	if (!read || intOpts[iRobot] == 2)
	{
		KeyValueFile_processItem(dxNDR);
		KeyValueFile_processItem(dvNDR);
	}

	if (!read || intOpts[iRobot] == 2 || intOpts[iOdom] != 0)
		try { KeyValueFile_processItem(ODO_TIMESTAMP_CORRECTION); }
		catch(kernel::KernelException &e) { if (read) keyValueFile.getItem("POS_TIMESTAMP_CORRECTION", ODO_TIMESTAMP_CORRECTION); else throw e; }


	if (!read || intOpts[iOdom] != 0)
		KeyValueFile_processItem(ODO_CALIB);
	if (!read || intOpts[iGps] != 0)
		try { KeyValueFile_processItem(GPS_MAX_CONSIST_SIG); }
		catch(kernel::KernelException &e) { GPS_MAX_CONSIST_SIG = 1e9; }


	if (!read || (intOpts[iRobot] == 1 && intOpts[iSimu]))
	{
		KeyValueFile_processItem(SIMU_IMU_TIMESTAMP_CORRECTION);
		KeyValueFile_processItem(SIMU_IMU_FREQ);
		KeyValueFile_processItem(SIMU_IMU_GRAVITY);
		KeyValueFile_processItem(SIMU_IMU_GYR_BIAS);
		KeyValueFile_processItem(SIMU_IMU_GYR_BIAS_NOISESTD);
		KeyValueFile_processItem(SIMU_IMU_GYR_GAIN);
		KeyValueFile_processItem(SIMU_IMU_GYR_GAIN_NOISESTD);
		KeyValueFile_processItem(SIMU_IMU_RANDWALKGYR_FACTOR);
		KeyValueFile_processItem(SIMU_IMU_ACC_BIAS);
		KeyValueFile_processItem(SIMU_IMU_ACC_BIAS_NOISESTD);
		KeyValueFile_processItem(SIMU_IMU_ACC_GAIN);
		KeyValueFile_processItem(SIMU_IMU_ACC_GAIN_NOISESTD);
		KeyValueFile_processItem(SIMU_IMU_RANDWALKACC_FACTOR);
	}
}


void ConfigEstimation::loadKeyValueFile(jafar::kernel::KeyValueFile const& keyValueFile)
{
  jafar::kernel::KeyValueFile &keyValueFile2 = const_cast<jafar::kernel::KeyValueFile&>(keyValueFile);
  processKeyValueFile(keyValueFile2, true);
}
void ConfigEstimation::saveKeyValueFile(jafar::kernel::KeyValueFile& keyValueFile)
{
  processKeyValueFile(keyValueFile, false);
}

void ConfigEstimation::processKeyValueFile(jafar::kernel::KeyValueFile& keyValueFile, bool read)
{
	KeyValueFile_processItem(CORRECTION_SIZE);

	KeyValueFile_processItem(MAP_SIZE);
	KeyValueFile_processItem(PIX_NOISE);
	KeyValueFile_processItem(PIX_NOISE_SIMUFACTOR);

	KeyValueFile_processItem(D_MIN);
	KeyValueFile_processItem(REPARAM_TH);

	KeyValueFile_processItem(GRID_HCELLS);
	KeyValueFile_processItem(GRID_VCELLS);
	KeyValueFile_processItem(GRID_MARGIN);
	KeyValueFile_processItem(GRID_SEPAR);

	KeyValueFile_processItem(RELEVANCE_TH);
	KeyValueFile_processItem(MAHALANOBIS_TH);
	KeyValueFile_processItem(N_UPDATES_TOTAL);
	KeyValueFile_processItem(N_UPDATES_RANSAC);
	KeyValueFile_processItem(N_INIT);
	KeyValueFile_processItem(N_RECOMP_GAINS);
	KeyValueFile_processItem(RANSAC_LOW_INNOV);

	KeyValueFile_processItem(RANSAC_NTRIES);
	try { KeyValueFile_processItem(MULTIPLE_DEPTH_HYPOS); } catch (kernel::KernelException &e) { if (read) MULTIPLE_DEPTH_HYPOS = false; }

	KeyValueFile_processItem(HARRIS_CONV_SIZE);
	KeyValueFile_processItem(HARRIS_TH);
	KeyValueFile_processItem(HARRIS_EDDGE);

	KeyValueFile_processItem(DESC_SIZE);
	KeyValueFile_processItem(MULTIVIEW_DESCRIPTOR);
	KeyValueFile_processItem(DESC_SCALE_STEP);
	KeyValueFile_processItem(DESC_ANGLE_STEP);
	KeyValueFile_processItem(DESC_PREDICTION_TYPE);

	KeyValueFile_processItem(PATCH_SIZE);
	KeyValueFile_processItem(MAX_SEARCH_SIZE);
	KeyValueFile_processItem(KILL_SEARCH_SIZE);
	KeyValueFile_processItem(MATCH_TH);
	KeyValueFile_processItem(MIN_SCORE);

	try { KeyValueFile_processItem(HI_MATCH_TH) } catch (kernel::KernelException &e) { if (read) HI_MATCH_TH = MATCH_TH; }
	try { KeyValueFile_processItem(HI_LIMIT) } catch (kernel::KernelException &e) { if (read) HI_LIMIT = 100; }

	KeyValueFile_processItem(PARTIAL_POSITION);
}

