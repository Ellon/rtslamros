// We want to use RT-SLAM's typedefs
#include <rtslam/rtSlam.hpp>

// From jafar's kernel we are using...
#include <kernel/threads.hpp> // ...for Variable conditions

// We are using the following components of RT-SLAM
#include <rtslam/worldAbstract.hpp>
#include <rtslam/mapAbstract.hpp>
#include <rtslam/landmarkEuclideanPoint.hpp>
#include <rtslam/landmarkAnchoredHomogeneousPoint.hpp>
#include <rtslam/mapManager.hpp>
#include <rtslam/robotInertial.hpp>
#include <rtslam/sensorAbstract.hpp>
#include <rtslam/observationMakers.hpp>
#include <rtslam/activeSearch.hpp>
#include <rtslam/rawProcessors.hpp>
#include <rtslam/dataManagerOnePointRansac.hpp>
#include <rtslam/sensorManager.hpp>

// From the node, we want to use the following components
#include "rtslamros/hardwareSensorMtiRos.hpp"
#include "rtslamros/hardwareSensorCameraRos.hpp"

// Namespaces being used
using namespace jafar::rtslamros;
using namespace jafar::rtslam;
using namespace jafar;
using namespace boost;

// type definitions
typedef ImagePointObservationMaker<ObservationPinHoleEuclideanPoint, SensorPinhole, LandmarkEuclideanPoint,
	 AppearanceImagePoint, SensorAbstract::PINHOLE, LandmarkAbstract::PNT_EUC> PinholeEucpObservationMaker;
typedef ImagePointObservationMaker<ObservationPinHoleAnchoredHomogeneousPoint, SensorPinhole, LandmarkAnchoredHomogeneousPoint,
	AppearanceImagePoint, SensorAbstract::PINHOLE, LandmarkAbstract::PNT_AH> PinholeAhpObservationMaker;
typedef DataManagerOnePointRansac<RawImage, SensorPinhole, FeatureImagePoint, image::ConvexRoi, ActiveSearchGrid, ImagePointHarrisDetector, ImagePointZnccMatcher> DataManager_ImagePoint_Ransac;

// Global variables needed by multiple functions
world_ptr_t worldPtr; // Pointer to hold the RT-SLAM world
kernel::VariableCondition<int> estimatordata_condition(0); // Used to syncronize estimator thread with this thread
kernel::VariableCondition<int> rawdata_condition(0); // Used to syncronize camera thread with this thread
boost::scoped_ptr<kernel::LoggerTask> loggerTask; // Used to create a thread to log objects
boost::scoped_ptr<kernel::DataLogger> dataLogger; // Jafar's logger
sensor_manager_ptr_t sensorManager;


// General parameters
#define NICENESS 10 ///< \note Value 10 for "niceness" was taken from main.hpp
#define LOG_FILE "/home/emendes/workspace/rtslam-log-test/rtslamros.log" /// \todo Set this path from options
/// \todo Understand and maybe update these grid parameters
#define GRID_ANGULAR_RES 6.77233 ///< \note This was calculated as min_cell_fov from all cameras.
#define GRID_DIST_INIT 0.5
#define GRID_DIST_FACTOR 2.0
#define GRID_N_DIST 5
#define GRID_PHI_FACTOR 1.2
#define TRIGGER_OPTION 0
#define FREQ_OPTION 60.0
#define SHUTTER_OPTION 0.0
#define MTI_BUFFER_SIZE 1024
#define MODE_OPTION rtslam::hardware::mOnline
#define DATA_PATH_OPTION "/home/emendes/workspace/rtslam-log-test/"
#define CAMERA_ID 0
#define CAMERA_BUFFER 500

// Estimation parameters
#define MAP_SIZE 500 ///< \todo Read map size from estimation file
#define REPARAM_TH 0.1
#define KILL_SEARCH_SIZE 100000
#define D_MIN 0.3
#define PATCH_SIZE 11
#define PIX_NOISE 1.0
#define CORRECTION_SIZE 4
#define GRID_HCELLS 6
#define GRID_VCELLS 4
#define GRID_MARGIN 5
#define GRID_SEPAR 7
#define RANSAC_NTRIES 6
#define DESC_SIZE 21
#define HARRIS_CONV_SIZE 5
#define HARRIS_TH 5.0
#define HARRIS_EDDGE 1.4
#define MIN_SCORE 0.70
#define PARTIAL_POSITION 0.25
#define MAX_SEARCH_SIZE 10000
#define RANSAC_LOW_INNOV 1.0
#define MATCH_TH 0.75
#define HI_MATCH_TH 0.85
#define HI_LIMIT 100
#define MAHALANOBIS_TH 3.0
#define RELEVANCE_TH 2.0
#define N_UPDATES_TOTAL 25
#define N_UPDATES_RANSAC 22
#define N_INIT 10
#define N_RECOMP_GAINS 2
#define MULTIPLE_DEPTH_HYPOS false ///< \note MULTIPLE_DEPTH_HYPOS was set in an exeption catch on main.hpp



// Setup parameters
#define UNCERT_VLIN 0.0
#define UNCERT_ABIAS 0.01
#define UNCERT_WBIAS 0.01
#define UNCERT_GRAVITY 0.01
#define ACCELERO_FULLSCALE 17.0
#define GYRO_FULLSCALE 5.23599
#define INITIAL_GRAVITY 9.806 ///< \note Initial gravity was set in an exeption catch on main.hpp
#define PERT_AERR 1.0
#define PERT_WERR 1.0
#define ACCELERO_NOISE 0.0109545
#define GYRO_NOISE 0.00772691
#define PERT_RANWALKACC 0.0
#define PERT_RANWALKGYRO 0.0
#define MTI_DEVICE "" ///< \todo Not used in HardwareSensorMtiRos. Could be the topic name to read IMU data
#define IMU_TIMESTAMP_CORRECTION 0.0
#define ROBOT_POSE {0,0,-0.647,0,0,0} ///< \todo Set robot pose tf
#define INITIAL_HEADING 0
#define UNCERT_ATTITUDE 0.05
#define UNCERT_HEADING 0.1
#define CAMERA_POSE {0.292,0.193,0.108,-90,0,-90} ///< \todo Set camera pose from setup file
#define CAMERA_INTRINSIC {+3.2703920036560964e+02,+2.4714638294268735e+02,+3.7290284087776149e+02,+3.7271088432359716e+02} ///< \todo Set camera intrinsics from tf
#define CAMERA_DISTORTION {-2.6188796889984112e-01,+1.2158236902559119e-01,-3.0273706315773623e-02} ///< \todo Set camera distortion from tf
#define IMG_WIDTH 640 ///< \todo Set image width from tf
#define IMG_HEIGHT 480 ///< \todo Set image height from tf

bool demo_slam_simple_init()
{ JFR_GLOBAL_TRY

	/// ---------------------------------------------------------------------------
	/// --- INIT LOGGER -----------------------------------------------------------
	/// ---------------------------------------------------------------------------
	loggerTask.reset(new kernel::LoggerTask(NICENESS));
	dataLogger.reset(new kernel::DataLogger(LOG_FILE));
	dataLogger->setLoggerTask(loggerTask.get());
	dataLogger->writeCurrentDate();
	dataLogger->writeNewLine();

	/// ---------------------------------------------------------------------------
	/// --- INIT WORLD ------------------------------------------------------------
	/// ---------------------------------------------------------------------------
	worldPtr.reset(new WorldAbstract());

	/// ---------------------------------------------------------------------------
	/// --- INIT MAPS -------------------------------------------------------------
	/// ---------------------------------------------------------------------------

	// Create map
	map_ptr_t mapPtr(new MapAbstract(MAP_SIZE));
	mapPtr->linkToParentWorld(worldPtr);

	// Create map manager.
	landmark_factory_ptr_t pointLmkFactory(new LandmarkFactory<LandmarkAnchoredHomogeneousPoint, LandmarkEuclideanPoint>());

	const double gridAngularRes = GRID_ANGULAR_RES;
	const double gridDistInit = GRID_DIST_INIT;
	const double gridDistFactor = GRID_DIST_FACTOR;
	const int gridNDist = GRID_N_DIST;
	const double gridPhiFactor = GRID_PHI_FACTOR;
	map_manager_ptr_t mmPoint(new MapManagerOdometry(pointLmkFactory,
													 REPARAM_TH, KILL_SEARCH_SIZE,
													 gridAngularRes, gridDistInit, gridDistFactor, gridNDist, gridPhiFactor));
	mmPoint->linkToParentMap(mapPtr);

	/// ---------------------------------------------------------------------------
	/// --- CREATE ROBOT ----------------------------------------------------------
	/// ---------------------------------------------------------------------------

	robinertial_ptr_t robPtr1(new RobotInertial(mapPtr));

	robPtr1->setInitialStd(UNCERT_VLIN,
						   UNCERT_ABIAS*ACCELERO_FULLSCALE,
						   UNCERT_WBIAS*GYRO_FULLSCALE,
						   UNCERT_GRAVITY*INITIAL_GRAVITY);

	robPtr1->setInitialParams(INITIAL_GRAVITY);
	double aerr = PERT_AERR * ACCELERO_NOISE;
	double werr = PERT_WERR * GYRO_NOISE;
	double _v12[12] = {
		aerr, aerr, aerr, werr, werr, werr,
		PERT_RANWALKACC, PERT_RANWALKACC, PERT_RANWALKACC,
		PERT_RANWALKACC, PERT_RANWALKACC, PERT_RANWALKACC};
	vec pertStd = createVector<12>(_v12);
	robPtr1->perturbation.set_std_continuous(pertStd);

	boost::shared_ptr<rtslamros::hardware::HardwareSensorMtiRos> hardEst1(new rtslamros::hardware::HardwareSensorMtiRos(&estimatordata_condition,
																														MTI_DEVICE,
																														TRIGGER_OPTION, FREQ_OPTION, SHUTTER_OPTION, ///< \todo Verify if trigger, freq and shutter are important when reading data from ROS topics.
																														MTI_BUFFER_SIZE, MODE_OPTION,
																														DATA_PATH_OPTION,
																														loggerTask.get()));
	hardEst1->setSyncConfig(IMU_TIMESTAMP_CORRECTION);
	robPtr1->setHardwareEstimator(hardEst1);

	robPtr1->linkToParentMap(mapPtr);
	double _vrobot_pose[6] = ROBOT_POSE;
	robPtr1->setRobotPose( createVector<6>(_vrobot_pose), true);
	robPtr1->setOrientationStd(0,0,INITIAL_HEADING,
							   UNCERT_ATTITUDE,UNCERT_ATTITUDE,UNCERT_HEADING,
							   false);

	if (dataLogger) dataLogger->addLoggable(*robPtr1.get()); /// \warning This line creates a segmentation fault when destroying the robot...

	/// ---------------------------------------------------------------------------
	/// --- INIT SENSORS ----------------------------------------------------------
	/// ---------------------------------------------------------------------------

	boost::shared_ptr<ObservationFactory> obsFact(new ObservationFactory());
	obsFact->addMaker(boost::shared_ptr<ObservationMakerAbstract>(new PinholeEucpObservationMaker(D_MIN,PATCH_SIZE)));
	obsFact->addMaker(boost::shared_ptr<ObservationMakerAbstract>(new PinholeAhpObservationMaker(D_MIN,PATCH_SIZE)));

	// a. Create camera
	pinhole_ptr_t senPtr11(new SensorPinhole(robPtr1, MapObject::UNFILTERED));
	senPtr11->linkToParentRobot(robPtr1);
	senPtr11->name("cam");
	double _vcamera_pose[6] = CAMERA_POSE; ///< \todo Set camera pose from setup file
	jblas::vec camera_pose = createVector<6>(_vcamera_pose);
	senPtr11->setPose(camera_pose(0), camera_pose(1), camera_pose(2), camera_pose(3), camera_pose(4), camera_pose(5)); // x,y,z,roll,pitch,yaw

	double _vintrinsic[4] = CAMERA_INTRINSIC;
	double _vdistortion[3] = CAMERA_DISTORTION;
	senPtr11->params.setIntrinsicCalibration(IMG_WIDTH, IMG_HEIGHT, createVector<4>(_vintrinsic), createVector<3>(_vdistortion), CORRECTION_SIZE);
	senPtr11->params.setMiscellaneous(PIX_NOISE, D_MIN);

	if (dataLogger) dataLogger->addLoggable(*senPtr11.get());

	senPtr11->setIntegrationPolicy(false);
	senPtr11->setUseForInit(false);
	senPtr11->setNeedInit(true); // for auto exposure

	// b. Create data manager.
	boost::shared_ptr<ActiveSearchGrid> asGrid(new ActiveSearchGrid(IMG_WIDTH, IMG_HEIGHT, GRID_HCELLS, GRID_VCELLS, GRID_MARGIN, GRID_SEPAR));
	boost::shared_ptr<DescriptorFactoryAbstract> pointDescFactory(new DescriptorImagePointFirstViewFactory(DESC_SIZE));
	boost::shared_ptr<ImagePointHarrisDetector> harrisDetector(new ImagePointHarrisDetector(HARRIS_CONV_SIZE, HARRIS_TH, HARRIS_EDDGE, PATCH_SIZE, PIX_NOISE, pointDescFactory));
	boost::shared_ptr<ImagePointZnccMatcher> znccMatcher(new ImagePointZnccMatcher(MIN_SCORE, PARTIAL_POSITION, PATCH_SIZE, MAX_SEARCH_SIZE, RANSAC_LOW_INNOV, MATCH_TH, HI_MATCH_TH, HI_LIMIT, MAHALANOBIS_TH, RELEVANCE_TH, PIX_NOISE));
	boost::shared_ptr<DataManager_ImagePoint_Ransac> dmPt11(new DataManager_ImagePoint_Ransac(harrisDetector, znccMatcher, asGrid, N_UPDATES_TOTAL, N_UPDATES_RANSAC, RANSAC_NTRIES, N_INIT, N_RECOMP_GAINS, MULTIPLE_DEPTH_HYPOS, loggerTask.get()));

	dmPt11->linkToParentSensorSpec(senPtr11);
	dmPt11->linkToParentMapManager(mmPoint);
	dmPt11->setObservationFactory(obsFact);
	if (dataLogger) dataLogger->addLoggable(*dmPt11.get());

	// Create hardware for the camera
	rtslamros::hardware::hardware_sensor_camera_ros_ptr_t hardSen11(new rtslamros::hardware::HardwareSensorCameraRos(&rawdata_condition, MODE_OPTION, CAMERA_ID, cv::Size(IMG_WIDTH,IMG_HEIGHT),CAMERA_BUFFER,loggerTask.get(),DATA_PATH_OPTION));
	senPtr11->setHardwareSensor(hardSen11);

	// Create sensor manager
	sensorManager.reset(new SensorManagerOnline(mapPtr, DATA_PATH_OPTION, loggerTask.get()));

	return true;
	JFR_GLOBAL_CATCH
} // demo_slam_simple_init


void demo_slam_simple_stop(world_ptr_t *world)
{
	// stop all sensors
	map_ptr_t mapPtr = (*world)->mapList().front();
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

	if (loggerTask)
	{
		std::cout << "Stopping and joining logger..."; std::cout.flush();
		loggerTask->stop(true);
		std::cout << " OK." << std::endl;
	}

	/// \note Need to start hardware first (to launch their threads) before uncommenting this code
//	for (MapAbstract::RobotList::iterator robIter = mapPtr->robotList().begin();
//		robIter != mapPtr->robotList().end(); ++robIter)
//	{
//		if ((*robIter)->hardwareEstimatorPtr)
//		{
//			std::cout << "Joining robot " << (*robIter)->id() << " estimator..."; std::cout.flush();
//			(*robIter)->hardwareEstimatorPtr->join();
//			std::cout << " OK." << std::endl;
//		}
//		for (RobotAbstract::SensorList::iterator senIter = (*robIter)->sensorList().begin();
//			senIter != (*robIter)->sensorList().end(); ++senIter)
//		{
//			std::cout << "Joining sensor " << (*senIter)->id() << " " << (*senIter)->name() << "..."; std::cout.flush();
//			if ((*senIter)->join(1000)) std::cout << " OK." << std::endl; else std::cout << " FAILED." << std::endl;
//		}
//	}

} // demo_slam_simple_stop
