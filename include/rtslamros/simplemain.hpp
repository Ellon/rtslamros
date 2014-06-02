#include <signal.h>

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
#include <rtslam/observationMakers.hpp>
#include <rtslam/activeSearch.hpp>
#include <rtslam/rawProcessors.hpp>
#include <rtslam/dataManagerOnePointRansac.hpp>
#include <rtslam/sensorManager.hpp>

// To visualize the estimation, we'll use the following RT-SLAM displays
#include <rtslam/display_qt.hpp>
#include <rtslam/display_gdhe.hpp>

// From the node, we want to use the following components
#include "rtslamros/hardwareSensorMtiRos.hpp"
#include "rtslamros/hardwareSensorCameraRos.hpp"

// To parse options from config file and command line
#include "rtslamros/option_parser.hpp"

// To broadcast the estimated robot pose we need a tf broadcaster
#include <tf/transform_broadcaster.h>

// Namespaces being used
using namespace jafar::rtslamros;
using namespace jafar::rtslam;
using namespace jafar;

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
bool ready = false; // Variable that indicates if the extrapolation was initialized at least once after the robot moves
boost::scoped_ptr<tf::TransformBroadcaster> tfBroadcasterPtr; // Used to broadcast robot estimated pose
jafar::rtslam::hardware::Mode mode; // Used to set the hardware properly

#ifdef HAVE_MODULE_QDISPLAY
display::ViewerQt *viewerQt = NULL;
#endif

#ifdef HAVE_MODULE_GDHE
display::ViewerGdhe *viewerGdhe = NULL;
#endif


// General parameters
#define NICENESS 10 ///< \note Value 10 for "niceness" was taken from main.hpp
#define DISPLAY_PERIOD 100 // ms
/// \todo Understand and maybe update these grid parameters
#define GRID_ANGULAR_RES 6.77233 ///< \note This was calculated as min_cell_fov from all cameras.
#define GRID_DIST_INIT 0.5
#define GRID_DIST_FACTOR 2.0
#define GRID_N_DIST 5
#define GRID_PHI_FACTOR 1.2
#define TRIGGER_OPTION 0
#define FREQ_OPTION 15.0
#define SHUTTER_OPTION 0.0
#define MTI_BUFFER_SIZE 1024
#define CAMERA_ID 1
#define CAMERA_BUFFER 500

/// \note MULTIPLE_DEPTH_HYPOS was set in an exeption catch on main.hpp
/// \todo Set initial camera pose from tf topic
/// \todo Set initial robot pose from tf topic
/// \todo Set image width from camera topic
/// \todo Set image height from camera topic
/// \todo Set camera intrinsics from camera topic
/// \todo Set camera distortion from camera topic
/// \todo Set the MTI device to be the MTI topic name
/// \note Initial gravity was set in an exeption catch on main.hpp

//distortion_model: plumb_bob
//D: [-0.409016, 0.197327, -0.0051849999999999995, -0.0048839999999999995, 0.0]
//K: [300.3122405,   0.0,       161.209259,
//      0.0,       299.5597015, 123.44138,
//      0.0,         0.0,         1.0]
//R: [1.0, 0.0, 0.0,
//    0.0, 1.0, 0.0,
//    0.0, 0.0, 1.0]
//P: [262.5751955,   0.0,      159.216994, 0.0,
//      0.0,       278.414032, 122.720959, 0.0,
//      0.0,         0.0,        1.0,      0.0]

bool demo_slam_simple_init()
{ JFR_GLOBAL_TRY

	// Set the mode based on the options.
	/// \todo I feel a mixing between mode and replay/dump variables... it should be clarified and maybe the mode could be set directly by the user, and the program would stop if the combinations of modes are not allowed.
	/// \warning I have no idea if the lines below are correct
	if(rtslamoptions::replay == rtslamoptions::rOffline || rtslamoptions::replay == rtslamoptions::rOfflineReplay) mode = jafar::rtslam::hardware::mOffline;
	else{
		if(rtslamoptions::dump) mode = jafar::rtslam::hardware::mOnlineDump;
		else mode = jafar::rtslam::hardware::mOnline;
	}

	// Unset to be used later on main
	ready = false;

	/// ---------------------------------------------------------------------------
	/// --- INIT LOGGER -----------------------------------------------------------
	/// ---------------------------------------------------------------------------
	if (!rtslamoptions::logfile.empty() || rtslamoptions::dump)
		loggerTask.reset(new kernel::LoggerTask(NICENESS));

	if (!rtslamoptions::logfile.empty()) {
		dataLogger.reset(new kernel::DataLogger(rtslamoptions::logfile));
		dataLogger->setLoggerTask(loggerTask.get());
		dataLogger->writeCurrentDate();
		dataLogger->writeNewLine();
	}

	/// ---------------------------------------------------------------------------
	/// --- INIT WORLD ------------------------------------------------------------
	/// ---------------------------------------------------------------------------
	worldPtr.reset(new WorldAbstract());

	/// ---------------------------------------------------------------------------
	/// --- INIT MAPS -------------------------------------------------------------
	/// ---------------------------------------------------------------------------

	// Create map
	map_ptr_t mapPtr(new MapAbstract(configEstimation.MAP_SIZE));
	mapPtr->linkToParentWorld(worldPtr);

	// Create map manager.
	landmark_factory_ptr_t pointLmkFactory(new LandmarkFactory<LandmarkAnchoredHomogeneousPoint, LandmarkEuclideanPoint>());

	const double gridAngularRes = GRID_ANGULAR_RES;
	const double gridDistInit = GRID_DIST_INIT;
	const double gridDistFactor = GRID_DIST_FACTOR;
	const int gridNDist = GRID_N_DIST;
	const double gridPhiFactor = GRID_PHI_FACTOR;
	map_manager_ptr_t mmPoint(new MapManagerOdometry(pointLmkFactory,
													 configEstimation.REPARAM_TH, configEstimation.KILL_SEARCH_SIZE,
													 gridAngularRes, gridDistInit, gridDistFactor, gridNDist, gridPhiFactor));
	mmPoint->linkToParentMap(mapPtr);

	/// ---------------------------------------------------------------------------
	/// --- CREATE ROBOT ----------------------------------------------------------
	/// ---------------------------------------------------------------------------

	robinertial_ptr_t robPtr1(new RobotInertial(mapPtr));

	robPtr1->setInitialStd(configSetup.UNCERT_VLIN,
						   configSetup.UNCERT_ABIAS*configSetup.ACCELERO_FULLSCALE,
						   configSetup.UNCERT_WBIAS*configSetup.GYRO_FULLSCALE,
						   configSetup.UNCERT_GRAVITY*configSetup.INITIAL_GRAVITY);

	robPtr1->setInitialParams(configSetup.INITIAL_GRAVITY);
	double aerr = configSetup.PERT_AERR * configSetup.ACCELERO_NOISE;
	double werr = configSetup.PERT_WERR * configSetup.GYRO_NOISE;
	double _v12[12] = {
		aerr, aerr, aerr, werr, werr, werr,
		configSetup.PERT_RANWALKACC, configSetup.PERT_RANWALKACC, configSetup.PERT_RANWALKACC,
		configSetup.PERT_RANWALKACC, configSetup.PERT_RANWALKACC, configSetup.PERT_RANWALKACC};
	vec pertStd = createVector<12>(_v12);
	robPtr1->perturbation.set_std_continuous(pertStd);

	boost::shared_ptr<rtslamros::hardware::HardwareSensorMtiRos> hardEst1(new rtslamros::hardware::HardwareSensorMtiRos(&estimatordata_condition,
																														TRIGGER_OPTION, FREQ_OPTION, SHUTTER_OPTION, ///< \todo Verify if trigger, freq and shutter are important when reading data from ROS topics.
																														MTI_BUFFER_SIZE, mode, rtslamoptions::datapath,
																														loggerTask.get()));
	hardEst1->setSyncConfig(configSetup.IMU_TIMESTAMP_CORRECTION);
	robPtr1->setHardwareEstimator(hardEst1);

	robPtr1->linkToParentMap(mapPtr);
	robPtr1->setRobotPose( configSetup.ROBOT_POSE, true);
	robPtr1->setOrientationStd(0,0,configSetup.INITIAL_HEADING,
							   configSetup.UNCERT_ATTITUDE,configSetup.UNCERT_ATTITUDE,configSetup.UNCERT_HEADING,
							   false);

	if (dataLogger) dataLogger->addLoggable(*robPtr1.get()); /// \warning This line creates a segmentation fault when destroying the robot...

	/// ---------------------------------------------------------------------------
	/// --- INIT SENSORS ----------------------------------------------------------
	/// ---------------------------------------------------------------------------

	boost::shared_ptr<ObservationFactory> obsFact(new ObservationFactory());
	obsFact->addMaker(boost::shared_ptr<ObservationMakerAbstract>(new PinholeEucpObservationMaker(configEstimation.D_MIN,configEstimation.PATCH_SIZE)));
	obsFact->addMaker(boost::shared_ptr<ObservationMakerAbstract>(new PinholeAhpObservationMaker(configEstimation.D_MIN,configEstimation.PATCH_SIZE)));

	// a. Create camera
	pinhole_ptr_t senPtr11(new SensorPinhole(robPtr1, MapObject::UNFILTERED));
	senPtr11->linkToParentRobot(robPtr1);
	senPtr11->name("cam");
	senPtr11->setPose(configSetup.CAMERA_POSE(0), configSetup.CAMERA_POSE(1), configSetup.CAMERA_POSE(2), configSetup.CAMERA_POSE(3), configSetup.CAMERA_POSE(4), configSetup.CAMERA_POSE(5)); // x,y,z,roll,pitch,yaw

	senPtr11->params.setIntrinsicCalibration(configSetup.CAMERA_IMG_WIDTH, configSetup.CAMERA_IMG_HEIGHT, configSetup.CAMERA_INTRINSIC, configSetup.CAMERA_DISTORTION, configEstimation.CORRECTION_SIZE);
	senPtr11->params.setMiscellaneous(configEstimation.PIX_NOISE, configEstimation.D_MIN);

	if (dataLogger) dataLogger->addLoggable(*senPtr11.get());

	senPtr11->setIntegrationPolicy(false);
	senPtr11->setUseForInit(false);
	senPtr11->setNeedInit(true); // for auto exposure

	// b. Create data manager.
	boost::shared_ptr<ActiveSearchGrid> asGrid(new ActiveSearchGrid(configSetup.CAMERA_IMG_WIDTH, configSetup.CAMERA_IMG_HEIGHT, configEstimation.GRID_HCELLS, configEstimation.GRID_VCELLS, configEstimation.GRID_MARGIN, configEstimation.GRID_SEPAR));
	boost::shared_ptr<DescriptorFactoryAbstract> pointDescFactory(new DescriptorImagePointFirstViewFactory(configEstimation.DESC_SIZE));
	boost::shared_ptr<ImagePointHarrisDetector> harrisDetector(new ImagePointHarrisDetector(configEstimation.HARRIS_CONV_SIZE, configEstimation.HARRIS_TH, configEstimation.HARRIS_EDDGE, configEstimation.PATCH_SIZE, configEstimation.PIX_NOISE, pointDescFactory));
	boost::shared_ptr<ImagePointZnccMatcher> znccMatcher(new ImagePointZnccMatcher(configEstimation.MIN_SCORE, configEstimation.PARTIAL_POSITION, configEstimation.PATCH_SIZE, configEstimation.MAX_SEARCH_SIZE, configEstimation.RANSAC_LOW_INNOV, configEstimation.MATCH_TH, configEstimation.HI_MATCH_TH, configEstimation.HI_LIMIT, configEstimation.MAHALANOBIS_TH, configEstimation.RELEVANCE_TH, configEstimation.PIX_NOISE));
	boost::shared_ptr<DataManager_ImagePoint_Ransac> dmPt11(new DataManager_ImagePoint_Ransac(harrisDetector, znccMatcher, asGrid, configEstimation.N_UPDATES_TOTAL, configEstimation.N_UPDATES_RANSAC, configEstimation.RANSAC_NTRIES, configEstimation.N_INIT, configEstimation.N_RECOMP_GAINS, configEstimation.MULTIPLE_DEPTH_HYPOS, loggerTask.get()));

	dmPt11->linkToParentSensorSpec(senPtr11);
	dmPt11->linkToParentMapManager(mmPoint);
	dmPt11->setObservationFactory(obsFact);
	if (dataLogger) dataLogger->addLoggable(*dmPt11.get());

	// Create hardware for the camera
	rtslamros::hardware::hardware_sensor_camera_ros_ptr_t hardSen11(new rtslamros::hardware::HardwareSensorCameraRos(&rawdata_condition, mode, CAMERA_ID, cv::Size(configSetup.CAMERA_IMG_WIDTH,configSetup.CAMERA_IMG_HEIGHT),CAMERA_BUFFER,loggerTask.get(),rtslamoptions::datapath));
	hardSen11->setTimingInfos(1.0/hardSen11->getFreq(), 1.0/hardSen11->getFreq());
	senPtr11->setHardwareSensor(hardSen11);

	// Create sensor manager
	// Obs: Offline has no logger task;
	if(rtslamoptions::replay == rtslamoptions::rOffline)
		sensorManager.reset(new SensorManagerOffline(mapPtr));
	else if (rtslamoptions::replay == rtslamoptions::rOfflineReplay)
		sensorManager.reset(new SensorManagerOffline(mapPtr, rtslamoptions::datapath)); // data path to replay data offline
	else if(rtslamoptions::dump)
		sensorManager.reset(new SensorManagerOnline(mapPtr, rtslamoptions::datapath, loggerTask.get())); // datapath to dump
	else
		sensorManager.reset(new SensorManagerOnline(mapPtr, "", loggerTask.get())); // empty datapath to disable dumping.

	// Initialize the tf broadcaster
	tfBroadcasterPtr.reset(new tf::TransformBroadcaster());

	/// ---------------------------------------------------------------------------
	/// --- INIT DISPLAYS ---------------------------------------------------------
	/// ---------------------------------------------------------------------------
	/// \note I saw no reason for creating the display at the beginning of this function and then forcing the first display in the end (the way it's done on main.hpp), so I put everything here.
#ifdef HAVE_MODULE_QDISPLAY
	if (rtslamoptions::dispQt)
	{
		display::ViewerQt *viewerQt = new display::ViewerQt(8, configEstimation.MAHALANOBIS_TH);
		worldPtr->addDisplayViewer(viewerQt, display::ViewerQt::id());

		// force a first display with empty slam to ensure that all windows are loaded
		viewerQt = PTR_CAST<display::ViewerQt*> (worldPtr->getDisplayViewer(display::ViewerQt::id()));
		viewerQt->bufferize(worldPtr);

		// initializing stuff for controlling run/pause from viewer
		boost::unique_lock<boost::mutex> runStatus_lock(viewerQt->runStatus.mutex);
		viewerQt->runStatus.pause = rtslamoptions::pause;
		viewerQt->runStatus.render_all = rtslamoptions::renderall;
		runStatus_lock.unlock();
	}
#endif
#ifdef HAVE_MODULE_GDHE
	if (rtslamoptions::dispGdhe)
	{
		display::ViewerGdhe *viewerGdhe = new display::ViewerGdhe("camera", configEstimation.MAHALANOBIS_TH);
		worldPtr->addDisplayViewer(viewerGdhe, display::ViewerGdhe::id());

		// force a first display with empty slam to ensure that all windows are loaded
		viewerGdhe = PTR_CAST<display::ViewerGdhe*> (worldPtr->getDisplayViewer(display::ViewerGdhe::id()));
		viewerGdhe->bufferize(worldPtr);
	}
#endif

	return true;
	JFR_GLOBAL_CATCH
} // demo_slam_simple_init


void demo_slam_simple_exit(world_ptr_t *world, boost::thread *thread_main) {
	(*world)->exit(true);
	(*world)->display_condition.notify_all();

#ifdef HAVE_MODULE_QDISPLAY
	if (rtslamoptions::dispQt)
	{
		viewerQt->runStatus.pause = 0;
		viewerQt->runStatus.condition.notify_all();
	}
#endif

	thread_main->join();
} // demo_slam_simple_exit

void demo_slam_simple_stop(world_ptr_t *world)
{
	// stop all sensors
	map_ptr_t mapPtr = (*world)->mapList().front();
	ready = false; ///< \note Is it needed here? Maybe to avoid exporting after stop
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
	// Join estimator and sensor threads
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

} // demo_slam_simple_stop

/** Sets several signals to the same catcher function
  */
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
} // set_signals

/** Function to stop properly RT-SLAM from a signal catch
  *
  * Useful for example when working online and stoping
  * with a Ctrl-C. Signals to be catch should be set with
  * a signal(SIGNAL_TYPE, this function).
  */
void signal_catcher(int sig __attribute__((unused)))
{
	if (worldPtr->error == eNoError) worldPtr->error = eCrashed;
	// This boolean mark the first time we entered this function. It's needed in the case the stop procedure gets blocked
	// when called the first time. In this case it will not try again a second time.
	static bool first = true;
	if (first)
	{
		first = false;
		std::cerr << "RT-SLAM is stopping because it received signal " << sig << " \"" << strsignal(sig) << "\"" << std::endl;
		demo_slam_simple_stop(&worldPtr);

		// force deleting sensors object to call their destructor
		map_ptr_t mapPtr = worldPtr->mapList().front();
		for (MapAbstract::RobotList::iterator robIter = mapPtr->robotList().begin();
			robIter != mapPtr->robotList().end(); ++robIter)
		{
			for (RobotAbstract::SensorList::iterator senIter = (*robIter)->sensorList().begin();
				senIter != (*robIter)->sensorList().end(); ++senIter)
				delete (*senIter).get();
//			if ((*robIter)->hardwareEstimatorPtr.get() != trigger.get()) delete (*robIter).get();
		}
//		delete trigger.get();
	}
	else std::cerr << "RT-SLAM failed to stop because it received signal " << sig << " \"" << strsignal(sig) << "\"" << std::endl;

	signal(sig, SIG_DFL);
	raise(sig);
} // signal_catcher

void demo_slam_simple_main(world_ptr_t *world)
{ JFR_GLOBAL_TRY

	// Declare pointers to be used in the function
	map_ptr_t mapPtr = (*world)->mapList().front(); // We only have one map
	robot_ptr_t robotPtr = mapPtr->robotList().front(); // We only have one robot

	// Set the signal catcher. Allows proper finalization of RT-SLAM in a event
	// of a signal being raised (like a stop by interruption with Ctrl-C).
	set_signals(signal_catcher);

	// Start hardware sensors that need long init
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
	}

	// start other hardware sensors that doesn't need initialization
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

	// Set the start date
	double start_date = kernel::Clock::getTime();
	{
		// Save the start data in a log file
		std::fstream f((std::string(rtslamoptions::datapath) + std::string("/sdate.log")).c_str(), std::ios_base::out);
		f << std::setprecision(19) << start_date << std::endl;
		f.close();
	}
	// Set the start date in the sensor manager
	sensorManager->setStartDate(start_date);


	// ---------------------------------------------------------------------------
	// --- LOOP ------------------------------------------------------------------
	// ---------------------------------------------------------------------------
	// INIT : complete observations set
	// loop all sensors
	// loop all lmks
	// create sen--lmk observation
	// Temporal loop
	// if (dataLogger) dataLogger->log();
	double filterTime = 0.0;
	kernel::Chrono chrono;

	// Main loop
	while (!(*world)->exit())
	{
		bool had_data = false;
		chrono.reset();

		// Get next data from the camera
		SensorManagerAbstract::ProcessInfo pinfo = sensorManager->getNextDataToUse(filterTime);
		bool no_more_data = pinfo.no_more_data;

		if (pinfo.sen)
		{
			had_data = true;

			// Get the robot that owns the sensor. Normally not needed because we only have one robot in this demo
			robot_ptr_t robPtr = pinfo.sen->robotPtr();

			double newt = pinfo.date;

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
			while (!robPtr->move(newt)) // Acumulates the estimator data until after the time the data from the camera arrived
			{
				// This block waits for more data to arrive to the IMU and measures the time spent waiting for new data
				if (!waited) wait_time = kernel::Clock::getTime();
				waited = true;
				if (robPtr->hardwareEstimatorPtr->stopped()) { stop = true; break; }
				estimatordata_condition.wait(boost::lambda::_1 != 0);
				estimatordata_condition.set(0);
				waitedmove_date = kernel::Clock::getTime();
			}
			double moved_date = kernel::Clock::getTime();
			if (stop) // Stop if there was no IMU data
			{
				std::cout << "No more estimator data, stopping." << std::endl;
				break;
			}
			if (waited) // If had to wait for the IMU, print the wa(i|s)ted time
			{
				wait_time = kernel::Clock::getTime() - wait_time;
				/*if (wait_time > 0.001)*/ std::cout << "wa(i|s)ted " << wait_time << " for estimator data" << std::endl;
			}

			if (!ready && sensorManager->allInit())
			{ // here to ensure that at least one move has been done (to init estimator)
				robPtr->reinit_extrapolate();
				ready = true;
			}

			// Process data from the camera until the time the next date will arrive.
			pinfo.sen->process(pinfo.id, pinfo.date_next);

			// Set which sensor was updated last
			pinfo.sen->robotPtr()->last_updated = pinfo.sen;

			robPtr->reinit_extrapolate();
			double processed_date = kernel::Clock::getTime();

			sensorManager->logData(pinfo.sen, start_date, waitedmove_date, moved_date, processed_date);
			filterTime = robPtr->self_time;

			// Broadcast current estimation on /tf topic
			tf::Transform transform;
			// NOTE: The robot pose is represented inside RT-SLAM in the following order: [x y z qw qx qy qz]
			transform.setOrigin( tf::Vector3(robPtr->state.x(0), robPtr->state.x(1), robPtr->state.x(2)) );
			transform.setRotation( tf::Quaternion(robPtr->state.x(4), robPtr->state.x(5), robPtr->state.x(6), robPtr->state.x(3)) );
			tfBroadcasterPtr->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "uav0/rtslam"));
		}

		// Wait that display has finished if we're rendering all frames
		if (had_data)
		{
			// First we need to get the "render all" status (it may have changed through the qtdisplay
			bool renderall_status;
#ifdef HAVE_MODULE_QDISPLAY
			// If we're using a qt display, get the current status from it because the user may have changed it at runtime
			if (rtslamoptions::dispQt)
			{
				boost::unique_lock<boost::mutex> runStatus_lock(viewerQt->runStatus.mutex);
				renderall_status = viewerQt->runStatus.render_all;
				runStatus_lock.unlock();
			} else
#endif
			// If not, get the renderall option set at the beginning
			renderall_status = rtslamoptions::renderall;

			// If we are rendering all, wait for the display to finish the rendering.
			if ((rtslamoptions::dispQt || rtslamoptions::dispGdhe) && renderall_status)
			{
				boost::unique_lock<boost::mutex> display_lock((*world)->display_mutex);
				while(!(*world)->display_rendered && !(*world)->exit()) (*world)->display_condition.wait(display_lock);
				display_lock.unlock();
			}
		}

		// Bufferizes the world to be displayed if the displays had finished rendering the last buffered world.
		// First get the world time of the last processed data and check if it's newer than the last display time. If it is, that means we have new things to display
		unsigned processed_t = (had_data ? (*world)->t : (*world)->t-1);
		if ((*world)->display_t+1 < processed_t+1)
		{
			boost::unique_lock<boost::mutex> display_lock((*world)->display_mutex);
			// Verifies if the last buffered world was already rendered. If not we do nothing.
			if ((*world)->display_rendered)
			{
#ifdef HAVE_MODULE_QDISPLAY
				// If we're using a qt display, bufferize the world on in
				display::ViewerQt *viewerQt = NULL; ///< \note Why do we declare another pointer here if we already have the display stored in a global pointer?
				if (rtslamoptions::dispQt) viewerQt = PTR_CAST<display::ViewerQt*> ((*world)->getDisplayViewer(display::ViewerQt::id()));
				if (rtslamoptions::dispQt) viewerQt->bufferize(*world);
#endif
#ifdef HAVE_MODULE_GDHE
				// If we're using a GDHE display, bufferize the world on in
				display::ViewerGdhe *viewerGdhe = NULL; ///< \note Why do we declare another pointer here if we already have the display stored in a global pointer?
				if (rtslamoptions::dispGdhe) viewerGdhe = PTR_CAST<display::ViewerGdhe*> ((*world)->getDisplayViewer(display::ViewerGdhe::id()));
				if (rtslamoptions::dispGdhe) viewerGdhe->bufferize(*world);
#endif
				// Set the display time for the time of the last buffered world, clear rendered flag and notify display threads that may be waiting.
				(*world)->display_t = (*world)->t;
				(*world)->display_rendered = false;
				display_lock.unlock();
				(*world)->display_condition.notify_all();
			} else
				display_lock.unlock();
		}


		// Break the main loop if we have no more data
		if (no_more_data) break;

		// If we didn't have data in this loop, set this thread to wait for more data from the camera
		if (!had_data)
		{
			if (pinfo.date_next < 0) rawdata_condition.wait(boost::lambda::_1 != 0); else
				rawdata_condition.timed_wait(boost::lambda::_1 != 0, boost::posix_time::microseconds((pinfo.date_next-kernel::Clock::getTime())*1e6));
		}
		rawdata_condition.set(0);

		// Pause the SLAM if the pause is activated. As for "render all", we need to get the current status of the pause flag
		bool doPause;
#ifdef HAVE_MODULE_QDISPLAY
		// If we're using a qt display, get the pause status from it
		if (rtslamoptions::dispQt)
		{
			boost::unique_lock<boost::mutex> runStatus_lock(viewerQt->runStatus.mutex);
			doPause = viewerQt->runStatus.pause;
			runStatus_lock.unlock();
		} else
#endif
			// If not, use the value set by the options.
			// NOTE: The pause option may be defined to the number of frames to run before pausing the
			//       system. Due to this feature we need to test if its value is !=0, since 0 means no pause.
			doPause = (rtslamoptions::pause != 0);
		// Check for the pause. Only do it if we also had new data.
		if (doPause && had_data && !(*world)->exit())
		{
			(*world)->slam_blocked(true); // block the slam
#ifdef HAVE_MODULE_QDISPLAY
			// If we're using qt display, wait for the user unpause or ask to process the next data
			if (rtslamoptions::dispQt)
			{
				boost::unique_lock<boost::mutex> runStatus_lock(viewerQt->runStatus.mutex);
				do {
					viewerQt->runStatus.condition.wait(runStatus_lock);
				} while (viewerQt->runStatus.pause && !viewerQt->runStatus.next);
				viewerQt->runStatus.next = 0; // clear the next status flag to be able to pause again if the user asked only for a slam iteraction
				runStatus_lock.unlock();
			} else
#endif
				// If not, wait for key in the console
				getchar();
			(*world)->slam_blocked(false); // Unblock the slam
		}

		// If we had data, increment world "time" and log
		if (had_data)
		{
			(*world)->t++;
			if (dataLogger) dataLogger->log();
		}
	} // temporal loop

	std::cout << "Stopping RT-SLAM" << std::endl;
	std::cout << "final_robot_position " << robotPtr->state.x(0) << " " << robotPtr->state.x(1) << " " << robotPtr->state.x(2) << std::endl;

	// Stop things properly before leaving
	demo_slam_simple_stop(world);

	JFR_GLOBAL_CATCH
} // demo_slam_simple_main

//bool demo_slam_display_first = true;

void demo_slam_simple_display(world_ptr_t *world)
{ JFR_GLOBAL_TRY

	/// \note I don't know why we need the next lines in the code so I'm commenting it out.
	/// \note For me it's not clear the distinction of "mode" and "replay option". There's no "mode" equivalent to "offline replay" option. From the original code I see the thread prioroty was set only in online mode, so I changed the line below to use the variable that stores the current mode.
//	if (demo_slam_display_first && (rtslamoptions::mode == rtslam::hardware::mOnline || rtslamoptions::mode == rtslam::hardware::mOnlineDump))
//	{
//		kernel::setCurrentThreadPriority(NICENESS);
//		demo_slam_display_first = false;
//	}

	// Start the main loop for the display
	kernel::Timer timer(DISPLAY_PERIOD*1000);
	while(!(*world)->exit())
	{
		// The lines below are just to display the last frame if slam is blocked or has finished. It's almost the same found on demo_slam_simple_main function
		/// \note Maybe move this block of code to a function?
		boost::unique_lock<kernel::VariableMutex<bool> > blocked_lock((*world)->slam_blocked);
		if ((*world)->slam_blocked.var)
		{
			if ((*world)->display_t+1 < (*world)->t+1 && (*world)->display_rendered)
			{
#ifdef HAVE_MODULE_QDISPLAY
				display::ViewerQt *viewerQt = NULL; ///< \note Why do we declare another pointer here if we already have the display stored in a global pointer?
				if (rtslamoptions::dispQt) viewerQt = PTR_CAST<display::ViewerQt*> ((*world)->getDisplayViewer(display::ViewerQt::id()));
				if (rtslamoptions::dispQt) viewerQt->bufferize(*world);
#endif
#ifdef HAVE_MODULE_GDHE
				display::ViewerGdhe *viewerGdhe = NULL; ///< \note Why do we declare another pointer here if we already have the display stored in a global pointer?
				if (rtslamoptions::dispGdhe) viewerGdhe = PTR_CAST<display::ViewerGdhe*> ((*world)->getDisplayViewer(display::ViewerGdhe::id()));
				if (rtslamoptions::dispGdhe) viewerGdhe->bufferize(*world);
#endif
				(*world)->display_t = (*world)->t;
				(*world)->display_rendered = false;
			}
		}
		blocked_lock.unlock();

		// waiting that display is ready
		boost::unique_lock<boost::mutex> display_lock((*world)->display_mutex);
		// If we're NOT using the qt display, just wait until the last rendering is over (maybe from GDHE).
		if (!rtslamoptions::dispQt)
		{
			while((*world)->display_rendered)
				(*world)->display_condition.wait(display_lock);
		} else
		{
#ifdef HAVE_MODULE_QDISPLAY
			// If we're using qt display, wait for the display to finish rendering but call processEvents() every 10 milliseconds while waiting.
			// This is needed to capture user inputs from qt display (qt events). The maximum wait time is equal to a display period (nwait loops).
			int nwait = std::max(1,DISPLAY_PERIOD/10-1);
			for(int i = 0; (*world)->display_rendered && i < nwait; ++i)
			{
				(*world)->display_condition.timed_wait(display_lock, boost::posix_time::milliseconds(10)); // Wait here
				display_lock.unlock();
				QApplication::instance()->processEvents(); // Process events here
				display_lock.lock();
			}
			// If we waited enough but the display didn't finish rendering, break the main loop and wait the function to be called again by qt
			if ((*world)->display_rendered) break;
#endif
		}
		display_lock.unlock();

		// Call the render functions for the displays being used.
#ifdef HAVE_MODULE_QDISPLAY
		display::ViewerQt *viewerQt = NULL; ///< \note Again... Why do we declare another pointer here if we already have the display stored in a global pointer?
		if (rtslamoptions::dispQt) viewerQt = PTR_CAST<display::ViewerQt*> ((*world)->getDisplayViewer(display::ViewerQt::id()));
		if (rtslamoptions::dispQt) viewerQt->render();
#endif
#ifdef HAVE_MODULE_GDHE
		display::ViewerGdhe *viewerGdhe = NULL; ///< \note Again... Why do we declare another pointer here if we already have the display stored in a global pointer?
		if (rtslamoptions::dispGdhe) viewerGdhe = PTR_CAST<display::ViewerGdhe*> ((*world)->getDisplayViewer(display::ViewerGdhe::id()));
		if (rtslamoptions::dispGdhe) viewerGdhe->render();
#endif

		// Dump rendered views if we are in any offline mode and with dump on.
		if ((rtslamoptions::replay == rtslamoptions::rOffline || rtslamoptions::replay == rtslamoptions::rOfflineReplay) && rtslamoptions::dump && (*world)->display_t+1 != 0)
		{
#ifdef HAVE_MODULE_QDISPLAY
			if (rtslamoptions::dispQt)
				{
					std::ostringstream oss; oss << rtslamoptions::datapath << "/rendered-2D_%d-" << std::setw(6) << std::setfill('0') << (*world)->display_t << ".png";
					viewerQt->dump(oss.str());
				}
#endif
#ifdef HAVE_MODULE_GDHE
			if (rtslamoptions::dispGdhe)
				{
					std::ostringstream oss; oss << rtslamoptions::datapath << "/rendered-3D_" << std::setw(6) << std::setfill('0') << (*world)->display_t << ".png";
					viewerGdhe->dump(oss.str());
				}
#endif
		}

		// Set that the display was rendered and notify other threads.
		display_lock.lock();
		(*world)->display_rendered = true;
		display_lock.unlock();
		(*world)->display_condition.notify_all();

		// If we're using the display qt we need to break the main loop here because qt has it's
		// own loop that keeps calling this function. Otherwise we wait a little until next loop.
		if (rtslamoptions::dispQt) break; else timer.wait();
	}

JFR_GLOBAL_CATCH
} // demo_slam_simple_display


void demo_slam_simple_run() {

	//kernel::setProcessScheduler(slam_sched, 5); // for whole process, including display thread
	//	demo_slam_display_first = true;

	// to start with qt display
	if (rtslamoptions::dispQt) // at least 2d
	{
#ifdef HAVE_MODULE_QDISPLAY
		qdisplay::QtAppStart((qdisplay::FUNC)&demo_slam_simple_display,0,(qdisplay::FUNC)&demo_slam_simple_main,0,DISPLAY_PERIOD,&worldPtr,(qdisplay::EXIT_FUNC)&demo_slam_simple_exit);
#else
		std::cout << "Please install qdisplay module if you want 2D display" << std::endl;
#endif
	} else
		if (rtslamoptions::dispGdhe) // only 3d
		{
#ifdef HAVE_MODULE_GDHE
			boost::thread *thread_disp = new boost::thread(boost::bind(demo_slam_simple_display,&worldPtr));
			demo_slam_simple_main(&worldPtr);
			delete thread_disp;
#else
			std::cout << "Please install gdhe module if you want 3D display" << std::endl;
#endif
		} else // none
		{
			// launch the main function
			demo_slam_simple_main(&worldPtr);
		}
} // demo_slam_simple_run
