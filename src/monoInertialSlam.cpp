#include <rtslamros/monoInertialSlam.hpp>

#include <rtslam/worldAbstract.hpp>
#include <rtslam/mapManager.hpp>
#include <rtslam/mapManager.hpp>
#include <rtslam/robotInertial.hpp>
#include <rtslam/sensorPinhole.hpp>
#include <rtslam/observationMakers.hpp>
#include <rtslam/dataManagerOnePointRansac.hpp>
#include <rtslam/activeSearch.hpp>
#include <rtslam/rawProcessors.hpp>
#include <rtslam/sensorManager.hpp>

#include <rtslamros/configEstimation.hpp>
#include <rtslamros/configMonoInertial.hpp>
#include <rtslamros/hardwareSensorMtiRos.hpp>
#include <rtslamros/hardwareSensorCameraRos.hpp>


namespace rtslamros {

MonoInertialSlam::MonoInertialSlam() :
	imudata_condition(0),
	imagedata_condition(0)
{
	using namespace jafar;
	using namespace jafar::rtslam;
	using namespace boost;

	typedef ImagePointObservationMaker<ObservationPinHoleEuclideanPoint, SensorPinhole, LandmarkEuclideanPoint,
		AppearanceImagePoint, SensorAbstract::PINHOLE, LandmarkAbstract::PNT_EUC> PinholeEucpObservationMaker;
	typedef ImagePointObservationMaker<ObservationPinHoleAnchoredHomogeneousPoint, SensorPinhole, LandmarkAnchoredHomogeneousPoint,
		AppearanceImagePoint, SensorAbstract::PINHOLE, LandmarkAbstract::PNT_AH> PinholeAhpObservationMaker;
	typedef DataManagerOnePointRansac<RawImage, SensorPinhole, FeatureImagePoint, image::ConvexRoi, ActiveSearchGrid, ImagePointHarrisDetector, ImagePointZnccMatcher> DataManager_ImagePoint_Ransac;

	const double gridDistInit = 0.5;
	const double gridDistFactor = 2.0;
	const int gridNDist = 5;
	const double gridPhiFactor = 1.2;

	// 1) create world
	worldPtr_.reset(new WorldAbstract());

	// 2) create map
	map_ptr_t mapPtr(new MapAbstract(ConfigEstimation::mapSize()));
	mapPtr->linkToParentWorld(worldPtr_);

	// 3) create map manager
	// 3.1) create landmark factory
	landmark_factory_ptr_t pointLmkFactory(new LandmarkFactory<LandmarkAnchoredHomogeneousPoint, LandmarkEuclideanPoint>());
	// 3.2) create the map manager and add to map
	double min_cell_fov = computeMinCelFov();
	map_manager_ptr_t mmPoint(new MapManagerOdometry(pointLmkFactory, ConfigEstimation::reparamTh(), ConfigEstimation::killSearchSize(),
					min_cell_fov, gridDistInit, gridDistFactor, gridNDist, gridPhiFactor));
	mmPoint->linkToParentMap(mapPtr);

	// 4) create robot
	// 4.1) Create robot and set initial parameters
	robinertial_ptr_t robPtr1(new RobotInertial(mapPtr));
	robPtr1->setInitialStd(
			ConfigMonoInertial::uncertVlin(),
			ConfigMonoInertial::uncertAbias()*ConfigMonoInertial::acceleroFullscale(),
			ConfigMonoInertial::uncertWbias()*ConfigMonoInertial::gyro_Fullscale(),
			ConfigMonoInertial::uncertGravity()*ConfigMonoInertial::initialGravity());
	robPtr1->setInitialParams(ConfigMonoInertial::initialGravity());
	double aerr = ConfigMonoInertial::pertAerr() * ConfigMonoInertial::acceleroNoise();
	double werr = ConfigMonoInertial::pertWerr() * ConfigMonoInertial::gyroNoise();
	double _v[12] = {
		aerr, aerr, aerr, werr, werr, werr,
		ConfigMonoInertial::pertRanWalkAcc(), ConfigMonoInertial::pertRanWalkAcc(), ConfigMonoInertial::pertRanWalkAcc(),
		ConfigMonoInertial::pertRanWalkGyro(), ConfigMonoInertial::pertRanWalkGyro(), ConfigMonoInertial::pertRanWalkGyro()};
	vec pertStd = createVector<12>(_v);
	robPtr1->perturbation.set_std_continuous(pertStd);
	// or robPtr1->perturbation.set_std_continuous(ublas::subrange(pertStd,0,6));
	// 4.2) set robot hardware (the IMU)
	boost::shared_ptr<rtslamros::hardware::HardwareSensorMtiRos> hardEst1(new rtslamros::hardware::HardwareSensorMtiRos(
				&imudata_condition, 1024));
	hardEst1->setSyncConfig(ConfigMonoInertial::imuTimestampCorrection());
	robPtr1->setHardwareEstimator(hardEst1);
	robPtr1->linkToParentMap(mapPtr);

	robPtr1->setRobotPose(ublas::subrange(ConfigMonoInertial::robotPose(),0,6), true);
	double heading = ConfigMonoInertial::initialHeading();
	robPtr1->setOrientationStd(0,0,heading,
		ConfigMonoInertial::uncertAttitude(),ConfigMonoInertial::uncertAttitude(),ConfigMonoInertial::uncertHeading(), false);

	// 5) Create camera
	// 5.1) Create observation factory
	boost::shared_ptr<ObservationFactory> obsFact(new ObservationFactory());
	obsFact->addMaker(boost::shared_ptr<ObservationMakerAbstract>(new PinholeEucpObservationMaker(
		ConfigEstimation::dMin(), ConfigEstimation::patchSize())));
	obsFact->addMaker(boost::shared_ptr<ObservationMakerAbstract>(new PinholeAhpObservationMaker(
		ConfigEstimation::dMin(), ConfigEstimation::patchSize())));
	// 5.2) Create camera
	pinhole_ptr_t senPtr11;
	// TODO: Add filtered camera position through options
	senPtr11 = pinhole_ptr_t(new SensorPinhole(robPtr1, MapObject::UNFILTERED ));
	senPtr11->linkToParentRobot(robPtr1);
	senPtr11->setPose(ConfigMonoInertial::cameraPose()(0), ConfigMonoInertial::cameraPose()(1), ConfigMonoInertial::cameraPose()(2),
					  ConfigMonoInertial::cameraPose()(3), ConfigMonoInertial::cameraPose()(4), ConfigMonoInertial::cameraPose()(5)); // x,y,z,roll,pitch,yaw
	// or...
	// senPtr11->setPoseStd(camera_pose(0), camera_pose(1), camera_pose(2), camera_pose(3), camera_pose(4), camera_pose(5),
	// 	camera_pose(6), camera_pose(7), camera_pose(8), camera_pose(9), camera_pose(10), camera_pose(11)); // x,y,z,roll,pitch,yaw + std_dev
	senPtr11->params.setIntrinsicCalibration(ConfigMonoInertial::cameraImgWidth(), ConfigMonoInertial::cameraImgHeight(), ConfigMonoInertial::cameraIntrinsic(), ConfigMonoInertial::cameraDistortion(), ConfigEstimation::correctionSize());
	senPtr11->params.setMiscellaneous(ConfigEstimation::pixNoise(), ConfigEstimation::dMin());
	senPtr11->setIntegrationPolicy(false);
	senPtr11->setUseForInit(false);
	senPtr11->setNeedInit(true); // for auto exposure

	// 5.3) Create data manager
	boost::shared_ptr<ActiveSearchGrid> asGrid(new ActiveSearchGrid(ConfigMonoInertial::cameraImgWidth(), ConfigMonoInertial::cameraImgHeight(), ConfigEstimation::gridHcells(), ConfigEstimation::gridVcells(), ConfigEstimation::gridMargin(), ConfigEstimation::gridSepar()));
	boost::shared_ptr<DescriptorFactoryAbstract> pointDescFactory;
	pointDescFactory.reset(new DescriptorImagePointFirstViewFactory(ConfigEstimation::descSize()));
	boost::shared_ptr<ImagePointHarrisDetector> harrisDetector(new ImagePointHarrisDetector(ConfigEstimation::harrisConvSize(), ConfigEstimation::harrisTh(), ConfigEstimation::harrisEdge(), ConfigEstimation::patchSize(), ConfigEstimation::pixNoise(), pointDescFactory));
	boost::shared_ptr<ImagePointZnccMatcher> znccMatcher(new ImagePointZnccMatcher(ConfigEstimation::minScore(), ConfigEstimation::partialPosition(), ConfigEstimation::patchSize(), ConfigEstimation::maxSearchSize(), ConfigEstimation::ransacLowInnov(), ConfigEstimation::matchTh(), ConfigEstimation::hiMatchTh(), ConfigEstimation::hiLimit(), ConfigEstimation::mahalanobisTh(), ConfigEstimation::relevanceTh(), ConfigEstimation::pixNoise()));
	boost::shared_ptr<DataManager_ImagePoint_Ransac> dmPt11(new DataManager_ImagePoint_Ransac(harrisDetector, znccMatcher, asGrid, ConfigEstimation::nUpdatesTotal(), ConfigEstimation::nUpdatesRansac(), ConfigEstimation::ransacNtries(), ConfigEstimation::nInit(), ConfigEstimation::nRecompGains(), ConfigEstimation::multipleDepthHypos, NULL));

	dmPt11->linkToParentSensorSpec(senPtr11);
	dmPt11->linkToParentMapManager(mmPoint);
	dmPt11->setObservationFactory(obsFact);

	// 5.4) Create camera hardware
	rtslamros::hardware::hardware_sensor_camera_ros_ptr_t hardSen11;
	hardSen11 = rtslamros::hardware::hardware_sensor_camera_ros_ptr_t(new rtslamros::hardware::HardwareSensorCameraRos(&imagedata_condition,
		                                                                                                               1, // camera ID
		                                                                                                               cv::Size(ConfigMonoInertial::cameraImgWidth(), ConfigMonoInertial::cameraImgHeight()), // image size
		                                                                                                               500)); // buffer size
	hardSen11->setTimingInfos(1.0/hardSen11->getFreq(), 1.0/hardSen11->getFreq());
	// hardSen11->setFilter(filter_div, filter_mods[c]);
	senPtr11->setHardwareSensor(hardSen11);

	sensorManager.reset(new SensorManagerOnline(mapPtr));
	// or maybe use the offline?
	// or maybe do not use at all?
}

double MonoInertialSlam::computeMinCelFov()
{
	// compute min_cell_fov, min over all camera sensors
	double cell_fov, min_cell_fov = 1e10;
	cell_fov = 2. * atan(ConfigMonoInertial::cameraImgWidth() / (2. * ConfigMonoInertial::cameraIntrinsic()(2))) / ConfigEstimation::gridHcells();
	if (cell_fov < min_cell_fov) min_cell_fov = cell_fov;
	cell_fov = 2. * atan(ConfigMonoInertial::cameraImgHeight() / (2. * ConfigMonoInertial::cameraIntrinsic()(3))) / ConfigEstimation::gridVcells();
	if (cell_fov < min_cell_fov) min_cell_fov = cell_fov;

	min_cell_fov /= 2.; // security factor
	min_cell_fov *= 180./M_PI;

	return min_cell_fov;
}

} // namespace rtslamros