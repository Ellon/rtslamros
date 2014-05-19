bool demo_slam_simple_init()
{ JFR_GLOBAL_TRY

  std::cout << "Loading config files " << strOpts[sConfigSetup] << " and " << strOpts[sConfigEstimation] << std::endl;
  configSetup.load(strOpts[sConfigSetup]);
  configEstimation.load(strOpts[sConfigEstimation]);
	
  /// init world
  worldPtr.reset(new WorldAbstract());

  /// init display
  display::ViewerQt *viewerQt = new display::ViewerQt(8, configEstimation.MAHALANOBIS_TH, false, "data/rendered2D_%02d-%06d.png");
  worldPtr->addDisplayViewer(viewerQt, display::ViewerQt::id());
  display::ViewerGdhe *viewerGdhe = new display::ViewerGdhe("camera", configEstimation.MAHALANOBIS_TH, "localhost");
  boost::filesystem::path ram_path("/mnt/ram");
  if (boost::filesystem::exists(ram_path) && boost::filesystem::is_directory(ram_path))
    viewerGdhe->setConvertTempPath("/mnt/ram");
  worldPtr->addDisplayViewer(viewerGdhe, display::ViewerGdhe::id());

  /// ---------------------------------------------------------------------------
  /// --- INIT MAPS -------------------------------------------------------------
  /// ---------------------------------------------------------------------------

  // create map
  map_ptr_t mapPtr(new MapAbstract(configEstimation.MAP_SIZE));
  mapPtr->linkToParentWorld(worldPtr);
	
  // 1b. Create map manager.
  landmark_factory_ptr_t pointLmkFactory;
  pointLmkFactory.reset(new LandmarkFactory<LandmarkAnchoredHomogeneousPoint, LandmarkEuclideanPoint>());

  map_manager_ptr_t mmPoint;
  const double gridDistInit = 0.5;
  const double gridDistFactor = 2.0;
  const int gridNDist = 5;
  const double gridPhiFactor = 1.2;
  mmPoint.reset(new MapManagerOdometry(pointLmkFactory, configEstimation.REPARAM_TH, configEstimation.KILL_SEARCH_SIZE,
				       min_cell_fov, gridDistInit, gridDistFactor, gridNDist, gridPhiFactor));
  mmPoint->linkToParentMap(mapPtr);

  /// ---------------------------------------------------------------------------
  /// --- CREATE ROBOTS ---------------------------------------------------------
  /// ---------------------------------------------------------------------------

  double trigger_construction_date = -1;
  robot_ptr_t robPtr1;

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
	robPtr1_->perturbation.set_std_continuous(pertStd);
	
	hardware::hardware_sensorprop_ptr_t hardEst1;
	boost::shared_ptr<hardware::HardwareSensorMti> hardEst1_(new hardware::HardwareSensorMti(
												 &estimatordata_condition, configSetup.MTI_DEVICE, intOpts[iTrigger], floatOpts[fFreq], floatOpts[fShutter], 1024, mode, strOpts[sDataPath], loggerTask.get()));
	trigger_construction_date = kernel::Clock::getTime();
	if (intOpts[iTrigger] != 0) floatOpts[fFreq] = hardEst1_->getFreq();
	hardEst1_->setSyncConfig(configSetup.IMU_TIMESTAMP_CORRECTION);
	//hardEst1_->setUseForInit(true);
	//hardEst1_->setNeedInit(true);
	//hardEst1_->start();
	hardEst1 = hardEst1_;
	robPtr1_->setHardwareEstimator(hardEst1);
	
	robPtr1 = robPtr1_;
      
	robPtr1->linkToParentMap(mapPtr);
	robPtr1->setRobotPose(ublas::subrange(configSetup.ROBOT_POSE,0,6), true);
	double heading = (floatOpts[fHeading] < 1e4 ? floatOpts[fHeading] : configSetup.INITIAL_HEADING);
	robPtr1->setOrientationStd(0,0,heading,
				   configSetup.UNCERT_ATTITUDE,configSetup.UNCERT_ATTITUDE,configSetup.UNCERT_HEADING, false);
	if (dataLogger) dataLogger->addLoggable(*robPtr1.get());

  /// ---------------------------------------------------------------------------
  /// --- INIT SENSORS ----------------------------------------------------------
  /// ---------------------------------------------------------------------------


  /// pin-hole parameters in BOOST format
	boost::shared_ptr<ObservationFactory> obsFact(new ObservationFactory());
	obsFact->addMaker(boost::shared_ptr<ObservationMakerAbstract>(new PinholeEucpObservationMaker(
												      configEstimation.D_MIN, configEstimation.PATCH_SIZE)));
	obsFact->addMaker(boost::shared_ptr<ObservationMakerAbstract>(new PinholeAhpObservationMaker(
												     configEstimation.D_MIN, configEstimation.PATCH_SIZE)));

	pinhole_ptr_t senPtr11;
	  // a. Create camera
	  jblas::vec camera_pose = (intOpts[iRobot] == 1 ? configSetup.CAMERA_POSE_INERTIAL[c] : configSetup.CAMERA_POSE_CONSTVEL[c]);
	  int cp_size = camera_pose.size();
	  if (cp_size != 6 && cp_size != 12) { std::cerr << "Camera pose must have size 6 or 12 (with uncertainties), not " << cp_size << std::endl; worldPtr->error = eConfig; return false; }
	  senPtr11 = pinhole_ptr_t(new SensorPinhole(robPtr1, (cp_size == 6 ? MapObject::UNFILTERED : MapObject::FILTERED)));
	  senPtr11->linkToParentRobot(robPtr1);
	  senPtr11->name("cam");
	  if (cp_size == 6)
	    {
	      senPtr11->setPose(camera_pose(0), camera_pose(1), camera_pose(2), camera_pose(3), camera_pose(4), camera_pose(5)); // x,y,z,roll,pitch,yaw
	    } else
	    {
	      senPtr11->setPoseStd(camera_pose(0), camera_pose(1), camera_pose(2), camera_pose(3), camera_pose(4), camera_pose(5),
				   camera_pose(6), camera_pose(7), camera_pose(8), camera_pose(9), camera_pose(10), camera_pose(11)); // x,y,z,roll,pitch,yaw + std_dev
	    }
	  senPtr11->params.setIntrinsicCalibration(img_width[c], img_height[c], intrinsic[c], distortion[c], configEstimation.CORRECTION_SIZE);
	  senPtr11->params.setMiscellaneous(configEstimation.PIX_NOISE, configEstimation.D_MIN);

	  if (dataLogger) dataLogger->addLoggable(*senPtr11.get());

 	      senPtr11->setIntegrationPolicy(false);
	      senPtr11->setUseForInit(false);
	      senPtr11->setNeedInit(true); // for auto exposure

	  // b. Create data manager.
	  boost::shared_ptr<ActiveSearchGrid> asGrid(new ActiveSearchGrid(img_width[c], img_height[c], configEstimation.GRID_HCELLS, configEstimation.GRID_VCELLS, configEstimation.GRID_MARGIN, configEstimation.GRID_SEPAR));

	  int ransac_ntries = configEstimation.RANSAC_NTRIES;

	      boost::shared_ptr<DescriptorFactoryAbstract> pointDescFactory;
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
	      if (dataLogger) dataLogger->addLoggable(*dmPt11.get());

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
		hardware::hardware_sensor_firewire_ptr_t hardSen11;
		for(int itry = 0; itry < 2; ++itry)
		  {
		    hardSen11 = hardware::hardware_sensor_firewire_ptr_t(new hardware::HardwareSensorCameraFirewire(&rawdata_condition, 500,
														    configSetup.CAMERA_DEVICE[c], cv::Size(img_width[c],img_height[c]), configSetup.CAMERA_FORMAT[c], crop, floatOpts[fFreq], intOpts[iTrigger],
														    floatOpts[fShutter], mode, cam_id, load_calib ? configSetup.CAMERA_CALIB[c] : "", strOpts[sDataPath], loggerTask.get()));
		    if (hardSen11->initialized()) break; else std::cerr << "!HardwareSensorCameraFirewire " << hardSen11->id() << " failed to initialize" << (itry != 1 ? ", reset sensor and retry in 1 second." : ".") << std::endl;
		    hardSen11.reset();
		    if (itry != 1) sleep(1);
		  }
		if (!hardSen11)
		  {
		    std::cerr << "!!HardwareSensorCameraFirewire " << cam_id << " failed to start" << (itrya != ntrya-1 ? ", resetting bus and retrying." : ", abandoning.") << std::endl ;
		    initialized_cameras = false;
		    for(int cc = 0; cc < c; cc++) if (cams[cc]) cams_built[cc]->setHardwareSensor(hardware::hardware_sensorext_ptr_t());
		    sleep(1);
		    int r = std::system("dc1394_reset_bus || dc1394_reset_bus2");
		    if (r) std::cerr << "dc1394_reset_bus failed with error " << r << std::endl;
		    sleep(1);
		    break;
		  }

		if (!(intOpts[iReplay] & 1)) hardSen11->assessFirstImage(trigger_construction_date);
		hardSen11->setTimingInfos(1.0/hardSen11->getFreq(), 1.0/hardSen11->getFreq());
		hardSen11->setFilter(filter_div, filter_mods[c]);
		senPtr11->setHardwareSensor(hardSen11);
#else
		if (intOpts[iReplay] & 1)
		  {
		    hardware::hardware_sensorext_ptr_t hardSen11(new hardware::HardwareSensorCameraFirewire(&rawdata_condition, c+1, cv::Size(img_width[c],img_height[c]),strOpts[sDataPath]));
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
