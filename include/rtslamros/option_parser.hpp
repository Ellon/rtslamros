#ifndef OPTION_PARSER_HPP
#define OPTION_PARSER_HPP

// To parse options from config file and command line
#include <boost/program_options.hpp>

#include <iostream>
#include <fstream>
#include <iterator>

// for rtslam::hardware::mode
#include <rtslam/hardwareSensorAbstract.hpp>

// to save values into config files
#include <kernel/keyValueFile.hpp>


class ConfigSetup: public jafar::kernel::KeyValueFileSave
{
 public:
	/// SENSOR
	jblas::vec6 ROBOT_POSE; ///< real robot pose in SLAM frame (IMU frame in inertial) (for init and export)
	jblas::vec6 CAMERA_POSE; ///< camera pose in SLAM frame (IMU frame) for inertial (x,y,z,roll,pitch,yaw) (m,deg). If add std devs, will be filtered.

	std::string CAMERA_DEVICE; ///< camera device (firewire ID or device)
	unsigned CAMERA_IMG_WIDTH;     ///< image width
	unsigned CAMERA_IMG_HEIGHT;    ///< image height
	jblas::vec4 CAMERA_INTRINSIC;  ///< intrisic calibration parameters (u0,v0,alphaU,alphaV)
	jblas::vec3 CAMERA_DISTORTION; ///< distortion calibration parameters (r1,r2,r3)
	std::string CAMERA_CALIB;      ///< calibration file if need to rectify

	/// INERTIAL (also using UNCERT_VLIN)
	double UNCERT_VLIN;        ///< initial uncertainty stdev on linear velocity (m/s)
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

private:
	void processKeyValueFile(jafar::kernel::KeyValueFile& keyValueFile, bool read);
public:
	virtual void saveKeyValueFile(jafar::kernel::KeyValueFile& keyValueFile);
} configSetup;

class ConfigEstimation: public jafar::kernel::KeyValueFileSave
{
 public:
	/// MISC
	unsigned CORRECTION_SIZE; ///< number of coefficients for the distortion correction polynomial

	/// FILTER
	unsigned MAP_SIZE; ///< map size in # of states, robot + landmarks
	double PIX_NOISE;  ///< measurement noise of a point

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
	virtual void saveKeyValueFile(jafar::kernel::KeyValueFile& keyValueFile);

} configEstimation;

// Protect the option variables by an namespace
namespace rtslamoptions{

/// \todo Create enumerations here for the int options to help checking for options in the main code

// Replay enumeration
enum {
	rOnline = 0,
	rOffline,
	rOnlineNoSlam,
	rOfflineReplay
};
// Random Seed enumeration
enum {
	seedGenerate = 0,
	seedUseSaved
};
// Dump enumeration
enum {
	dumpOff = 0,
	dumpSensorsOrRendered,
	dumpMatches
};

// List of variables set by the command line arguments and config file
std::string logfile;
std::string datapath;
bool dispQt;
bool dispGdhe;
unsigned pause;
bool renderall;
unsigned replay;
unsigned dump;
unsigned randomseed;


// parser function
int parse_options(int ac, char* av[])
{
	namespace po = boost::program_options;
	using namespace std;

	try {
		// Declare a group of options that will be allowed only on command line
		po::options_description generic("Generic options");
		generic.add_options()
				("version,v", "print version string")
				("help,h", "produce help message")
		        ("help-setup", "show setup options")
		        ("help-estimation", "show estimation options")
				("config,c", po::value< vector<string> >(), "name of a configuration file.")
				;

		// Mount the description string of replay option.
		std::stringstream replay_ss;
		replay_ss << std::string("replay mode: '")
				  << rOnline << std::string("' for online, '")
				  << rOffline << std::string("' for offline, '")
				  << rOnlineNoSlam << std::string("' for online no slam, '")
				  << rOfflineReplay << std::string("' for offline replay");

		// Mount the description string of random seed
		std::stringstream seed_ss;
		seed_ss << std::string("randon seed: '")
				  << seedGenerate << std::string("' generate a new one, '")
				  << seedUseSaved << std::string("' use a saved one, '")
				  << std::string("'n' uses n as a new seed");

		// Mount the description string of random seed
		std::stringstream dump_ss;
		dump_ss << std::string("dump data: '")
				  << dumpOff << std::string("' do not dump, '")
				  << dumpSensorsOrRendered << std::string("' dump images or rendered views, '")
				  << dumpMatches << std::string("' dump matches'");

		// Declare a group of options that will be allowed both on
		// command line and in config file
		po::options_description config("Configuration");
		config.add_options()
				("replay", po::value<unsigned>(&replay)->default_value(rOnline), replay_ss.str().c_str())
				("rand-seed", po::value<unsigned>(&randomseed)->default_value(seedGenerate), seed_ss.str().c_str())
				("log-file", po::value<string>(&logfile)->default_value("rtslam.log"), "log result output in text file in --data-path")
				("data-path", po::value<string>(&datapath)->default_value("."), "path to store or read data")
				("disp-2d", po::value<bool>(&dispQt)->default_value(false), "use 2D display (Qt)")
				("disp-3d", po::value<bool>(&dispGdhe)->default_value(false), "use 3D display (GDHE)")
				("pause", po::value<unsigned>(&pause)->default_value(0), "pause after integrating data: '0' disables, 'n' pauses after frame n")
				("render-all", po::value<bool>(&renderall)->default_value(false), "force rendering display for all frames")
				("dump", po::value<unsigned>(&dump)->default_value(dumpOff), dump_ss.str().c_str())
				;

		// Declare a group of hidden options, will be allowed both on command line and
		// in config file, but will not be shown to the user. These are most the
		// setup and estimation options
        po::options_description setup("Setup options");
        setup.add_options()
				("ROBOT_POSE", po::value< jblas::vec6 >(&configSetup.ROBOT_POSE), "real robot pose in SLAM frame (IMU frame in inertial) (for init and export)")
				("CAMERA_POSE", po::value< jblas::vec6 >(&configSetup.CAMERA_POSE), "camera pose in SLAM frame (IMU frame) for inertial (x,y,z,roll,pitch,yaw) (m,deg)")
		        ("CAMERA_DEVICE", po::value< std::string >(&configSetup.CAMERA_DEVICE), "camera device (firewire ID or device)")
		        ("CAMERA_IMG_WIDTH", po::value< unsigned >(&configSetup.CAMERA_IMG_WIDTH), "image width")
		        ("CAMERA_IMG_HEIGHT", po::value< unsigned >(&configSetup.CAMERA_IMG_HEIGHT), "image height")
		        ("CAMERA_INTRINSIC", po::value< jblas::vec4 >(&configSetup.CAMERA_INTRINSIC), "intrisic calibration parameters (u0,v0,alphaU,alphaV)")
		        ("CAMERA_DISTORTION", po::value< jblas::vec3 >(&configSetup.CAMERA_DISTORTION), "distortion calibration parameters (r1,r2,r3)")
		        ("CAMERA_CALIB", po::value< std::string >(&configSetup.CAMERA_CALIB), "calibration file if need to rectify")
		        ("UNCERT_VLIN", po::value< double >(&configSetup.UNCERT_VLIN), "initial uncertainty stdev on linear velocity (m/s)")
		        ("ACCELERO_FULLSCALE", po::value< double >(&configSetup.ACCELERO_FULLSCALE), "full scale of accelerometers (m/s2)  (MTI: 17)")
		        ("ACCELERO_NOISE", po::value< double >(&configSetup.ACCELERO_NOISE), "noise stdev of accelerometers (m/s2) (MTI: 0.002*sqrt(30) )")
		        ("GYRO_FULLSCALE", po::value< double >(&configSetup.GYRO_FULLSCALE), "full scale of gyrometers (rad/s)     (MTI: rad(300) )")
		        ("GYRO_NOISE", po::value< double >(&configSetup.GYRO_NOISE), "noise stdev of gyrometers (rad/s)    (MTI: rad(0.05)*sqrt(40) )")
		        ("INITIAL_GRAVITY", po::value< double >(&configSetup.INITIAL_GRAVITY), "initial value of gravity (default value 9.806, m/s2)")
		        ("UNCERT_GRAVITY", po::value< double >(&configSetup.UNCERT_GRAVITY), "initial gravity uncertainty (% of INITIAL_GRAVITY)")
		        ("UNCERT_ABIAS", po::value< double >(&configSetup.UNCERT_ABIAS), "initial accelerometer bias uncertainty (% of ACCELERO_FULLSCALE, m/s2)")
		        ("UNCERT_WBIAS", po::value< double >(&configSetup.UNCERT_WBIAS), "initial gyrometer bias uncertainty (% of GYRO_FULLSCALE, rad/s)")
		        ("PERT_AERR", po::value< double >(&configSetup.PERT_AERR), "noise stdev coeff of accelerometers, for testing purpose (% of ACCELERO_NOISE)")
		        ("PERT_WERR", po::value< double >(&configSetup.PERT_WERR), "noise stdev coeff of gyrometers, for testing purpose (% of GYRO_NOISE)")
		        ("PERT_RANWALKACC", po::value< double >(&configSetup.PERT_RANWALKACC), "IMU a_bias random walk (m/s2 per sqrt(s))")
		        ("PERT_RANWALKGYRO", po::value< double >(&configSetup.PERT_RANWALKGYRO), "IMU w_bias random walk (rad/s per sqrt(s))")
		        ("INITIAL_HEADING", po::value< double >(&configSetup.INITIAL_HEADING), "initial heading of the real robot (0 is east, positive is toward north, rad)")
		        ("UNCERT_HEADING", po::value< double >(&configSetup.UNCERT_HEADING), "initial heading uncertainty of the real robot (rad)")
		        ("UNCERT_ATTITUDE", po::value< double >(&configSetup.UNCERT_ATTITUDE), "initial attitude angles uncertainty (rad)")
		        ("IMU_TIMESTAMP_CORRECTION", po::value< double >(&configSetup.IMU_TIMESTAMP_CORRECTION), "correction to add to the IMU timestamp for synchronization (s)")
		        ;

		po::options_description estimation("Estimation options");
		estimation.add_options()
		        ("CORRECTION_SIZE", po::value< unsigned >(&configEstimation.CORRECTION_SIZE), "number of coefficients for the distortion correction polynomial")
		        ("MAP_SIZE", po::value< unsigned >(&configEstimation.MAP_SIZE), "map size in # of states, robot + landmarks")
		        ("PIX_NOISE", po::value< double >(&configEstimation.PIX_NOISE), "measurement noise of a point")
		        ("D_MIN", po::value< double >(&configEstimation.D_MIN), "inverse depth mean initialization")
				("REPARAM_TH", po::value< double >(&configEstimation.REPARAM_TH), "reparametrization threshold")
		        ("GRID_HCELLS", po::value< unsigned >(&configEstimation.GRID_HCELLS), "number of horizontal cells of the image grid for landmark density control")
		        ("GRID_VCELLS", po::value< unsigned >(&configEstimation.GRID_VCELLS), "number of vertical cells")
		        ("GRID_MARGIN", po::value< unsigned >(&configEstimation.GRID_MARGIN), "min margin of a cell that must be in the image when shifting the grid")
		        ("GRID_SEPAR", po::value< unsigned >(&configEstimation.GRID_SEPAR), "min separation between landmarks in the image for creation (margin with the border of the cell where landmarks can be initialized)")
		        ("RELEVANCE_TH", po::value< double >(&configEstimation.RELEVANCE_TH), "relevance threshold to make an update (# of sigmas)")
		        ("MAHALANOBIS_TH", po::value< double >(&configEstimation.MAHALANOBIS_TH), "mahalanobis distance for gating (# of sigmas)")
		        ("N_UPDATES_TOTAL", po::value< unsigned >(&configEstimation.N_UPDATES_TOTAL), "max number of landmarks to update every frame")
		        ("N_UPDATES_RANSAC", po::value< unsigned >(&configEstimation.N_UPDATES_RANSAC), "max number of landmarks to update with ransac every frame")
		        ("N_INIT", po::value< unsigned >(&configEstimation.N_INIT), "maximum number of landmarks to try to initialize every frame")
		        ("N_RECOMP_GAINS", po::value< unsigned >(&configEstimation.N_RECOMP_GAINS), "how many times information gain is recomputed to resort observations in active search")
		        ("RANSAC_LOW_INNOV", po::value< double >(&configEstimation.RANSAC_LOW_INNOV), "ransac low innovation threshold (pixels)")
		        ("RANSAC_NTRIES", po::value< unsigned >(&configEstimation.RANSAC_NTRIES), "number of base observation used to initialize a ransac set")
		        ("MULTIPLE_DEPTH_HYPOS", po::value< bool >(&configEstimation.MULTIPLE_DEPTH_HYPOS), "make multiple depth hypotheses when search ellipses are too big and distortion too strong")
		        ("HARRIS_CONV_SIZE", po::value< unsigned >(&configEstimation.HARRIS_CONV_SIZE), "harris detector convolution size")
		        ("HARRIS_TH", po::value< double >(&configEstimation.HARRIS_TH), "harris threshold")
		        ("HARRIS_EDDGE", po::value< double >(&configEstimation.HARRIS_EDDGE), "harris symmetry factor")
		        ("DESC_SIZE", po::value< unsigned >(&configEstimation.DESC_SIZE), "descriptor patch size (odd value)")
		        ("MULTIVIEW_DESCRIPTOR", po::value< bool >(&configEstimation.MULTIVIEW_DESCRIPTOR), "whether use or not the multiview descriptor")
		        ("DESC_SCALE_STEP", po::value< double >(&configEstimation.DESC_SCALE_STEP), "MultiviewDescriptor: min change of scale (ratio)")
		        ("DESC_ANGLE_STEP", po::value< double >(&configEstimation.DESC_ANGLE_STEP), "MultiviewDescriptor: min change of point of view (deg)")
		        ("DESC_PREDICTION_TYPE", po::value< int >(&configEstimation.DESC_PREDICTION_TYPE), "type of prediction from descriptor (0 = none, 1 = affine, 2 = homographic)")
		        ("PATCH_SIZE", po::value< unsigned >(&configEstimation.PATCH_SIZE), "patch size used for matching")
		        ("MAX_SEARCH_SIZE", po::value< unsigned >(&configEstimation.MAX_SEARCH_SIZE), "if the search area is larger than this # of pixels, we bound it")
		        ("KILL_SEARCH_SIZE", po::value< unsigned >(&configEstimation.KILL_SEARCH_SIZE), "if the search area is larger than this # of pixels, we vote for killing the landmark")
		        ("MATCH_TH", po::value< double >(&configEstimation.MATCH_TH), "ZNCC score threshold")
		        ("MIN_SCORE", po::value< double >(&configEstimation.MIN_SCORE), "min ZNCC score under which we don't finish to compute the value of the score")
		        ("HI_MATCH_TH", po::value< double >(&configEstimation.HI_MATCH_TH), "higher ZNCC score threshold for landmarks with high depth uncertainty and high expectation uncertainty")
		        ("HI_LIMIT", po::value< double >(&configEstimation.HI_LIMIT), "limit in pixels of the expectation uncertainty to use HI_MATCH_TH")
		        ("PARTIAL_POSITION", po::value< double >(&configEstimation.PARTIAL_POSITION), "position in the patch where we test if we finish the correlation computation")
		        ;

		// Create the description for the cmdline options from generic and config descriptions
		po::options_description cmdline_options;
		cmdline_options.add(generic).add(config).add(setup).add(estimation);

		// Create the description for the config file options from generic and config descriptions
		po::options_description config_file_options;
		config_file_options.add(config).add(setup).add(estimation);

		// Create a descriotion to be shown to the user if he asks for help
		po::options_description visible("Allowed options");
		visible.add(generic).add(config);

		// Parser the command line options
		po::variables_map vm;
		po::store(po::command_line_parser(ac, av).
				  options(cmdline_options).run(), vm);
		po::notify(vm);

		// Verify if a config file was passed as an option
		if (vm.count("config")) {

			const std::vector<std::string> &config_files = vm["config"].as< std::vector<std::string> >();

			for(std::vector<std::string>::const_iterator it = config_files.begin();
				it != config_files.end(); ++it) {

				ifstream ifs(it->c_str());
				if (!ifs)
				{
					cout << "can not open config file: " << *it << "\n";
					exit(1);
				}

				// Parse the options on the config file.
				// NOTE: They are not going to be updated if they were already set by the command line option
				po::store(parse_config_file(ifs, config_file_options), vm);
				po::notify(vm);
			}
		}

		// Check for generic options
		if (vm.count("help")) {
			cout << visible << "\n";
		}
		if (vm.count("help-setup")) {
			cout << setup << "\n";
		}
		if (vm.count("help-estimation")) {
			cout << estimation << "\n";
		}
		if (vm.count("version")) {
			cout << "RT-SLAM ROS, version 0.1\n";
		}
		// Exit here if any of these options where selected
		if(vm.count("help") || vm.count("version") || vm.count("help-setup") || vm.count("help-estimation")) {
			exit(0);
		}

		// Check for invalid options
		if(vm.count("replay") && vm.count("dump"))
		{
			if((replay == rOnline || replay == rOnlineNoSlam) && dump == dumpMatches){
				std::cerr << "Error: --dump=" << dumpMatches << " only valid with --replay="
						  << rOffline << " or " << rOfflineReplay << std::endl;
				exit(1);
			}
			/// \note There's a bug when replaying with no slam and dumping. Since we process_fake, we do not set last_updated on robotPtr and then it causes a segfaut when logging. Need to investigate if it also happens on RT-SLAM demo.
			if(replay == rOnlineNoSlam && dump == dumpSensorsOrRendered){
				std::cerr << "Error: There's a bug when using --dump=" << dumpSensorsOrRendered
						  << " and --replay=" << rOnlineNoSlam << ". These options are not available until we fix the bug." << std::endl;
				exit(1);
			}
		}

		if (vm.count("log-file"))
		{
			logfile = datapath + "/" + logfile;
		}

	}
	catch(exception& e)
	{
		cout << e.what() << "\n";
		exit(1);
	}
	return 0;
}

}

#define KeyValueFile_processItem(k) { read ? keyValueFile.getItem(#k, k) : keyValueFile.setItem(#k, k); }
#define KeyValueFile_processItem_def(k, def) { read ? keyValueFile.getItem(#k, k, def) : keyValueFile.setItem(#k, k); }

void ConfigSetup::saveKeyValueFile(jafar::kernel::KeyValueFile& keyValueFile)
{
  processKeyValueFile(keyValueFile, false);
}

void ConfigSetup::processKeyValueFile(jafar::kernel::KeyValueFile& keyValueFile, bool read)
{
	/// SENSOR
	KeyValueFile_processItem(ROBOT_POSE);
	KeyValueFile_processItem(CAMERA_POSE); ///< camera pose in SLAM frame (IMU frame) for inertial (x,y,z,roll,pitch,yaw) (m,deg). If add std devs, will be filtered.

	KeyValueFile_processItem(CAMERA_DEVICE);
	KeyValueFile_processItem(CAMERA_IMG_WIDTH);
	KeyValueFile_processItem(CAMERA_IMG_HEIGHT);
	KeyValueFile_processItem(CAMERA_INTRINSIC);
	KeyValueFile_processItem(CAMERA_DISTORTION);
	KeyValueFile_processItem(CAMERA_CALIB);

	/// INERTIAL (also using UNCERT_VLIN)
	KeyValueFile_processItem(UNCERT_VLIN);
	KeyValueFile_processItem(ACCELERO_FULLSCALE);
	KeyValueFile_processItem(ACCELERO_NOISE);
	KeyValueFile_processItem(GYRO_FULLSCALE);
	KeyValueFile_processItem(GYRO_NOISE);

	KeyValueFile_processItem(INITIAL_GRAVITY);
	KeyValueFile_processItem(UNCERT_GRAVITY);
	KeyValueFile_processItem(UNCERT_ABIAS);
	KeyValueFile_processItem(UNCERT_WBIAS);
	KeyValueFile_processItem(PERT_AERR);
	KeyValueFile_processItem(PERT_WERR);
	KeyValueFile_processItem(PERT_RANWALKACC);
	KeyValueFile_processItem(PERT_RANWALKGYRO);

	KeyValueFile_processItem(INITIAL_HEADING);
	KeyValueFile_processItem(UNCERT_HEADING);
	KeyValueFile_processItem(UNCERT_ATTITUDE);

	KeyValueFile_processItem(IMU_TIMESTAMP_CORRECTION);
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
	KeyValueFile_processItem(MULTIPLE_DEPTH_HYPOS);

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

	KeyValueFile_processItem(HI_MATCH_TH);
	KeyValueFile_processItem(HI_LIMIT);

	KeyValueFile_processItem(PARTIAL_POSITION);
}

#endif // OPTION_PARSER_HPP
