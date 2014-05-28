#ifndef OPTION_PARSER_HPP
#define OPTION_PARSER_HPP

// To parse options from config file and command line
#include <boost/program_options.hpp>

#include <iostream>
#include <fstream>
#include <iterator>

// for rtslam::hardware::mode
#include <rtslam/hardwareSensorAbstract.hpp>

// Protect the option variables by an namespace
namespace rtslamoptions{

/// \todo Create enumerations here for the int options to help checking for options in the main code

// List of variables set by the command line arguments and config file
std::string logfile;
std::string datapath;
bool dispQt;
bool dispGdhe;
unsigned pause;
bool renderall;
unsigned replay;
bool dump;


// parser function
int parse_options(int ac, char* av[])
{
	namespace po = boost::program_options;
	using namespace std;

	// Internal variables to the parser functions
	string config_file;

	try {
		// Declare a group of options that will be allowed only on command line
		po::options_description generic("Generic options");
		generic.add_options()
				("version,v", "print version string")
				("help,h", "produce help message")
				("config,c", po::value<string>(&config_file),"name of a file of a configuration.")
				;

		// Declare a group of options that will be allowed both on
		// command line and in config file
		po::options_description config("Configuration");
		config.add_options()
				("replay", po::value<unsigned>(&replay)->default_value(0), "replay mode: '0' for online, '1' for offline, '2' for online no slam, '3' for offline replay")
				("log-file", po::value<string>(&logfile)->default_value("rtslam.log"), "log result output in text file in --data-path")
				("data-path", po::value<string>(&datapath)->default_value("."), "path to store or read data")
				("disp-2d", po::value<bool>(&dispQt)->default_value(false), "use 2D display (Qt)")
				("disp-3d", po::value<bool>(&dispGdhe)->default_value(false), "use 3D display (GDHE)")
				("pause", po::value<unsigned>(&pause)->default_value(0), "pause after integrating data: '0' disables, 'n' pauses after frame n")
				("render-all", po::value<bool>(&renderall)->default_value(false), "force rendering display for all frames")
				("dump", po::value<bool>(&dump)->default_value(false), "dump images (with --replay=0/2) or rendered views (with --replay=1/3)") /// \warning Not sure of the behaviour with --replay=2/3
				;

		// Create the description for the cmdline options from generic and config descriptions
		po::options_description cmdline_options;
		cmdline_options.add(generic).add(config);

		// Create the description for the config file options from generic and config descriptions
		po::options_description config_file_options;
		config_file_options.add(config);

		// Parser the command line options
		po::variables_map vm;
		po::store(po::command_line_parser(ac, av).
				  options(cmdline_options).run(), vm);
		po::notify(vm);

		// Verify if a config file was passed as an option
		if (vm.count("config")) {

			ifstream ifs(config_file.c_str());
			if (!ifs)
			{
				cout << "can not open config file: " << config_file << "\n";
				exit(1);
			}

			// Parse the options on the config file.
			// NOTE: They are not going to be updated if they were already set by the command line option
			po::store(parse_config_file(ifs, config_file_options), vm);
			po::notify(vm);
		}

		// Check for generic options
		if (vm.count("help")) {
			cout << cmdline_options << "\n";
			exit(0);
		}
		if (vm.count("version")) {
			cout << "RT-SLAM ROS, version 0.1\n";
			exit(0);
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

#endif // OPTION_PARSER_HPP
