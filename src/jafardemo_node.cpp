/**
 * \file demo_slam.cpp
 *
 * ## Add brief description here ##
 *
 * \author jsola@laas.fr
 * \date 28/04/2010
 *
 *  ## Add a description here ##
 *
 * \ingroup rtslam
 */

#include <fenv.h>

#define HAVE_MODULE_QDISPLAY
#define HAVE_MODULE_GDHE

/// See documentation of these options in demo_suite/demo_slam.cpp
#include <rtslam/main.hpp>


/**
 * If you want to run with real-time SCHED_RR scheduling policy, you need to start the program
 * as root or to chown root and chmod u+s it.
 *
 * Program options:
 * --disp-2d=0/1
 * --disp-3d=0/1
 * --render-all=0/1 (needs --replay 1)
 * --replay=0/1/2/3 (online/offline/online no slam/offline replay) (needs --data-path)
 * --dump=0/1/2 (off/sensors or rendered/matches)  (needs --data-path)
 * --rand-seed=0/1/n, 0=generate new one, 1=in replay use the saved one, n=use seed n
 * --pause=0/n 0=don't, n=pause for frames>n (needs --replay 1)
 * --log=0/1/filename -> log result in text file
 * --export=0/1/2 -> Off/socket/poster
 * --verbose=0/1/2/*3/4/5 -> Off/Trace/Warning/Debug/VerboseDebug/VeryVerboseDebug (only works with debug builds)
 * --data-path=/mnt/ram/rtslam
 * --config-setup=data/setup.cfg
 * --config-estimation=data/estimation.cfg
 * --help
 * --usage
 * --robot 0=constant vel, 1=inertial, 2=odometry, 10/11/12/13=generic constant position/velocity/acceleration/jerk
 * --map 0=odometry, 1=global, 2=local/multimap
 * --trigger 0=internal, 1=external mode 1 (controls shutter), 2=external mode 0, 3=external mode 14 (PointGrey (Flea) only)
 * --simu 0 or <environment id>*10+<trajectory id> (
 * --camera= <cams>*10 + <mode> -> <cams>: sum(2^id) ; <mode>: 0=Raw, 1=Rectify, 2=Stereo
 * --freq camera frequency in double Hz (with trigger==0/1)
 * --shutter shutter time in double seconds (0=auto); for trigger modes 0,2,3 the value is relative between 0 and 1
 * --gps=0/1/2/3 -> Off / Pos / Pos+Vel / Pos+Ori(mocap)
 * --odom=0/1 -> Off / On
 * --extloc=<source>*10+<type> ; source 0=disabled 1=online, 2=file on request ; type 0=bundle, 1=full pose
 *
 * You can use the following examples and only change values:
 * online test (old mode=0):
 *   demo_slam --disp-2d=1 --disp-3d=1 --render-all=0 --replay=0 --dump=0 --rand-seed=0 --pause=0 --data-path=data/rtslam01
 *   demo_slam --disp-2d=1 --disp-3d=1
 * online with dump (old mode=1):
 *   demo_slam --disp-2d=1 --disp-3d=1 --render-all=0 --replay=0 --dump=1 --rand-seed=0 --pause=0 --data-path=data/rtslam01
 *   demo_slam --disp-2d=1 --disp-3d=1 --dump=1 --data-path=data/rtslam01
 * replay with pause  (old mode=2):
 *   demo_slam --disp-2d=1 --disp-3d=1 --render-all=1 --replay=1 --dump=0 --rand-seed=1 --pause=1 --data-path=data/rtslam01
 * replay with dump  (old mode=3):
 *   demo_slam --disp-2d=1 --disp-3d=1 --render-all=1 --replay=1 --dump=1 --rand-seed=1 --pause=0 --data-path=data/rtslam01
 */
int main(int argc, char* const* argv)
{ JFR_GLOBAL_TRY
    std::cout << "_POSIX_PRIORITY_SCHEDULING: \"" << _POSIX_PRIORITY_SCHEDULING << "\"" << std::endl;
  intOpts[iVerbose] = 3;
  intOpts[iMap] = 1;
  intOpts[iCamera] = 10;
  intOpts[iCamsFilter] = 100000;
  floatOpts[fFreq] = 60.0;
  floatOpts[fShutter] = 0.0;
  floatOpts[fHeading] = 1e5;
  strOpts[sDataPath] = ".";
  strOpts[sConfigSetup] = "#!@";
  strOpts[sConfigEstimation] = "data/estimation.cfg";

#ifndef JFR_NDEBUG
  feenableexcept(FE_INVALID);
#endif

  while (1)
    {
      int c, option_index = 0;
      c = getopt_long_only(argc, argv, "", long_options, &option_index);
      if (c == -1) break;
      if (c == 0)
	{
	  if (option_index <= nLastIntOpt)
	    {
	      intOpts[option_index] = 1;
	      if (optarg) intOpts[option_index-nFirstIntOpt] = atoi(optarg);
	    } else
	    if (option_index <= nLastFloatOpt)
	      {
		if (optarg) floatOpts[option_index-nFirstFloatOpt] = atof(optarg);
	      } else
	      if (option_index <= nLastStrOpt)
		{
		  if (optarg) strOpts[option_index-nFirstStrOpt] = optarg;
		} else
		{
		  std::cout << "Integer options:" << std::endl;
		  for(int i = 0; i < nIntOpts; ++i)
		    std::cout << "\t--" << long_options[i+nFirstIntOpt].name << std::endl;
		  
		  std::cout << "Float options:" << std::endl;
		  for(int i = 0; i < nFloatOpts; ++i)
		    std::cout << "\t--" << long_options[i+nFirstFloatOpt].name << std::endl;

		  std::cout << "String options:" << std::endl;
		  for(int i = 0; i < nStrOpts; ++i)
		    std::cout << "\t--" << long_options[i+nFirstStrOpt].name << std::endl;
		  
		  std::cout << "Breaking options:" << std::endl;
		  for(int i = 0; i < 2; ++i)
		    std::cout << "\t--" << long_options[i+nFirstBreakingOpt].name << std::endl;
		  
		  return 0;
		}
	} else
	{
	  std::cerr << "Unknown option " << c << std::endl;
	}
    }
  
  if (!demo_slam_init()) exit(1);
  demo_slam_run();
  
  JFR_GLOBAL_CATCH
    }
