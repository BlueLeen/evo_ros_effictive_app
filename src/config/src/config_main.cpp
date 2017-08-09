#include <ros/ros.h>
#include <python2.7/Python.h>
#include "comlib/ulog.h"

#define NODENAME 			"config"
#define CONFIG_PATH 		"/home/su/Workspace/robot/E01S/android-e01s_of/src/config"
#define VERSION_FILE 		"version.yaml"
#define VERSION_TAG			"version"

void load_param(std::string file, std::string ns)
{
	std::string strcmd = "rosparam load " + file + " " + ns;
	system(strcmd.c_str());
}

void dump_param(std::string file, std::string ns)
{
	std::string strcmd = "rosparam dump " + file + " " + ns;
	system(strcmd.c_str());
}

void load_all_param()
{
	std::string file;
	file = file + CONFIG_PATH + "/" + VERSION_FILE;
	load_param(file, VERSION_TAG);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, NODENAME, ros::init_options::NoRosout);
	load_all_param();
	ros::spin();
	return 0;
}
