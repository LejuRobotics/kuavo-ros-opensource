/* 1. First please add data folder parallel with 'build' folder
 *  2. Use remove(SavePATH); before saveVec3() function
 */
#pragma once
#include <fstream>
#include "orientation_tools.h"

#define INPATH "./../data/mpcinput.txt"
#define OUPATH "./../data/mpcoutput.txt"
#define VYPATH_R "./../data/VYPATH_R.txt"
#define VYPATH_L "./../data/VYPATH_L.txt"
#define RYPATH_R "./../data/RYPATH_R.txt"
#define RYPATH_L "./../data/RYPATH_L.txt"

#define VNEXT_BEF_R "./../data/VNEXT_BEF_R.txt"
#define VNEXT_AFT_R "./../data/VNEXT_AFT_R.txt"
#define VNEXT_BEF_L "./../data/VNEXT_BEF_L.txt"
#define VNEXT_AFT_L "./../data/VNEXT_AFT_L.txt"

void saveVec3(const char *path, double timeSec, Vec3 v3);
void saveVx(const char *path, Eigen::VectorXd vx);
Eigen::VectorXd readVx(const char *path);
