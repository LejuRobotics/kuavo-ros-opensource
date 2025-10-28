#ifndef __ECLOGFILE_H__
#define __ECLOGFILE_H__

#include <iostream>
#include <fstream>
#include <vector>
#include <regex>
#include <map>
#include "EcType.h"
#include "EcNotification.h"

bool readWkcErrorFromLog(const std::string& logFilePath, std::vector<EC_T_WKCERR_DESC>& error_list);
bool readRedundancyLineFromLog(const std::string& logFilePath);
#endif /* INC_ECLOGFILE */ 