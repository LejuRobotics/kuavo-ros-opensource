/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

#include "humanoid_interface/common/Types.h"

namespace ocs2
{
  namespace humanoid
  {

    enum ArmControlMode {
      KEEP = 0,
      AUTO_SWING = 1,
      EXTERN_CONTROL = 2
    };

    enum ModeNumber
    {
      FF = 0,
      FH = 1,
      FT = 2,
      FS = 3,
      HF = 4,
      HH = 5,
      HT = 6,
      HS = 7,
      TF = 8,
      TH = 9,
      TT = 10,
      TS = 11,
      SF = 12,
      SH = 13,
      ST = 14,
      SS = 15
    };

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    /*******************************************************************
    FF(0): 00000000  HF(4): 00110000  TF(8):  11000000  SF(12): 11110000
    FH(1): 00000011  HH(5): 00110011  TH(9):  11000011  SH(13): 11110011
    FT(2): 00001100  HT(6): 00111100  TT(10): 11001100  ST(14): 11111100
    FS(3): 00001111  HS(7): 00111111  TS(11): 11001111  SS(15): 11111111
    ********************************************************************/
    // 只适用于单脚4接触点的情况
    inline contact_flag_t modeNumber2StanceLeg(const size_t &modeNumber)
    {
      if (modeNumber < 0 || modeNumber > 15)
        std::cerr << "Invalid mode number: " << modeNumber << std::endl;
      contact_flag_t contact_flags;
      for (int i = 0; i < 4; ++i)
      {
        contact_flags[2 * i] = (modeNumber & (1 << (3 - i))) != 0; // 单脚左触点
        contact_flags[2 * i + 1] = contact_flags[2 * i];           // 单脚右触点
      }
      return contact_flags;
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    // 只适用于单脚4接触点的情况
    inline size_t stanceLeg2ModeNumber(const contact_flag_t &stanceLegs)
    {
      return static_cast<size_t>(stanceLegs[0]) * 8 + static_cast<size_t>(stanceLegs[2]) * 4 + static_cast<size_t>(stanceLegs[4]) * 2 + static_cast<size_t>(stanceLegs[6]);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    inline std::string modeNumber2String(const size_t &modeNumber)
    {
      // build the map from mode number to name
      std::map<size_t, std::string> modeToName;
      modeToName[FF] = "FF";
      modeToName[FH] = "FH";
      modeToName[FT] = "FT";
      modeToName[FS] = "FS";
      modeToName[HF] = "HF";
      modeToName[HH] = "HH";
      modeToName[HT] = "HT";
      modeToName[HS] = "HS";
      modeToName[TF] = "TF";
      modeToName[TH] = "TH";
      modeToName[TT] = "TT";
      modeToName[TS] = "TS";
      modeToName[SF] = "SF";
      modeToName[SH] = "SH";
      modeToName[ST] = "ST";
      modeToName[SS] = "SS";

      return modeToName[modeNumber];
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    inline size_t string2ModeNumber(const std::string &modeString)
    {
      // build the map from name to mode number
      std::map<std::string, size_t> nameToMode;
      nameToMode["FF"] = FF;
      nameToMode["FH"] = FH;
      nameToMode["FT"] = FT;
      nameToMode["FS"] = FS;
      nameToMode["HF"] = HF;
      nameToMode["HH"] = HH;
      nameToMode["HT"] = HT;
      nameToMode["HS"] = HS;
      nameToMode["TF"] = TF;
      nameToMode["TH"] = TH;
      nameToMode["TT"] = TT;
      nameToMode["TS"] = TS;
      nameToMode["SF"] = SF;
      nameToMode["SH"] = SH;
      nameToMode["ST"] = ST;
      nameToMode["SS"] = SS;

      return nameToMode[modeString];
    }

  } // namespace humanoid
} // end of namespace ocs2
