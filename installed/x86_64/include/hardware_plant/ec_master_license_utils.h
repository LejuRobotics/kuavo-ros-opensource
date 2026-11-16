#pragma once

#include <algorithm>
#include <cctype>
#include <string>

// Validate format length only; MAC binding and feature entitlement are checked
// by ecatSetLicenseKey(). Never print the key itself.
inline std::string normalizeEcMasterLicense(std::string key)
{
    key.erase(std::remove_if(key.begin(), key.end(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    }), key.end());
    if (key.size() != 26 && key.size() != 53)
    {
        key.clear();
    }
    return key;
}
