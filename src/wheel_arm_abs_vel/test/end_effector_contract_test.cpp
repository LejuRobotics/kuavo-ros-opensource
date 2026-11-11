#include "wheel_arm_abs_vel/end_effector_contract.h"
#include <cassert>
#include <iostream>
using kuavo_common::endEffectorContract;
int main() {
  assert(endEffectorContract(nullptr,63).empty());
  nlohmann::json j={{"robot_version",63},{"types",{"lejuclaw","lejuclaw"}},
                    {"real",true},{"source","machine CAN inventory"}};
  for (int version: {60,61,62,63,200062,300062,400062,400063}) {
    j["robot_version"]=version;
    assert(endEffectorContract(j.dump().c_str(),version).at(0)=="lejuclaw");
  }
  auto reject=[](const nlohmann::json& x) {
    bool threw=false;try {endEffectorContract(x.dump().c_str(),63);} catch(const std::exception&) {threw=true;}
    assert(threw);
  };
  reject(j); // Cross-model contracts must not be reused.
  j["robot_version"]=63;
  j["types"]={"qibeng","qibeng"};
  assert(endEffectorContract(j.dump().c_str(),63).at(0)=="qibeng");
  j["types"]={"lejuclaw","lejuclaw"};
  auto bad=j;bad["types"]={"lejuclaw","heiman"};reject(bad);
  bad=j;bad["types"]={"unknown","unknown"};reject(bad);
  bad=j;bad["types"]={"lejuclaw"};reject(bad);
  bad=j;bad.erase("real");reject(bad);
  bad=j;bad["source"]="";reject(bad);
  std::cout << "PASS shared hardware/controller contract and invalid selection rejection\n";
}
