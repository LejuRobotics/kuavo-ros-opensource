#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <set>
using J=nlohmann::json;
int main(int argc,char**argv){
 if(argc!=5){std::cerr<<"direct_tcp_acceptance EPISODES.json REGRESSION.json SYNTHETIC.json OUTPUT.json\n";return 2;}
 J result;std::set<std::string> types;bool complete=true,pass=true;J common;
 const std::string expected[3]={"episodes","regression","synthetic"};
 for(int i=0;i<3;++i){std::ifstream f(argv[i+1]);if(!f){complete=false;result[expected[i]]="MISSING";continue;}J r;try{f>>r;}catch(const J::exception& e){complete=false;result[expected[i]]=e.what();continue;}
  const bool c=r.value("trial_type","")==expected[i]&&r.value("coverage_complete",false)&&r.contains("acceptance")&&r["acceptance"].contains("all_required_gates_pass");
  complete=complete&&c;const bool p=c&&r["acceptance"]["all_required_gates_pass"].get<bool>();pass=pass&&p;result[expected[i]]=r.value("status","INCOMPLETE");
  if(!r.contains("config")){complete=false;continue;}if(common.is_null())common=r["config"];else complete=complete&&common==r["config"];
 }
 result["all_required_gates_pass"]=complete&&pass;result["status"]=!complete?"INCOMPLETE":pass?"PASS":"FAIL";
 std::ofstream out(argv[4]);out<<result.dump(2)<<'\n';std::cout<<result.dump(2)<<std::endl;return complete?(pass?0:1):2;
}
