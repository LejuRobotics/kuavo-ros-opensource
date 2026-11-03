#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <sys/wait.h>
#include <unistd.h>
using J=nlohmann::json;
int main(int argc,char**argv){
 if(argc!=5)throw std::runtime_error("direct_tcp_report_check REPORT_BINARY CONFIG COMPLETE_EPISODES_TRACE OUTPUT_DIR");
 std::ifstream in(argv[3]);std::vector<J> rows;std::string line;while(std::getline(in,line))rows.push_back(J::parse(line));std::filesystem::create_directories(argv[4]);J result;
 for(const auto& test:{"missing_window","missing_raw","joint_order","quaternion","integer_health","wrong_settle_twist"}){
  auto modified=rows;bool changed=false;for(auto& r:modified){
   if(std::string(test)=="missing_window"&&r.value("phase","")=="motion"&&r.value("source_time",0.)>170.){r["phase"]="excluded";changed=true;}
   if(!changed&&std::string(test)=="wrong_settle_twist"&&r.value("kind","")=="mapper"&&r.value("phase","")=="settle"){r["command"][0][7]=1.;changed=true;}
   if(!changed&&r.value("kind","")=="mapper"&&r.value("phase","")=="motion"){
    if(std::string(test)=="missing_raw"){r.erase("raw");changed=true;}
    if(std::string(test)=="quaternion"){r["command"][0][6]=2.;changed=true;}
   }
   if(!changed&&r.value("kind","")=="reference"&&r.value("phase","")=="motion"){
    if(std::string(test)=="joint_order"){std::swap(r["joint_names"][7],r["joint_names"][8]);changed=true;}
    if(std::string(test)=="integer_health"){r["solver_valid"]=2;changed=true;}
   }
  }
  if(!changed)throw std::runtime_error("Test did not corrupt its intended evidence");
  const std::string input=std::string(argv[4])+"/"+test+".jsonl",output=std::string(argv[4])+"/"+test+".json";
  {std::ofstream out(input);for(const auto& r:modified)out<<r.dump()<<'\n';}
  const auto pid=fork();if(pid==0){freopen("/dev/null","w",stdout);freopen("/dev/null","w",stderr);execl(argv[1],argv[1],input.c_str(),output.c_str(),argv[2],"episodes",nullptr);_exit(127);}int status;waitpid(pid,&status,0);std::ifstream file(output);J r;file>>r;
  const bool health=std::string(test)=="integer_health",twist=std::string(test)=="wrong_settle_twist";
  const bool passed=WIFEXITED(status)&&((health||twist)?WEXITSTATUS(status)==0&&r["acceptance"][health?"all_sampled_motion_and_settle_healthy":"final_geometry_and_domain_consistent"]==false:WEXITSTATUS(status)==2&&r["status"]=="INCOMPLETE")&&r["acceptance"]["all_required_gates_pass"]==false;
  result[test]={{"passed",passed},{"exit_code",WIFEXITED(status)?WEXITSTATUS(status):-1},{"report_status",r.value("status","")}};
  std::filesystem::remove(input);if(!passed)throw std::runtime_error(std::string("Report negative test failed: ")+test);
 }
 std::ofstream out(std::string(argv[4])+"/checks.json");out<<result.dump(2)<<'\n';std::cout<<result.dump(2)<<std::endl;
}
