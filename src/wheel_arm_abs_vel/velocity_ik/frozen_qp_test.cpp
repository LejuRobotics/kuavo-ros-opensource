#include "weighted_qp.h"
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
int main(int argc,char**argv){
 nlohmann::json j;std::ifstream(argv[1])>>j;
 auto matrix=[](const nlohmann::json& a){Eigen::MatrixXd v(a.size(),a[0].size());for(int r=0;r<v.rows();++r)for(int c=0;c<v.cols();++c)v(r,c)=a[r][c];return v;};
 Eigen::MatrixXd H=matrix(j["H"]),C=matrix(j["C"]);Eigen::VectorXd g=matrix(j["g"]),lo=matrix(j["lower"]),hi=matrix(j["upper"]),original=matrix(j["x"]);int n=g.size();
 velocity_ik::WeightedQp solver;bool good=true;
 for(int i=0;i<12;++i){double scale=(i%2)?1.03:1.;Eigen::MatrixXd h=H*scale;Eigen::VectorXd x;int status;bool ok=solver.solve(h,g,lo.head(n),hi.head(n),C.bottomRows(C.rows()-n),lo.tail(lo.size()-n),hi.tail(hi.size()-n),&x,&status);auto s=solver.stats();std::cout<<i<<" ok="<<ok<<" status="<<status<<" native="<<s.native_status<<" iter="<<s.iterations<<" complementarity="<<s.complementarity<<" stationarity="<<s.stationarity<<" violation="<<s.violation;
 if(ok){good &= ((.5*original.dot(h*original)+g.dot(original))-(.5*x.dot(h*x)+g.dot(x)))>3.;
 std::cout<<" objective="<<.5*x.dot(h*x)+g.dot(x)<<" witness_gain="<<(.5*original.dot(h*original)+g.dot(original))-(.5*x.dot(h*x)+g.dot(x));}
 std::cout<<std::endl;good &=ok;
 }
 return good?0:1;
}
