#include "runtime_timing_probe.h"
#include "weighted_qp.h"
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <nlohmann/json.hpp>
#include <proxsuite/proxqp/dense/dense.hpp>
#include <map>
#include <mutex>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <stdexcept>
namespace velocity_ik {
namespace pq=proxsuite::proxqp;
using Clock=std::chrono::steady_clock;
static double micros(Clock::time_point a,Clock::time_point b){return std::chrono::duration<double,std::micro>(b-a).count();}
static double nanos(Clock::time_point t){return std::chrono::duration<double,std::nano>(t.time_since_epoch()).count();}
ResultCheck checkQpResult(const Eigen::MatrixXd& H,const Eigen::VectorXd& g,
                         const Eigen::MatrixXd& C,const Eigen::VectorXd& lower,
                         const Eigen::VectorXd& upper,const Eigen::VectorXd& x,
                         const Eigen::VectorXd& z){
 ResultCheck r;const int n=g.size(),m=C.rows();
 if(n<=0||H.rows()!=n||H.cols()!=n||C.cols()!=n||lower.size()!=m||upper.size()!=m||x.size()!=n||z.size()!=m||
    !H.allFinite()||!g.allFinite()||!C.allFinite()||!lower.allFinite()||!upper.allFinite()||!x.allFinite()||!z.allFinite())return r;
 const Eigen::VectorXd cx=C*x,stationarity=H*x+g+C.transpose()*z;
 if(!cx.allFinite()||!stationarity.allFinite())return r;
 r.violation=m?std::max(0.0,std::max((lower-cx).maxCoeff(),(cx-upper).maxCoeff())):0;
 r.stationarity=stationarity.lpNorm<Eigen::Infinity>();r.complementarity=0;
 for(int i=0;i<m;++i){
  const double bound=z(i)>=0?upper(i):lower(i);
  const double value=std::abs(bound)<1e19?std::abs(z(i)*(cx(i)-bound)):std::abs(z(i));
  if(!std::isfinite(value))return ResultCheck{};
  r.complementarity=std::max(r.complementarity,value);
 }
 if(!std::isfinite(r.violation)||!std::isfinite(r.stationarity)||!std::isfinite(r.complementarity))return ResultCheck{};
 r.accepted=r.violation<=2e-8&&r.stationarity<=2e-7&&r.complementarity<=2e-7;
 r.status=r.accepted?0:-1100;return r;
}
struct WeightedQp::Impl {
 struct State {
  pq::dense::QP<double> qp;bool initialized=false;
  Eigen::MatrixXd eq,ci;Eigen::VectorXd b,lower,upper;
  State(int n,int m):qp(n,0,n+m,false,pq::DenseBackend::PrimalDualLDLT),eq(0,n),ci(n+m,n),b(0),lower(n+m),upper(n+m){
   ci.topRows(n).setIdentity();
   qp.settings.eps_abs=1e-9;qp.settings.eps_rel=0;qp.settings.max_iter=100;
   qp.settings.max_iter_in=100;qp.settings.default_rho=1e-9;
   qp.settings.check_duality_gap=true;qp.settings.eps_duality_gap_abs=1e-8;qp.settings.eps_duality_gap_rel=0;
   qp.settings.verbose=false;qp.settings.primal_infeasibility_solving=false;
  }
 };
 struct Failure {Eigen::MatrixXd H,C;Eigen::VectorXd g,lower,upper,x,z;Stats stats;};
 std::map<std::pair<int,int>,std::unique_ptr<State>> solvers;
 std::mutex mutex;Stats stats;bool dimensions_restricted=false,solve_started=false;
 std::mutex failure_mutex;std::condition_variable failure_cv;std::unique_ptr<Failure> failure;
 bool frozen_failure=false,stopping=false;std::string failure_path;
 std::thread writer;
 Impl():failure_path(std::getenv("PINOCCHIO_QP_FAILURE_PATH")?std::getenv("PINOCCHIO_QP_FAILURE_PATH"):""),writer([this]{writeFailure();}){}
 ~Impl(){ {std::lock_guard<std::mutex> l(failure_mutex);stopping=true;}failure_cv.notify_one();writer.join();}
 void writeFailure(){
  std::unique_ptr<Failure> f;
  {std::unique_lock<std::mutex> l(failure_mutex);failure_cv.wait(l,[this]{return stopping||failure!=nullptr;});f=std::move(failure);}
  if(!f)return;
  auto matrix=[](const Eigen::MatrixXd& v){nlohmann::json rows=nlohmann::json::array();for(int r=0;r<v.rows();++r){std::vector<double> row;for(int c=0;c<v.cols();++c)row.push_back(v(r,c));rows.push_back(row);}return rows;};
  nlohmann::json j={{"record_kind","first_postsolve_rejection"},{"scope","native or independent result rejection; excludes preflight invalid inputs"},{"H",matrix(f->H)},{"g",matrix(f->g)},{"C",matrix(f->C)},{"lower",matrix(f->lower)},{"upper",matrix(f->upper)},{"x",matrix(f->x)},{"z",matrix(f->z)},{"native_status",f->stats.native_status},{"acceptance_status",f->stats.acceptance_status}};
  std::ofstream(failure_path)<<j.dump(2);
 }
};
WeightedQp::WeightedQp():impl_(new Impl){}
WeightedQp::~WeightedQp()=default;
void WeightedQp::prepareWorkspaces(std::initializer_list<std::pair<int,int>> dimensions){
 std::lock_guard<std::mutex> lock(impl_->mutex);
 if(impl_->solve_started || impl_->dimensions_restricted)
  throw std::logic_error("QP dimensions must be bound once before solving");
 if(dimensions.size()==0)throw std::invalid_argument("empty QP family set");
 if(std::any_of(dimensions.begin(),dimensions.end(),
                [](const auto& d){return d.first<=0 || d.second<0;}))
  throw std::invalid_argument("invalid QP dimensions");
 for(const auto& d:dimensions)
  if(!impl_->solvers.count(d))impl_->solvers.emplace(d,std::make_unique<Impl::State>(d.first,d.second));
 impl_->dimensions_restricted=true;
}
WeightedQp::Stats WeightedQp::stats()const{std::lock_guard<std::mutex> lock(impl_->mutex);return impl_->stats;}
bool WeightedQp::solve(const Eigen::MatrixXd& H,const Eigen::VectorXd& g,const Eigen::VectorXd& lo,const Eigen::VectorXd& hi,const Eigen::MatrixXd& C,const Eigen::VectorXd& l,const Eigen::VectorXd& u,Eigen::VectorXd* x,int* status,Stats* statistics){
 runtime_probe::point(60);
 const auto begin=Clock::now();std::lock_guard<std::mutex> lock(impl_->mutex);
 runtime_probe::point(61);
 const auto acquired=Clock::now();impl_->solve_started=true;
 impl_->stats=Stats{};auto& stats=impl_->stats;stats.begin_ns=nanos(begin);stats.lock_wait_us=micros(begin,acquired);
 struct Finish {Stats* dest;Stats& stats;Clock::time_point begin;int* status;~Finish(){auto end=Clock::now();stats.total_us=micros(begin,end);stats.end_ns=nanos(end);stats.acceptance_status=*status;if(dest)*dest=stats;}};
 // The destructor records solve statistics on every return path.
 [[maybe_unused]] Finish finish{statistics,stats,begin,status};
 *status=-1004;const int n=g.size(),m=C.rows();
 if(n<=0||H.rows()!=n||H.cols()!=n||lo.size()!=n||hi.size()!=n||C.cols()!=n||l.size()!=m||u.size()!=m||!H.allFinite()||!g.allFinite()||!C.allFinite()||!lo.allFinite()||!hi.allFinite()||!l.allFinite()||!u.allFinite()||(lo.array()>hi.array()).any()||(l.array()>u.array()).any())return false;
 auto found=impl_->solvers.find({n,m});
 if(found==impl_->solvers.end()){
  if(impl_->dimensions_restricted){*status=-1005;return false;}
  found=impl_->solvers.emplace(std::make_pair(n,m),std::make_unique<Impl::State>(n,m)).first;
  stats.workspace_created=true;
 }
 auto& s=*found->second;auto& q=s.qp;s.ci.bottomRows(m)=C;s.lower.head(n)=lo;s.lower.tail(m)=l;s.upper.head(n)=hi;s.upper.tail(m)=u;
 q.settings.initial_guess=s.initialized?pq::InitialGuessStatus::COLD_START_WITH_PREVIOUS_RESULT:pq::InitialGuessStatus::NO_INITIAL_GUESS;
 runtime_probe::point(62);
 const auto update_begin=Clock::now();stats.prepare_us=micros(acquired,update_begin);
 if(s.initialized)q.update(H,g,s.eq,s.b,s.ci,s.lower,s.upper,true);
 else {q.init(H,g,s.eq,s.b,s.ci,s.lower,s.upper,true);q.work.active_set_up.setConstant(false);q.work.active_set_low.setConstant(false);q.work.active_inequalities.setConstant(false);s.initialized=true;}
 runtime_probe::point(63);
 const auto solve_begin=Clock::now();stats.update_us=micros(update_begin,solve_begin);q.solve();
 runtime_probe::point(64);
 const auto check_begin=Clock::now();stats.solve_us=micros(solve_begin,check_begin);
 stats.native_status=static_cast<int>(q.results.info.status);stats.iterations=q.results.info.iter;stats.primal=q.results.info.pri_res;stats.dual=q.results.info.dua_res;
 const ResultCheck check=checkQpResult(H,g,s.ci,s.lower,s.upper,q.results.x,q.results.z);
 runtime_probe::point(65);
 stats.check_us=micros(check_begin,Clock::now());stats.violation=check.violation;stats.stationarity=check.stationarity;stats.complementarity=check.complementarity;
 *status=q.results.info.status==pq::QPSolverOutput::PROXQP_SOLVED?check.status:-1101;
 stats.accepted=*status==0;stats.acceptance_status=*status;
 if(!stats.accepted){
  if(!impl_->frozen_failure&&!impl_->failure_path.empty()){
   impl_->frozen_failure=true;
   auto f=std::make_unique<Impl::Failure>(Impl::Failure{H,s.ci,g,s.lower,s.upper,q.results.x,q.results.z,stats});
   {std::lock_guard<std::mutex> lck(impl_->failure_mutex);impl_->failure=std::move(f);}impl_->failure_cv.notify_one();
  }
  return false;
 }
 *x=q.results.x;return true;
}
}
