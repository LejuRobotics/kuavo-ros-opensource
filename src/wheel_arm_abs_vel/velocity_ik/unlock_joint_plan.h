#pragma once

#include "unlock_geometry.h"
#include "stop_reserve.h"
#include <Eigen/Cholesky>
#include <atomic>
#include <cstdint>
#include <utility>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <pthread.h>
#include <sched.h>

namespace velocity_ik {

struct UnlockJointGoal {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d elbow = Eigen::Vector3d::Zero();
  bool elbow_valid = false;
};

struct UnlockJointPlan {
  bool valid = false;
  std::string reason;
  std::vector<Eigen::VectorXd> points;
  std::vector<double> durations;
  // Positive quartic Bernstein coefficients of dt/du. Their integral is
  // monotone and C-infinity: changing timing never changes the joint path.
  std::vector<std::array<double,5>> time_weights;
  static double bezier(std::array<double,6> b, int degree, double u) {
    for (int n=degree;n>0;--n)
      for (int i=0;i<n;++i) b[i]=(1.-u)*b[i]+u*b[i+1];
    return b[0];
  }
  static double warpedTime(const std::array<double,5>& w, double u) {
    std::array<double,6> integral{};
    for (int i=0;i<5;++i) integral[i+1]=integral[i]+w[i]/5.;
    return bezier(integral,5,u);
  }
  double minimum_clearance = std::numeric_limits<double>::infinity();

  double duration() const {
    double t = 0.;
    for (double segment : durations) t += segment;
    return t;
  }

  // One common geometric progress variable for all joints. Waypoints retain
  // zero velocity/acceleration; nonuniform dt/du slows only constrained regions.
  void sample(double time, Eigen::VectorXd* q, Eigen::VectorXd* dq,
              Eigen::VectorXd* ddq=nullptr, Eigen::VectorXd* jerk=nullptr) const {
    if (!valid || points.size()!=durations.size()+1 || durations.empty())
      throw std::logic_error("invalid unlock trajectory");
    size_t segment=0;
    time=std::clamp(time,0.,duration());
    while(segment+1<durations.size() && time>durations[segment])
      time-=durations[segment++];
    std::array<double,5> w;
    if(time_weights.empty()) w.fill(durations[segment]);
    else w=time_weights.at(segment);
    double lo=0.,hi=1.,u=std::clamp(time/durations[segment],0.,1.);
    // Safeguarded Newton inversion of an exactly integrated positive clock.
    for(int k=0;k<24;++k) {
      const double error=warpedTime(w,u)-time;
      if(std::abs(error)<1e-12)break;
      if(error>0.)hi=u;else lo=u;
      const double rate=bezier({w[0],w[1],w[2],w[3],w[4],0.},4,u);
      const double candidate=u-error/rate;
      u=candidate>lo && candidate<hi ? candidate : .5*(lo+hi);
    }
    const double W=bezier({w[0],w[1],w[2],w[3],w[4],0.},4,u);
    const double W1=bezier({4*(w[1]-w[0]),4*(w[2]-w[1]),4*(w[3]-w[2]),4*(w[4]-w[3]),0.,0.},3,u);
    const double W2=bezier({12*(w[2]-2*w[1]+w[0]),12*(w[3]-2*w[2]+w[1]),12*(w[4]-2*w[3]+w[2]),0.,0.,0.},2,u);
    const double S=u*u*u*(10.+u*(-15.+6.*u));
    const double S1=30.*u*u*(1.-u)*(1.-u);
    const double S2=60.*u*(1.-u)*(1.-2.*u);
    const double S3=60.-360.*u+360.*u*u;
    const Eigen::VectorXd delta=points[segment+1]-points[segment];
    *q=points[segment]+S*delta;
    *dq=(S1/W)*delta;
    if(ddq)*ddq=(S2/(W*W)-S1*W1/(W*W*W))*delta;
    if(jerk)*jerk=(S3/std::pow(W,3)-3*S2*W1/std::pow(W,4)
        -S1*W2/std::pow(W,4)+3*S1*W1*W1/std::pow(W,5))*delta;
  }

};

// Used only to obtain an unlock endpoint and validate its path. This is NOT
// the online velocity IK. It follows the legacy absolute IK's two-stage
// strategy with the canonical full model/TCPs (the legacy 14-DoF plant cannot
// safely supply joint commands to this model without that adaptation).
class UnlockJointPlanner {
 public:
  struct Config {
    std::vector<int> q_index, v_index;
    std::array<int,14> arms{};
    std::array<std::string,2> hands, elbows, shoulders;
    std::string torso;
    Eigen::VectorXd lower, upper, velocity, acceleration, braking;
    double clearance = .025;
    double speed = 4.;
    double acceleration_cap = 12.;
    double jerk_cap = 150.;
    double execution_clearance = .015; // preview reserve above live 1 cm guard
    double tracking_lag = .04;
    double proximal_delay = .02, wrist_delay = .045;
    double position_tolerance = .025;
    double orientation_tolerance = .12;
    double planning_timeout = 5.;
  };

  UnlockJointPlanner(const ModelPinocchio& model, const UnlockGeometry& geometry,
                     Config config)
      : model_(model), geometry_(geometry), config_(std::move(config)),
        context_(model.CreateDefaultContext()), full_q_(model.GetPositions(*context_)) {}

  UnlockJointPlan plan(const Eigen::VectorXd& start,
                      const std::array<UnlockJointGoal,2>& goals,
                      const std::function<bool()>& cancelled = [] { return false; }) {
    UnlockJointPlan result;
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(config_.planning_timeout);
    cancelled_ = [deadline, cancelled] {
      return cancelled() || std::chrono::steady_clock::now() >= deadline;
    };
    if (!validState(start)) { result.reason = "invalid start/limits"; return result; }
    for (const auto& g : goals) {
      if (!g.position.allFinite() || !g.rotation.allFinite() ||
          (g.rotation.transpose()*g.rotation-Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
          g.rotation.determinant() < .99 || (g.elbow_valid && !g.elbow.allFinite())) {
        result.reason = "invalid goal"; return result;
      }
    }
    if (!segmentClear(start, start, &result.minimum_clearance)) {
      result.reason = "start lacks body/tool clearance"; return result;
    }
    Eigen::VectorXd endpoint;
    if (solveEndpoint(start, goals, &endpoint) &&
        segmentClear(start, endpoint, &result.minimum_clearance)) {
      result.points = {start, endpoint};
      finish(&result);
      return result;
    }

    // Retry a bounded set of front/outward waypoints. No unverified shoulder
    // angle is sent to hardware. Each endpoint and both complete joint paths
    // must pass the same geometric certificate.
    for (int attempt = 0; attempt < 3 && !cancelled_(); ++attempt) {
      setQ(start);
      auto waypoint_goals = goals;
      const auto& torso = model_.GetFrameByName(config_.torso);
      for (int arm = 0; arm < 2; ++arm) {
        const auto pose = model_.GetFrameByName(config_.hands[arm]).CalcPose(*context_, torso);
        const auto shoulder = model_.GetFrameByName(config_.shoulders[arm]).CalcPose(*context_, torso);
        auto& g = waypoint_goals[arm];
        g.position = pose.translation();
        g.position.x() = std::max(g.position.x(), geometry_.body_front + .12 + .04*attempt);
        const double side = std::max(std::abs(g.position.y()), geometry_.body_side + .12 + .025*attempt);
        g.position.y() = arm == 0 ? side : -side;
        g.position.z() = std::max(g.position.z(), shoulder.translation().z() - .40);
        g.rotation = pose.rotation().matrix();
        g.elbow_valid = false;
      }
      Eigen::VectorXd waypoint;
      double path_clearance = std::numeric_limits<double>::infinity();
      if (solveEndpoint(start, waypoint_goals, &waypoint) &&
          segmentClear(start, waypoint, &path_clearance) &&
          solveEndpoint(waypoint, goals, &endpoint) &&
          segmentClear(waypoint, endpoint, &path_clearance)) {
        result.points = {start, waypoint, endpoint};
        result.minimum_clearance = path_clearance;
        finish(&result);
        return result;
      }
    }
    result.reason = cancelled_() ? "planning cancelled/timeout" :
        "no accurate collision-free unlock path";
    return result;
  }

  // X+A return has a known joint goal. Use exactly the same path and timing
  // checks as acquisition, without solving an unnecessary Cartesian endpoint.
  UnlockJointPlan planJoint(const Eigen::VectorXd& start,const Eigen::VectorXd& goal,
                           const std::function<bool()>& cancelled) {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(config_.planning_timeout);
    cancelled_=[deadline,cancelled]{return cancelled()||std::chrono::steady_clock::now()>=deadline;};
    UnlockJointPlan result;
    if(!validState(start)||!validState(goal)) {result.reason="invalid return start/goal";return result;}
    if(!segmentClear(start,goal,&result.minimum_clearance)) {
      result.reason="return joint path lacks body/tool clearance";return result;
    }
    result.points={start,goal};finish(&result);return result;
  }

  // Conservative interval certification: enclose every link over the WHOLE
  // joint interval by lever*angle inflation. Endpoints alone are insufficient.
  bool segmentClear(const Eigen::VectorXd& a, const Eigen::VectorXd& b,
                    double* minimum) {
    if (!validState(a) || !validState(b)) return false;
    double largest = 0.;
    for (int i : config_.arms) largest = std::max(largest, std::abs(b[i]-a[i]));
    const int intervals = std::max(1, static_cast<int>(std::ceil(largest/.005)));
    for (int k = 0; k < intervals; ++k) {
      if (cancelled_ && cancelled_()) return false;
      const double fraction = (k+.5)/intervals;
      const Eigen::VectorXd middle = a + fraction*(b-a);
      std::array<std::array<double,7>,2> excursion{};
      for (int arm = 0; arm < 2; ++arm)
        for (int j = 0; j < 7; ++j) {
          const int i = config_.arms[7*arm+j];
          excursion[arm][j] = .5*std::abs(b[i]-a[i])/intervals;
        }
      setQ(middle);
      const double distance = geometry_.clearance(model_, *context_, config_.torso, excursion);
      *minimum = std::min(*minimum, distance);
      if (!std::isfinite(distance) || distance < config_.clearance) return false;
    }
    return true;
  }

 private:
  using Matrix14 = Eigen::Matrix<double,14,14>;
  using Vector14 = Eigen::Matrix<double,14,1>;
  bool validState(const Eigen::VectorXd& q) const {
    return q.size() == config_.lower.size() && q.allFinite() &&
        (q.array() >= config_.lower.array()).all() &&
        (q.array() <= config_.upper.array()).all();
  }
  void setQ(const Eigen::VectorXd& q) {
    for (size_t i = 0; i < config_.q_index.size(); ++i) full_q_[config_.q_index[i]] = q[i];
    model_.SetPositions(context_.get(), full_q_);
  }
  Eigen::Matrix<double,3,14> select(const Eigen::MatrixXd& full) const {
    Eigen::Matrix<double,3,14> result;
    for (int j = 0; j < 14; ++j) result.col(j) = full.col(config_.v_index[config_.arms[j]]);
    return result;
  }
  double objective(const Eigen::VectorXd& q, const Eigen::VectorXd& anchor,
                   const std::array<UnlockJointGoal,2>& goals, int stage,
                   Matrix14* h = nullptr, Vector14* rhs = nullptr) {
    setQ(q);
    const auto& torso = model_.GetFrameByName(config_.torso);
    double cost = 0.;
    if (h) { h->setIdentity(); *h *= 1e-5; rhs->setZero(); }
    const auto add = [&](const Eigen::Vector3d& error,
                         const Eigen::Matrix<double,3,14>& jacobian, double weight) {
      cost += weight*error.squaredNorm();
      if (h) { *h += weight*jacobian.transpose()*jacobian; *rhs += weight*jacobian.transpose()*error; }
    };
    for (int arm = 0; arm < 2; ++arm) {
      const auto& frame = model_.GetFrameByName(config_.hands[arm]);
      const auto pose = frame.CalcPose(*context_, torso);
      Eigen::MatrixXd jac;
      model_.spatialJacobian(*context_, frame, Eigen::Vector3d::Zero(), torso, torso, &jac);
      add(goals[arm].position-pose.translation(), select(jac.bottomRows(3)), 100.);
      if (stage == 1) {
        Eigen::AngleAxisd aa(goals[arm].rotation*pose.rotation().matrix().transpose());
        add(aa.angle()*aa.axis(), select(jac.topRows(3)), 4.);
      }
      if (goals[arm].elbow_valid) {
        const auto& elbow = model_.GetFrameByName(config_.elbows[arm]);
        const auto ep = elbow.CalcPose(*context_, torso).translation();
        model_.translationJacobian(*context_, elbow, Eigen::Vector3d::Zero(), torso, torso, &jac);
        add(goals[arm].elbow-ep, select(jac), stage == 0 ? 1. : .02);
      }
    }
    for (int j = 0; j < 14; ++j) {
      const double weight = stage == 0 ? (j%7 >= 4 ? .1 : .001) : 1e-5;
      const double error = anchor[config_.arms[j]]-q[config_.arms[j]];
      cost += weight*error*error;
      if (h) { (*h)(j,j) += weight; (*rhs)[j] += weight*error; }
    }
    return cost;
  }
  bool solveEndpoint(const Eigen::VectorXd& seed,
                     const std::array<UnlockJointGoal,2>& goals, Eigen::VectorXd* result) {
    Eigen::VectorXd q = seed;
    for (int stage = 0; stage < 2; ++stage) {
      const Eigen::VectorXd anchor = q;
      for (int iteration = 0; iteration < 160; ++iteration) {
        if (cancelled_()) return false;
        Matrix14 h; Vector14 rhs;
        const double cost = objective(q, anchor, goals, stage, &h, &rhs);
        Vector14 step = h.ldlt().solve(rhs);
        if (!step.allFinite()) return false;
        if (step.cwiseAbs().maxCoeff() < 1e-7) break;
        step *= std::min(1., .10/step.cwiseAbs().maxCoeff());
        bool improved = false;
        for (double alpha = 1.; alpha >= 1./128.; alpha *= .5) {
          Eigen::VectorXd next = q;
          for (int j = 0; j < 14; ++j) {
            const int i = config_.arms[j];
            next[i] = std::clamp(q[i]+alpha*step[j], config_.lower[i], config_.upper[i]);
          }
          if (objective(next, anchor, goals, stage) < cost - 1e-12) {
            q = next; improved = true; break;
          }
        }
        if (!improved) break;
      }
    }
    setQ(q);
    const auto& torso = model_.GetFrameByName(config_.torso);
    for (int arm = 0; arm < 2; ++arm) {
      const auto pose = model_.GetFrameByName(config_.hands[arm]).CalcPose(*context_, torso);
      const double angle = Eigen::AngleAxisd(goals[arm].rotation*pose.rotation().matrix().transpose()).angle();
      if ((goals[arm].position-pose.translation()).norm() > config_.position_tolerance ||
          angle > config_.orientation_tolerance) return false;
    }
    *result = q;
    return validState(q);
  }
  // Timing preview, not a replacement for the live measured stopping guard.
  // Uses the same discrete braking model and a configurable nominal servo lag.
  bool timingClear(const UnlockJointPlan& p, double step=.02) {
    Eigen::VectorXd q,v,a,j,m,mv;
    for(double t=0.;t<p.duration()+step;t+=step) {
      if(cancelled_())return false;
      p.sample(std::min(t,p.duration()),&q,&v,&a,&j);
      p.sample(std::max(0.,t-config_.tracking_lag),&m,&mv);
      std::array<std::array<double,7>,2> excursion{};
      for(int arm=0;arm<2;++arm)for(int k=0;k<7;++k) {
        const int i=config_.arms[7*arm+k];
        if(std::abs(v[i])>std::min(config_.speed,config_.velocity[i])+1e-8 ||
           std::abs(a[i])>std::min(config_.acceleration_cap,config_.acceleration[i])+1e-8 ||
           std::abs(j[i])>config_.jerk_cap+1e-8)return false;
        const double speed=std::max(std::abs(v[i]),std::abs(mv[i]));
        const double brake=config_.braking.size()==q.size() ? config_.braking[i] : config_.acceleration[i];
        excursion[arm][k]=std::abs(q[i]-m[i])+speed*.002+
            discreteStopDistance(speed,k<4?config_.proximal_delay:config_.wrist_delay,
                std::abs(a[i]),.8*std::min(brake,config_.acceleration[i]),.002);
      }
      for(const auto* state:{&q,&m}) {
        setQ(*state);
        std::array<int,14> indices;
        for(int i=0;i<14;++i)indices[i]=config_.q_index[config_.arms[i]];
        if(!geometry_.certifyBox(model_,context_.get(),&full_q_,indices,
                                config_.torso,excursion,config_.execution_clearance))return false;
      }
    }
    return true;
  }
  void finish(UnlockJointPlan* plan) {
    // Optimize each straight geometric segment independently. A positive
    // Bernstein clock preserves its swept-path certificate exactly.
    for(size_t k=1;k<plan->points.size();++k) {
      double duration=.25;
      for(int i:config_.arms) {
        const double d=std::abs(plan->points[k][i]-plan->points[k-1][i]);
        duration=std::max(duration,1.875*d/std::min(config_.speed,config_.velocity[i]));
        duration=std::max(duration,std::sqrt(5.774*d/std::min(config_.acceleration_cap,config_.acceleration[i])));
        duration=std::max(duration,std::cbrt(60.*d/config_.jerk_cap));
      }
      const double fastest=duration;
      UnlockJointPlan segment;
      segment.valid=true;segment.points={plan->points[k-1],plan->points[k]};
      segment.durations={duration};segment.time_weights.resize(1);
      auto assign=[&](const std::array<double,5>& w) {
        segment.time_weights[0]=w;
        segment.durations[0]=UnlockJointPlan::warpedTime(w,1.);
      };
      std::array<double,5> weights;weights.fill(duration);
      assign(weights);
      // First find a feasible clock; then reduce time locally, instead of
      // making the whole path run at its tightest region's speed.
      bool feasible=false;
      for(int attempt=0;attempt<20 && !cancelled_();++attempt) {
        if(timingClear(segment)){feasible=true;break;}
        for(auto& value:weights)value*=1.15;
        assign(weights);
      }
      if(!feasible) {plan->valid=false;plan->reason="no feasible unlock timing/timeout";return;}
      for(int pass=0;pass<2 && !cancelled_();++pass)for(int index:{4,0,3,1,2}) {
        double low=.5*fastest,high=weights[index];
        const auto accepted=weights;
        for(int iteration=0;iteration<6 && !cancelled_();++iteration) {
          auto candidate=accepted;candidate[index]=.5*(low+high);assign(candidate);
          if(timingClear(segment))high=candidate[index];else low=candidate[index];
        }
        weights=accepted;weights[index]=high;assign(weights);
      }
      // Dense final preview; numerical optimization cannot disable validation.
      while(!cancelled_() && !timingClear(segment,.004)) {
        for(auto& value:weights)value*=1.05;
        assign(weights);
      }
      if(cancelled_()){plan->valid=false;plan->reason="timing planning cancelled/timeout";return;}
      plan->durations.push_back(segment.duration());
      plan->time_weights.push_back(weights);
    }
    plan->valid=true;plan->reason="verified path and stopping-aware timing";
  }
  const ModelPinocchio& model_;
  const UnlockGeometry& geometry_;
  Config config_;
  std::unique_ptr<ModelContext> context_;
  Eigen::VectorXd full_q_;
  std::function<bool()> cancelled_;
};

// A single background owner with generation cancellation. Never waits for an
// IK solve in the reference thread; stale results cannot complete a new unlock.
class UnlockPlanningWorker {
 public:
  explicit UnlockPlanningWorker(std::unique_ptr<UnlockJointPlanner> planner)
      : planner_(std::move(planner)), thread_([this] { run(); }) {}
  ~UnlockPlanningWorker() {
    stop_.store(true); cancel(); cv_.notify_one();
    if (thread_.joinable()) thread_.join();
  }
  uint64_t submit(const Eigen::VectorXd& start, const std::array<UnlockJointGoal,2>& goals) {
    std::lock_guard<std::mutex> lock(mutex_);
    const uint64_t token = ++token_;
    request_ = {token, start, goals}; pending_ = true;
    cv_.notify_one(); return token;
  }
  uint64_t submitJoint(const Eigen::VectorXd& start,const Eigen::VectorXd& goal) {
    std::lock_guard<std::mutex> lock(mutex_);
    const uint64_t token=++token_;
    request_=Request{};request_.token=token;request_.start=start;
    request_.joint_goal=goal;request_.joint_return=true;pending_=true;
    cv_.notify_one();return token;
  }
  void cancel() { ++token_; }
  bool take(uint64_t token, UnlockJointPlan* result) {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock || result_token_ != token || token_.load() != token) return false;
    *result = std::move(result_); result_token_ = 0; return true;
  }
 private:
  struct Request {
    uint64_t token=0;Eigen::VectorXd start;std::array<UnlockJointGoal,2> goals;
    Eigen::VectorXd joint_goal;bool joint_return=false;
  };
  void run() {
    // The launch wrapper may start the process under FIFO. A planning worker
    // must never inherit that priority and monopolize the servo CPU.
    sched_param scheduling{};
    const bool scheduling_ok = pthread_setschedparam(pthread_self(), SCHED_OTHER, &scheduling) == 0;
    pthread_setname_np(pthread_self(), "unlock_plan");
    while (!stop_.load()) {
      Request r;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [&] { return pending_ || stop_.load(); });
        if (stop_.load()) break;
        r = request_; pending_ = false;
      }
      UnlockJointPlan result;
      try {
        if (!scheduling_ok) result.reason = "cannot lower planning thread priority";
        else if(r.joint_return)
          result=planner_->planJoint(r.start,r.joint_goal,[&]{return stop_.load()||token_.load()!=r.token;});
        else result = planner_->plan(r.start, r.goals, [&] { return stop_.load() || token_.load() != r.token; });
      }
      catch (const std::exception& e) { result.reason = e.what(); }
      std::lock_guard<std::mutex> lock(mutex_);
      if (token_.load() == r.token) { result_ = std::move(result); result_token_ = r.token; }
    }
  }
  std::unique_ptr<UnlockJointPlanner> planner_;
  std::atomic<bool> stop_{false};
  std::atomic<uint64_t> token_{0};
  std::mutex mutex_;
  std::condition_variable cv_;
  bool pending_ = false;
  Request request_;
  uint64_t result_token_ = 0;
  UnlockJointPlan result_;
  std::thread thread_;
};
}  // namespace velocity_ik
