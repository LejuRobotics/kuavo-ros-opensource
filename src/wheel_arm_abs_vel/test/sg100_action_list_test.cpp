#include "wheel_arm_abs_vel/sg100/SG100GestureLibrary.h"
#include <kuavo_msgs/SG100HandCommand.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

using HighlyDynamic::SG100GestureLibrary;
using HighlyDynamic::SG100HandPose;
using HighlyDynamic::CurveGesture;
namespace {
int failures = 0, checks = 0;
void expect(bool condition, const std::string& message) {
  ++checks;
  if (!condition) { ++failures; std::printf("FAIL: %s\n", message.c_str()); }
}
bool near(float a, float b) { return std::abs(a-b) < 1e-5f; }
void writeYaml(const std::string& path, const YAML::Node& node) {
  std::ofstream stream(path); stream << node;
}
YAML::Node validAction() {
  YAML::Node entry;
  entry["name"] = "radian_pose";
  for (int row=0; row<2; ++row) {
    YAML::Node values;
    values.push_back(row * 100.0f);
    for (int joint=0; joint<11; ++joint) values.push_back(-0.4f + .05f*joint + .2f*row);
    entry["action_list"].push_back(values);
  }
  return entry;
}
}
int main(int argc, char** argv) {
  if (argc != 2) { std::fprintf(stderr,"expected fixture directory\n"); return 2; }
  char temp[] = "/tmp/sg100-action-list-XXXXXX";
  if (!mkdtemp(temp)) return 2;
  const std::string file = std::string(temp)+"/test.yaml";
  const std::string saved = std::string(temp)+"/saved.yaml";
  SG100GestureLibrary library;
  SG100HandPose pose;
  for (const std::string side : {"left", "right"}) {
    const std::string fixture = std::string(argv[1])+"/sg100_action_list_"+side+".yaml";
    const YAML::Node input = YAML::LoadFile(fixture);
    expect(library.load(fixture), side+" field fixture loads");
    expect(library.modeCount()==3, side+" has all three gestures");
    for (int gesture=0; gesture<3; ++gesture) {
      CurveGesture g;
      expect(library.getGesture(gesture,g),"gesture is selectable by original index");
      expect(g.name==input[gesture]["name"].as<std::string>(),"gesture name preserved");
      for (float trigger : {0.0f, 0.25f, 0.5f, 1.0f}) {
        expect(library.buildTarget(gesture,trigger,pose),"trigger produces target");
        expect(pose.positions.size()==11 && pose.modes.size()==11,"11-joint target shape");
        if (pose.positions.size()!=11 || pose.modes.size()!=11) continue;
        for (int joint=0; joint<11; ++joint) {
          float lo=input[gesture]["action_list"][0][joint+1].as<float>();
          float hi=input[gesture]["action_list"][1][joint+1].as<float>();
          expect(near(pose.positions[joint],lo+(hi-lo)*trigger),"radian values and interpolation preserved");
          expect(pose.modes[joint]==kuavo_msgs::SG100HandCommand::MODE_JOINT_POSITION,"position mode only");
        }
        expect(!pose.has_force_control && pose.kp.empty() && pose.kd.empty() && pose.output_limit.empty(),"no inferred impedance or gain override");
      }
    }
    expect(library.save(saved),"legacy import saves in canonical format");
    SG100GestureLibrary reloaded;
    expect(reloaded.load(saved) && reloaded.modeCount()==3,"canonical round-trip preserves count");
    for(int gesture=0;gesture<3;++gesture) {
      SG100HandPose before, after;
      expect(library.buildTarget(gesture,.37f,before) && reloaded.buildTarget(gesture,.37f,after),"round-trip target builds");
      for(int joint=0;joint<11 && before.positions.size()==11 && after.positions.size()==11;++joint)
        expect(near(before.positions[joint],after.positions[joint]),"round-trip values preserved");
    }
  }
  // Multiple keyframes use the same piecewise interpolation as canonical curves.
  YAML::Node entry=validAction(), rows;
  rows.push_back(YAML::Clone(entry["action_list"][0]));
  YAML::Node middle=YAML::Clone(entry["action_list"][0]); middle[0]=25.0f;
  for(int joint=1;joint<=11;++joint) middle[joint]=.8f;
  rows.push_back(middle);
  YAML::Node plateau=YAML::Clone(middle);plateau[0]=50.0f;rows.push_back(plateau);
  rows.push_back(YAML::Clone(entry["action_list"][1]));
  entry["action_list"]=rows;
  YAML::Node root;root.push_back(entry);writeYaml(file,root);
  expect(library.load(file),"four-keyframe position action loads");
  expect(library.buildTarget(0,.25f,pose) && near(pose.positions[0],.8f),"middle keyframe preserved");
  expect(library.buildTarget(0,.75f,pose) && near(pose.positions[0],.3f),"second segment interpolates");
  // Both formats can coexist as separate gestures, without changing their IDs.
  root=YAML::Node(YAML::NodeType::Sequence);
  root.push_back(YAML::LoadFile(saved)[1]);root.push_back(validAction());writeYaml(file,root);
  expect(library.load(file) && library.modeCount()==2,"mixed-format library loads");
  CurveGesture mixed;
  expect(library.getGesture(1,mixed) && mixed.name=="radian_pose","mixed-format ordering preserved");
  // Never silently drop malformed legacy rows and renumber later gestures.
  for (int invalid=0;invalid<18;++invalid) {
    YAML::Node bad=validAction();
    switch(invalid) {
      case 0: {
        YAML::Node short_row;
        for(int i=0;i<11;++i) short_row.push_back(bad["action_list"][0][i].as<float>());
        bad["action_list"][0]=short_row;break;
      }
      case 1: bad["action_list"][0].push_back(0.0f); break;
      case 2: bad["action_list"][0][1]="not-a-number"; break;
      case 3: bad["action_list"][0][1]=YAML::Load(".nan"); break;
      case 4: bad["action_list"][0][1]=YAML::Load(".inf"); break;
      case 5: bad["action_list"][0][0]=YAML::Load(".nan"); break;
      case 6: bad["action_list"][0][0]=-1.0f; break;
      case 7: bad["action_list"][1][0]=101.0f; break;
      case 8: bad["action_list"][0][0]=1.0f; break;
      case 9: bad["action_list"][1][0]=99.0f; break;
      case 10: {
        YAML::Node reversed;
        reversed.push_back(YAML::Clone(bad["action_list"][0]));
        YAML::Node a=YAML::Clone(bad["action_list"][0]);a[0]=60.0f;reversed.push_back(a);
        YAML::Node b=YAML::Clone(bad["action_list"][0]);b[0]=40.0f;reversed.push_back(b);
        reversed.push_back(YAML::Clone(bad["action_list"][1]));bad["action_list"]=reversed;break;
      }
      case 11: bad["action_list"]=YAML::Node(YAML::NodeType::Map); break;
      case 12: bad["name"]=""; break;
      case 13: bad["position"]=YAML::Node(YAML::NodeType::Sequence); break;
      case 14: {
        YAML::Node single; single.push_back(YAML::Clone(bad["action_list"][0]));
        bad["action_list"]=single;break;
      }
      case 15: bad["action_list"][0][1]=YAML::Node(YAML::NodeType::Null); break;
      case 16: bad["action_list"][0]="invalid-row"; break;
      case 17: {
        YAML::Node three;
        three.push_back(YAML::Clone(bad["action_list"][0]));
        YAML::Node midpoint=YAML::Clone(bad["action_list"][0]); midpoint[0]=50.0f;
        three.push_back(midpoint); three.push_back(YAML::Clone(bad["action_list"][1]));
        bad["action_list"]=three;break;
      }
    }
    root=YAML::Node(YAML::NodeType::Sequence);
    root.push_back(validAction()); root.push_back(bad);root.push_back(validAction());writeYaml(file,root);
    expect(!library.load(file),"invalid action_list rejected case "+std::to_string(invalid));
    expect(library.modeCount()==0 && !library.buildTarget(0,.5f,pose),"invalid library cannot output a partial target");
  }
  std::filesystem::remove_all(temp);
  std::printf("SG100 action_list: %d checks, %d failures\n",checks,failures);
  return failures ? 1 : 0;
}
