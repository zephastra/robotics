// Assisted grasp: a fixed physics constraint after geometric/finger validation.
// No teleporting and no Gazebo set_pose calls. Not a friction grasp model.
#include <atomic>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <sstream>
#include <string>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>

namespace mm003 {
using namespace gz::sim;
class GuardedGrasp : public System, public ISystemConfigure,
                     public ISystemPreUpdate, public ISystemPostUpdate {
  Entity base{kNullEntity}, tool{kNullEntity}, tray{kNullEntity}, cargo{kNullEntity};
  Entity fingerL{kNullEntity}, fingerR{kNullEntity}, constraint{kNullEntity};
  gz::transport::Node node;
  gz::transport::Node::Publisher pub;
  std::mutex mutex;
  std::string pending, holder{"none"}, rejection;
  double lastPub{-1}, lastTime{-1}, speed{0}, lastYaw{0};
  gz::math::Vector3d lastBase;
  static void poseJson(std::ostream &s, const gz::math::Pose3d &p) {
    s << "[" << p.Pos().X() << "," << p.Pos().Y() << "," << p.Pos().Z()
      << "," << p.Rot().Yaw() << "]";
  }
 public:
  void Configure(const Entity &e,const std::shared_ptr<const sdf::Element>&,
                 EntityComponentManager &ecm,EventManager&) override {
    Model model(e);
    base=model.LinkByName(ecm,"base_link"); tool=model.LinkByName(ecm,"tool");
    tray=model.LinkByName(ecm,"tray");
    fingerL=model.JointByName(ecm,"finger_left"); fingerR=model.JointByName(ecm,"finger_right");
    for(auto j:{fingerL,fingerR}) if(!ecm.Component<components::JointPosition>(j))
      ecm.CreateComponent(j,components::JointPosition());
    node.Subscribe("/mm/grasp_command",&GuardedGrasp::Command,this);
    pub=node.Advertise<gz::msgs::StringMsg>("/mm/physics");
  }
  void Command(const gz::msgs::StringMsg &m) {
    std::lock_guard<std::mutex> lock(mutex); pending=m.data();
  }
  void PreUpdate(const UpdateInfo &info,EntityComponentManager &ecm) override {
    if(info.paused) return;
    if(cargo==kNullEntity) {
      auto model=ecm.EntityByComponents(components::Model(),components::Name("material_box"));
      cargo=ecm.EntityByComponents(components::Link(),components::ParentEntity(model),components::Name("body"));
      if(cargo==kNullEntity) return;
    }
    auto bp=worldPose(base,ecm);
    double now=std::chrono::duration<double>(info.simTime).count();
    if(lastTime>=0 && now>lastTime) {
      double dyaw=bp.Rot().Yaw()-lastYaw;
      dyaw=std::atan2(std::sin(dyaw),std::cos(dyaw));
      // Rotating in place also counts as motion for all grasp operations.
      speed=std::max((bp.Pos()-lastBase).Length(), .4*std::abs(dyaw))/(now-lastTime);
    }
    lastBase=bp.Pos(); lastTime=now; lastYaw=bp.Rot().Yaw();
    std::string command;
    {std::lock_guard<std::mutex> lock(mutex); command.swap(pending);}
    if(command.empty()) return;
    rejection.clear();
    if(speed>.035) {rejection="base_moving";return;}
    if(command=="release" || command=="unsecure") {
      if(holder!=(command=="release"?"tool":"tray")) {rejection="holder_mismatch";return;}
      ecm.RequestRemoveEntity(constraint); constraint=kNullEntity; holder="none"; return;
    }
    if(command!="grip" && command!="secure") {rejection="unknown_command";return;}
    if(holder!="none") {rejection="already_held";return;}
    if(speed>.035) {rejection="base_moving";return;}
    auto cp=worldPose(cargo,ecm);
    Entity parent=command=="grip"?tool:tray;
    auto pp=worldPose(parent,ecm);
    auto target=pp.Pos();
    if(command=="secure") target=pp.CoordPositionAdd(gz::math::Vector3d(0,0,.07));
    if((cp.Pos()-target).Length()>.045) {rejection="outside_capture_volume";return;}
    if(command=="grip") {
      for(auto j:{fingerL,fingerR}) {
        auto p=ecm.Component<components::JointPosition>(j);
        if(!p || p->Data().empty() || p->Data()[0]>.058) {rejection="fingers_not_closed";return;}
      }
    }
    constraint=ecm.CreateEntity();
    ecm.CreateComponent(constraint,components::DetachableJoint({parent,cargo,"fixed"}));
    holder=command=="grip"?"tool":"tray";
  }
  void PostUpdate(const UpdateInfo &info,const EntityComponentManager &ecm) override {
    double now=std::chrono::duration<double>(info.simTime).count();
    if(cargo==kNullEntity || now-lastPub<.1) return;
    lastPub=now;
    std::ostringstream out;
    out << "{\"sim_time\":"<<now<<",\"base\":"; poseJson(out,worldPose(base,ecm));
    out << ",\"tool\":"; poseJson(out,worldPose(tool,ecm));
    out << ",\"cargo\":"; poseJson(out,worldPose(cargo,ecm));
    out << ",\"tray\":"; poseJson(out,worldPose(tray,ecm));
    out << ",\"holder\":\""<<holder<<"\",\"rejection\":\""<<rejection<<"\",\"base_speed\":"<<speed<<"}";
    gz::msgs::StringMsg message; message.set_data(out.str()); pub.Publish(message);
  }
};
}
GZ_ADD_PLUGIN(mm003::GuardedGrasp,gz::sim::System,mm003::GuardedGrasp::ISystemConfigure,mm003::GuardedGrasp::ISystemPreUpdate,mm003::GuardedGrasp::ISystemPostUpdate)
