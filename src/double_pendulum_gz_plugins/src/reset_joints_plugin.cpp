#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <sdf/Element.hh>

#include <gz/plugin2/gz/plugin/Register.hh>

#include <gz/sim8/gz/sim/System.hh>
#include <gz/sim8/gz/sim/Model.hh>
#include <gz/sim8/gz/sim/Types.hh>
#include <gz/sim8/gz/sim/EntityComponentManager.hh>
#include <gz/sim8/gz/sim/components/JointPositionReset.hh>
#include <gz/sim8/gz/sim/components/JointVelocityReset.hh>

#include <gz/transport13/gz/transport/Node.hh>
#include <gz/msgs10/gz/msgs/boolean.pb.h>

#include <gz/common5/gz/common/Console.hh>

namespace double_pendulum_plugins
{

class ResetJointsPlugin
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  ~ResetJointsPlugin() override = default;

  void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> & /*_sdf*/,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager & /*_eventMgr*/) override
  {
    gzmsg << "ResetJointsPlugin loaded\n";
    this->model_ = gz::sim::Model(_entity);

    this->jointEntities_["joint1"] = this->model_.JointByName(_ecm, "joint1");
    this->jointEntities_["joint2"] = this->model_.JointByName(_ecm, "joint2");
    this->jointEntities_["joint3"] = this->model_.JointByName(_ecm, "joint3");
    this->jointEntities_["joint4"] = this->model_.JointByName(_ecm, "joint4");
    this->jointEntities_["joint5"] = this->model_.JointByName(_ecm, "joint5");
    this->jointEntities_["joint6"] = this->model_.JointByName(_ecm, "joint6");
    this->jointEntities_["pendulum_joint_1"] = this->model_.JointByName(_ecm, "pendulum_joint_1");
    this->jointEntities_["pendulum_joint_2"] = this->model_.JointByName(_ecm, "pendulum_joint_2");

    this->node_.Subscribe(
      "/double_pendulum/reset_joints",
      &ResetJointsPlugin::OnResetMsg,
      this);
  }

  void PreUpdate(
      const gz::sim::UpdateInfo & /*_info*/,
      gz::sim::EntityComponentManager &_ecm) override
  {
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (!this->pendingReset_)
      return;

    this->pendingReset_ = false;

    gzmsg << "Applying joint reset\n";

    this->ResetJoint(_ecm, "joint1", 0.0, 0.0);
    this->ResetJoint(_ecm, "joint2", 0.79, 0.0);
    this->ResetJoint(_ecm, "joint3", 0.79, 0.0);
    this->ResetJoint(_ecm, "joint4", 0.0, 0.0);
    this->ResetJoint(_ecm, "joint5", 0.0, 0.0);
    this->ResetJoint(_ecm, "joint6", 0.0, 0.0);

    this->ResetJoint(_ecm, "pendulum_joint_1", 0.0, 0.0);
    this->ResetJoint(_ecm, "pendulum_joint_2", 0.0, 0.0);
  }

private:
  void OnResetMsg(const gz::msgs::Boolean &_msg)
  {
    gzmsg << "Received reset request\n";
    if (_msg.data())
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->pendingReset_ = true;
    }
  }

  void ResetJoint(
      gz::sim::EntityComponentManager &_ecm,
      const std::string &jointName,
      double position,
      double velocity)
  {
    auto it = this->jointEntities_.find(jointName);
    if (it == this->jointEntities_.end() || it->second == gz::sim::kNullEntity)
      return;

    const gz::sim::Entity jointEntity = it->second;

    auto *posComp =
      _ecm.Component<gz::sim::components::JointPositionReset>(jointEntity);
    if (posComp)
      posComp->Data() = {position};
    else
      _ecm.CreateComponent(
        jointEntity,
        gz::sim::components::JointPositionReset({position}));

    auto *velComp =
      _ecm.Component<gz::sim::components::JointVelocityReset>(jointEntity);
    if (velComp)
      velComp->Data() = {velocity};
    else
      _ecm.CreateComponent(
        jointEntity,
        gz::sim::components::JointVelocityReset({velocity}));
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  std::unordered_map<std::string, gz::sim::Entity> jointEntities_;
  gz::transport::Node node_;
  std::mutex mutex_;
  bool pendingReset_{false};
};

}  // namespace double_pendulum_plugins

GZ_ADD_PLUGIN(
  double_pendulum_plugins::ResetJointsPlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate
)