/*
 * XTDrone actor motion system for Gazebo Harmonic (manual chapter 38).
 *
 * Port of XTDrone's sitl_config/gazebo_plugin/gazebo_ros_actor_plugin (ros_actor_cmd_pose_plugin): an actor
 * walks to a commanded (x, y) at a commanded speed, with its walking animation, and stays there.
 * Commands: gz topic /<actor name>/cmd_motion, gz.msgs.Vector3d (x, y, z = speed in m/s); with
 * xtdrone.launch.py the ROS topic of the same name (geometry_msgs/Vector3) is bridged to it.
 * The actor's world pose is published on /<actor name>/pose (gz.msgs.Pose, 10 Hz): gz-sim's pose
 * topics do not include actors.
 * Animation handling as in gz-sim's FollowActor system.
 *
 *   <actor name="actor_0"> ...
 *     <plugin filename="xtdrone_actor_motion" name="xtdrone::ActorMotion">
 *       <animation_x_vel>2.0</animation_x_vel>   <!-- optional -->
 *     </plugin>
 *   </actor>
 */
#include <chrono>
#include <mutex>
#include <optional>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>

namespace xtdrone
{
class ActorMotion : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &) override
  {
    namespace components = gz::sim::components;
    this->actor = _entity;
    auto actorComp = _ecm.Component<components::Actor>(_entity);
    if (!actorComp)
    {
      gzerr << "xtdrone::ActorMotion: entity [" << _entity << "] is not an actor\n";
      return;
    }
    if (_sdf->HasElement("animation_x_vel"))
      this->animationXVel = _sdf->Get<double>("animation_x_vel");
    std::string animation = _sdf->HasElement("animation") ? _sdf->Get<std::string>("animation") : "";
    if (animation.empty() && actorComp->Data().AnimationCount() > 0)
      animation = actorComp->Data().AnimationByIndex(0)->Name();
    if (animation.empty())
    {
      gzerr << "xtdrone::ActorMotion: actor has no animation\n";
      return;
    }
    if (auto c = _ecm.Component<components::AnimationName>(_entity))
      *c = components::AnimationName(animation);
    else
      _ecm.CreateComponent(_entity, components::AnimationName(animation));
    _ecm.SetChanged(_entity, components::AnimationName::typeId, gz::sim::ComponentState::OneTimeChange);
    if (!_ecm.Component<components::AnimationTime>(_entity))
      _ecm.CreateComponent(_entity, components::AnimationTime());

    // The actor's Pose keeps only the height; x, y and yaw move into the trajectory pose (the actor's
    // world pose is Pose * TrajectoryPose, so a yaw left in Pose would rotate the commanded points).
    gz::math::Pose3d initial;
    if (auto poseComp = _ecm.Component<components::Pose>(_entity))
    {
      initial = poseComp->Data();
      *poseComp = components::Pose(gz::math::Pose3d(0, 0, initial.Pos().Z(), 0, 0, 0));
    }
    else
    {
      _ecm.CreateComponent(_entity, components::Pose(gz::math::Pose3d::Zero));
    }
    if (!_ecm.Component<components::TrajectoryPose>(_entity))
    {
      initial.Pos().Z(0);
      _ecm.CreateComponent(_entity, components::TrajectoryPose(initial));
    }

    auto name = _ecm.Component<components::Name>(_entity)->Data();
    this->topic = "/" + name + "/cmd_motion";
    this->node.Subscribe(this->topic, &ActorMotion::OnCommand, this);
    this->posePub = this->node.Advertise<gz::msgs::Pose>("/" + name + "/pose");
    this->basePose = _ecm.Component<components::Pose>(_entity)->Data();
    this->name = name;
    gzmsg << "xtdrone::ActorMotion: actor [" << name << "] listens on " << this->topic << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override
  {
    namespace components = gz::sim::components;
    const double dt = std::chrono::duration<double>(_info.simTime - this->lastUpdate).count();
    this->lastUpdate = _info.simTime;
    if (_info.paused || this->actor == gz::sim::kNullEntity)
      return;
    this->PublishPose(_info, _ecm);
    gz::math::Vector3d goal;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      if (!this->command)
        return;
      goal = *this->command;
    }
    auto trajPoseComp = _ecm.Component<components::TrajectoryPose>(this->actor);
    if (!trajPoseComp)
      return;
    auto pose = trajPoseComp->Data();
    gz::math::Vector3d dir(goal.X() - pose.Pos().X(), goal.Y() - pose.Pos().Y(), 0);
    const double distance = dir.Length();
    const double speed = goal.Z();
    if (distance < 0.05 || speed <= 0.0)
      return;  // arrived: keep pose and orientation, animation stops
    dir.Normalize();
    const double step = std::min(distance, speed * dt);
    const auto start = pose.Pos();
    pose.Pos() += dir * step;
    pose.Pos().Z(0);
    pose.Rot() = gz::math::Quaterniond(0, 0, std::atan2(dir.Y(), dir.X()));
    *trajPoseComp = components::TrajectoryPose(pose);
    _ecm.SetChanged(this->actor, components::TrajectoryPose::typeId, gz::sim::ComponentState::OneTimeChange);

    auto animTimeComp = _ecm.Component<components::AnimationTime>(this->actor);
    auto animTime = animTimeComp->Data() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                               std::chrono::duration<double>((pose.Pos() - start).Length() *
                                                                             this->animationXVel));
    *animTimeComp = components::AnimationTime(animTime);
    _ecm.SetChanged(this->actor, components::AnimationTime::typeId, gz::sim::ComponentState::OneTimeChange);
  }

private:
  void PublishPose(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
  {
    if (_info.simTime - this->lastPosePub < std::chrono::milliseconds(100))
      return;
    this->lastPosePub = _info.simTime;
    auto traj = _ecm.Component<gz::sim::components::TrajectoryPose>(this->actor);
    if (!traj)
      return;
    // world pose = actor base pose (height, orientation offset) * trajectory pose
    auto world = this->basePose * traj->Data();
    gz::msgs::Pose msg = gz::msgs::Convert(world);
    msg.set_name(this->name);
    this->posePub.Publish(msg);
  }

  void OnCommand(const gz::msgs::Vector3d &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->command = gz::math::Vector3d(_msg.x(), _msg.y(), _msg.z());
  }

  gz::sim::Entity actor{gz::sim::kNullEntity};
  double animationXVel{2.0};
  std::chrono::steady_clock::duration lastUpdate{0};
  gz::transport::Node node;
  gz::transport::Node::Publisher posePub;
  gz::math::Pose3d basePose;
  std::chrono::steady_clock::duration lastPosePub{0};
  std::string name;
  std::string topic;
  std::mutex mutex;
  std::optional<gz::math::Vector3d> command;
};
}  // namespace xtdrone

GZ_ADD_PLUGIN(xtdrone::ActorMotion, gz::sim::System, xtdrone::ActorMotion::ISystemConfigure,
              xtdrone::ActorMotion::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(xtdrone::ActorMotion, "xtdrone::ActorMotion")
