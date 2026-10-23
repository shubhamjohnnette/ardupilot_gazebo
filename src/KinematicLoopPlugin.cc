/*
 * Copyright (C) 2024 ArduPilot Gazebo
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "KinematicLoopPlugin.hh"

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

class KinematicLoopPlugin::Impl
{
  public: Model model{kNullEntity};
  public: double speed{25.0};
  public: double height{0.0};
  public: double currentDistance{0.0};
  public: double totalDistance{0.0};
  public: std::vector<gz::math::Vector2d> waypoints;
  public: std::vector<double> cumDistances;
};

KinematicLoopPlugin::KinematicLoopPlugin()
  : impl(std::make_unique<Impl>())
{
}

KinematicLoopPlugin::~KinematicLoopPlugin() = default;

void KinematicLoopPlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &)
{
  this->impl->model = Model(_entity);

  if (_sdf->HasElement("speed"))
  {
    this->impl->speed = _sdf->Get<double>("speed");
  }

  if (_sdf->HasElement("height"))
  {
    this->impl->height = _sdf->Get<double>("height");
  }

  // Parse waypoints
  if (_sdf->HasElement("waypoints"))
  {
    auto waypointsElem = _sdf->FindElement("waypoints");
    if (waypointsElem->HasElement("waypoint"))
    {
      auto wpElem = waypointsElem->FindElement("waypoint");
      while (wpElem)
      {
        std::stringstream ss(wpElem->Get<std::string>());
        double x = 0.0, y = 0.0;
        if (ss >> x >> y)
        {
          this->impl->waypoints.push_back(gz::math::Vector2d(x, y));
        }
        wpElem = wpElem->GetNextElement("waypoint");
      }
    }
  }

  // Compute cumulative distances along the closed loop
  size_t n = this->impl->waypoints.size();
  if (n >= 2)
  {
    this->impl->cumDistances.clear();
    double acc = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
      this->impl->cumDistances.push_back(acc);
      size_t next = (i + 1) % n;
      double segLen = (this->impl->waypoints[next] - this->impl->waypoints[i]).Length();
      acc += segLen;
    }
    this->impl->cumDistances.push_back(acc);
    this->impl->totalDistance = acc;

    // Optional start offset
    if (_sdf->HasElement("start_dist"))
    {
      this->impl->currentDistance = _sdf->Get<double>("start_dist");
    }
    else if (_sdf->HasElement("start_index"))
    {
      size_t idx = static_cast<size_t>(_sdf->Get<int>("start_index")) % n;
      this->impl->currentDistance = this->impl->cumDistances[idx];
    }
  }
}

void KinematicLoopPlugin::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  size_t n = this->impl->waypoints.size();
  if (n < 2 || this->impl->totalDistance <= 0.0)
    return;

  double dt = std::chrono::duration<double>(_info.dt).count();
  if (dt > 0.0)
  {
    this->impl->currentDistance += this->impl->speed * dt;
    while (this->impl->currentDistance >= this->impl->totalDistance)
    {
      this->impl->currentDistance -= this->impl->totalDistance;
    }
    while (this->impl->currentDistance < 0.0)
    {
      this->impl->currentDistance += this->impl->totalDistance;
    }
  }

  // Find active segment
  size_t seg = 0;
  for (size_t i = 0; i < n; ++i)
  {
    if (this->impl->currentDistance >= this->impl->cumDistances[i] &&
        this->impl->currentDistance < this->impl->cumDistances[i + 1])
    {
      seg = i;
      break;
    }
  }

  size_t nextSeg = (seg + 1) % n;
  double segStart = this->impl->cumDistances[seg];
  double segLen = this->impl->cumDistances[seg + 1] - segStart;
  double u = (segLen > 1e-6) ? (this->impl->currentDistance - segStart) / segLen : 0.0;
  u = std::max(0.0, std::min(1.0, u));

  gz::math::Vector2d pos = (1.0 - u) * this->impl->waypoints[seg] +
                           u * this->impl->waypoints[nextSeg];
  gz::math::Vector2d delta = this->impl->waypoints[nextSeg] - this->impl->waypoints[seg];
  double yaw = std::atan2(delta.Y(), delta.X());

  gz::math::Pose3d targetPose(pos.X(), pos.Y(), this->impl->height, 0.0, 0.0, yaw);
  this->impl->model.SetWorldPoseCmd(_ecm, targetPose);
}

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::KinematicLoopPlugin,
    gz::sim::System,
    gz::sim::systems::KinematicLoopPlugin::ISystemConfigure,
    gz::sim::systems::KinematicLoopPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::KinematicLoopPlugin,
    "KinematicLoopPlugin")
