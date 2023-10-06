/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "OpticalFlow.hh"

#include <gz/msgs/stringmsg.pb.h>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <gz/plugin/Register.hh>

#include <sdf/Element.hh>

#include <gz/common/Profiler.hh>

#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/OpticalFlowSensor.hh>

#include "gz/sim/World.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/OpticalFlow.hh"
#include "gz/sim/components/Gravity.hh"
#include "gz/sim/components/LinearAcceleration.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Util.hh"
#include <gz/transport.hh>
#include <gz/msgs.hh>

using namespace gz;
using namespace sim;
using namespace systems;
using namespace custom;

class custom::OpticalFlowSystemPrivate
{
  /// \brief A map of logicalCamera entities
public:
  std::unordered_map<Entity,
                     std::unique_ptr<custom::OpticalFlowSensor>>
      entitySensorMap;

  /// \brief gz-sensors sensor factory for creating sensors
public:
  sensors::SensorFactory sensorFactory;

  /// \brief Keep list of sensors that were created during the previous
  /// `PostUpdate`, so that components can be created during the next
  /// `PreUpdate`.
public:
  std::unordered_set<Entity> newSensors;

  /// True if the rendering component is initialized
public:
  bool initialized = false;

public:
  void AddOpticalFlowSensor(
      const EntityComponentManager &_ecm,
      const Entity _entity,
      const components::OpticalFlow *_logicalCamera,
      const components::ParentEntity *_parent);

  /// \brief Create logicalCamera sensor
  /// \param[in] _ecm Immutable reference to ECM.
public:
  void CreateSensors(const EntityComponentManager &_ecm);

  /// \brief Update logicalCamera sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
public:
  void UpdateOpticalFlowSensors(const EntityComponentManager &_ecm);

  /// \brief Remove logicalCamera sensors if their entities have been removed
  /// from simulation.
  /// \param[in] _ecm Immutable reference to ECM.
public:
  void RemoveOpticalFlowSensorEntities(const EntityComponentManager &_ecm);
};

OpticalFlowSystem::OpticalFlowSystem() : System(),
                                         dataPtr(std::make_unique<OpticalFlowSystemPrivate>())
{
}

//////////////////////////////////////////////////
OpticalFlowSystem::~OpticalFlowSystem() = default;

//////////////////////////////////////////////////
void custom::OpticalFlowSystem::PreUpdate(const UpdateInfo & /*_info*/,
                                    EntityComponentManager &_ecm)
{
  GZ_PROFILE("OpticalFlowSystem::PreUpdate");

  // Create components
  for (auto entity : this->dataPtr->newSensors)
  {
    auto it = this->dataPtr->entitySensorMap.find(entity);
    if (it == this->dataPtr->entitySensorMap.end())
    {
      gzerr << "Entity [" << entity
            << "] isn't in sensor map, this shouldn't happen." << std::endl;
      continue;
    }
    // Set topic
    _ecm.CreateComponent(entity, components::SensorTopic(it->second->Topic()));
  }
  this->dataPtr->newSensors.clear();
}

//////////////////////////////////////////////////
void custom::OpticalFlowSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
                                           const EntityComponentManager &_ecm)
{
  // gzerr << "optical flow system :: post update" << std::endl;
  GZ_PROFILE("OpticalFlowSystem::PostUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  this->dataPtr->CreateSensors(_ecm);

  // Only update and publish if not paused.
  if (!_info.paused)
  {
    // check to see if update is necessary
    // we only update if there is at least one sensor that needs data
    // and that sensor has subscribers.
    // note: gz-sensors does its own throttling. Here the check is mainly
    // to avoid doing work in the LogicalCameraPrivate::UpdateLogicalCameras
    // function
    bool needsUpdate = false;
    for (auto &it : this->dataPtr->entitySensorMap)
    {
      if (it.second->NextDataUpdateTime() <= _info.simTime &&
          it.second->HasConnections())
      {
        needsUpdate = true;
        break;
      }
    }
    if (!needsUpdate)
      return;

    this->dataPtr->UpdateOpticalFlowSensors(_ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Update sensor
      it.second->Update(_info.simTime);
    }
  }

  this->dataPtr->RemoveOpticalFlowSensorEntities(_ecm);
}

void OpticalFlowSystemPrivate::AddOpticalFlowSensor(
    const EntityComponentManager &_ecm,
    const Entity _entity,
    const components::OpticalFlow *_logicalCamera,
    const components::ParentEntity *_parent)
{
  // gzerr << "optical flow system private :: add optical flow sensor" << std::endl;
  // create sensor
  std::string sensorScopedName =
      removeParentScope(scopedName(_entity, _ecm, "::", false), "::");
  sdf::Sensor data = _logicalCamera->Data();
  data.SetName(sensorScopedName);
  // check topic
  if (data.Topic().empty())
  {
    std::string topic = scopedName(_entity, _ecm) + "/optical_flow";
    data.SetTopic(topic);
  }
  std::unique_ptr<custom::OpticalFlowSensor> sensor =
      this->sensorFactory.CreateSensor<
          custom::OpticalFlowSensor>(data);
  if (nullptr == sensor)
  {
    gzerr << "Failed to create sensor [" << sensorScopedName << "]"
          << std::endl;
    return;
  }

  // set sensor parent
  std::string parentName = _ecm.Component<components::Name>(
                                   _parent->Data())
                               ->Data();
  sensor->SetParent(parentName);

  // set sensor world pose
  math::Pose3d sensorWorldPose = worldPose(_entity, _ecm);
  sensor->SetPose(sensorWorldPose);

  this->entitySensorMap.insert(
      std::make_pair(_entity, std::move(sensor)));
  this->newSensors.insert(_entity);
}

//////////////////////////////////////////////////
void OpticalFlowSystemPrivate::CreateSensors(const EntityComponentManager &_ecm)
{
  // gzerr << "optical flow system private :: create sensors" << std::endl;
  GZ_PROFILE("OpticalFlowPrivate::CreateOpticalFlowEntities");
  if (!this->initialized)
  {
    // Create logicalCameras
    _ecm.Each<components::OpticalFlow, components::ParentEntity>(
        [&](const Entity &_entity,
            const components::OpticalFlow *_logicalCamera,
            const components::ParentEntity *_parent) -> bool
        {
          this->AddOpticalFlowSensor(_ecm, _entity, _logicalCamera, _parent);
          return true;
        });
    this->initialized = true;
  }
  else
  {
    // Create logicalCameras
    _ecm.EachNew<components::OpticalFlow, components::ParentEntity>(
        [&](const Entity &_entity,
            const components::OpticalFlow *_logicalCamera,
            const components::ParentEntity *_parent) -> bool
        {
          this->AddOpticalFlowSensor(_ecm, _entity, _logicalCamera, _parent);
          return true;
        });
  }
}

//////////////////////////////////////////////////
void OpticalFlowSystemPrivate::UpdateOpticalFlowSensors(
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("OpticalFlowSystemPrivate::UpdateOpticalFlowSystems");
  std::map<std::string, math::Pose3d> modelPoses;

  // _ecm.Each<components::Model, components::Name, components::Pose>(
  //     [&](const Entity &,
  //         const components::Model *,
  //         const components::Name *_name,
  //         const components::Pose *_pose) -> bool
  //     {
  //       /// todo(anyone) We currently assume there are only top level models
  //       /// Update to retrieve world pose when nested models are supported.
  //       modelPoses[_name->Data()] = _pose->Data();
  //       return true;
  //     });

  // _ecm.Each<components::OpticalFlow, components::WorldPose>(
  //     [&](const Entity &_entity,
  //         const components::OpticalFlow * /*_logicalCamera*/,
  //         const components::WorldPose *_worldPose) -> bool
  //     {
  //       auto it = this->entitySensorMap.find(_entity);
  //       if (it != this->entitySensorMap.end())
  //       {
  //         const math::Pose3d &worldPose = _worldPose->Data();
  //         it->second->SetPose(worldPose);
  //         // Make a copy of modelPoses s.t. SetModelPoses can take ownership
  //         auto modelPoses_ = modelPoses;
  //         it->second->SetModelPoses(std::move(modelPoses_));
  //       }
  //       else
  //       {
  //         gzerr << "Failed to update logicalCamera: " << _entity << ". "
  //               << "Entity not found." << std::endl;
  //       }

  //       return true;
  //     });
}
//////////////////////////////////////////////////
void OpticalFlowSystemPrivate::RemoveOpticalFlowSensorEntities(
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("OpticalFlowSystemPrivate::RemoveOpticalFlowEntities");
  _ecm.EachRemoved<components::OpticalFlow>(
      [&](const Entity &_entity,
          const components::OpticalFlow *) -> bool
      {
        auto sensorIt = this->entitySensorMap.find(_entity);
        if (sensorIt == this->entitySensorMap.end())
        {
          gzerr << "Internal error, missing logicalCamera sensor for entity ["
                << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorIt);

        return true;
      });
}

GZ_ADD_PLUGIN(OpticalFlowSystem, gz::sim::System,
              OpticalFlowSystem::ISystemPreUpdate,
              OpticalFlowSystem::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(OpticalFlowSystem, "custom::OpticalFlowSystem")
