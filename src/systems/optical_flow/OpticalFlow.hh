/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef OPTICALFLOW_HH_
#define OPTICALFLOW_HH_

#include <memory>
#include <string>

#include <gz/sim/config.hh>
#include <gz/sim/System.hh>
#include <gz/sensors/Sensor.hh>
#include <sdf/Sensor.hh>

using namespace gz;
using namespace sim;
using namespace systems;

namespace custom
{
  // Forward declarations.
  class OpticalFlowPrivate;

  /// \class Sensors Sensors.hh gz/sim/systems/Sensors.hh
  /// \brief A system that manages sensors.
  ///
  /// ## System Parameters
  ///
  /// - `<render_engine>` Name of the render engine, such as 'ogre' or 'ogre2'.
  /// - `<background_color>` Color used for the scene's background. This
  /// will override the background color specified in a world's SDF <scene>
  /// element. This background color is used by sensors, not the GUI.
  /// - `<ambient_light>` Color used for the scene's ambient light. This
  /// will override the ambient value specified in a world's SDF <scene>
  /// element. This ambient light is used by sensors, not the GUI.
  /// - `<disable_on_drained_battery>` Disable sensors if the model's
  /// battery plugin charge reaches zero. Sensors that are in nested
  /// models are also affected.
  ///
  /// \TODO(louise) Have one system for all sensors, or one per
  /// sensor / sensor type?
  class OpticalFlow final:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
  {
    /// \brief Constructor
    public: OpticalFlow();

    /// \brief Destructor
    public: ~OpticalFlow() final = default;

    /// Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    /// Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                const EntityComponentManager &_ecm) final;

    /// \brief Private data pointer
    private: std::unique_ptr<OpticalFlowPrivate> dataPtr;
  };
}
#endif