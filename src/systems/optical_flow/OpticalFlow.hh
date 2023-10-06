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
#ifndef OPTICALFLOW_HH_
#define OPTICALFLOW_HH_

#include <gz/sim/System.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/transport/Node.hh>

using namespace gz;
using namespace sim;
using namespace systems;
namespace custom
{
  class OpticalFlowSystemPrivate;

  /// \brief Example showing how to tie a custom sensor, in this case an
  /// odometer, into simulation
  class OpticalFlowSystem : public gz::sim::System,
                            public gz::sim::ISystemPreUpdate,
                            public gz::sim::ISystemPostUpdate
  {
    /// \brief Constructor
  public:
    explicit OpticalFlowSystem();

    /// \brief Destructor
  public:
    ~OpticalFlowSystem() override;
    // Documentation inherited.
    // During PreUpdate, check for new sensors that were inserted
    // into simulation and create more components as needed.
  public:
    void PreUpdate(const UpdateInfo &_info,
                   EntityComponentManager &_ecm) final;

    // Documentation inherited.
    // During PostUpdate, update the known sensors and publish their data.
    // Also remove sensors that have been deleted.
  public:
    void PostUpdate(const UpdateInfo &_info,
                    const EntityComponentManager &_ecm) final;

  private:
    std::unique_ptr<OpticalFlowSystemPrivate> dataPtr;
  };
}
#endif
