// Copyright 2022 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//#include "mjpc/tasks/cartpole/cartpole.h"
#include "mjpc/tasks/cartpole_dynamic_goal/cartpole.h"

#include <cmath>
#include <string>
#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include "mjpc/utilities.h"

namespace mjpc {
namespace cartpole_dynamic_goal {

// 🔹 Define the destructor in the `.cc` file
CartpoleDynamicGoal::~CartpoleDynamicGoal() = default;

std::string CartpoleDynamicGoal::XmlPath() const {
  return GetModelPath("cartpole_dynamic_goal/task.xml");
}

std::string CartpoleDynamicGoal::Name() const { return "Cartpole Dynamic Goal"; }

void CartpoleDynamicGoal::ResidualFn::Residual(const mjModel* model, const mjData* data, double* residual) const {
  // Access the parent task (correct way)
  const auto* cartpole_task = static_cast<const CartpoleDynamicGoal*>(task_);

  double current_time = data->time;
  
  // Define goal positions
  double min_pos = -1.5;
  double max_pos = 1.5;
  double left_goal = min_pos + (max_pos - min_pos) / 4;
  double right_goal = max_pos - (max_pos - min_pos) / 4;

  // Switch goal every 5 seconds
  if (current_time - cartpole_task->elapsed_time_ > 5.0) {
    cartpole_task->elapsed_time_ = current_time;
    cartpole_task->goal_state_ = 1 - cartpole_task->goal_state_;
  }

  // Set dynamic goal
  double dynamic_goal = (cartpole_task->goal_state_ == 0) ? left_goal : right_goal;

  // Residuals (Cost Terms)
  residual[0] = std::cos(data->qpos[1]) - 1;  // Pole balance
  residual[1] = data->qpos[0] - dynamic_goal;  // Cart position to goal
  residual[2] = data->qvel[1];  // Pole angular velocity
  residual[3] = data->ctrl[0];  // Control effort
}


}  // namespace cartpole_dynamic_goal
}  // namespace mjpc

