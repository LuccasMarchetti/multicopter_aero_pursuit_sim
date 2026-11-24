#pragma once

#include "aero_pursuit/scenario/spec.hpp"

#include <string>

namespace aero_pursuit::scenario
{

/// Reads a YAML scenario (text), resolves the trajectory references and validates. Angles in the file are in degrees.
/// Throws ScenarioError with the path of the invalid field.
Scenario parseScenario(const std::string & yaml_text);
Scenario loadScenarioFile(const std::string & path);

/// One trajectory (the YAML/JSON map of a `trajectories:` entry), validated the same way.
TrajectorySpec parseTrajectoryText(const std::string & yaml_text);

/// Name valid as a Gazebo model, TF frame and topic namespace.
bool isValidEntityName(const std::string & name);

}  // namespace aero_pursuit::scenario
