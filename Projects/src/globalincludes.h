#pragma once

#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "Eigen/Core"
#include "Partio.h"

using Eigen::MatrixXd;
using Eigen::Matrix;

using T = float;
const int dim = 2;

using Vector2 = Matrix<T, dim, 1>;

const int NUM_AGENTS = 50;

constexpr float FRAMES_PER_SECOND = 30;
constexpr float SIMULATION_DURATION = 10;
constexpr float TIME_STEP = 1.f / FRAMES_PER_SECOND;

constexpr float VELOCITY_BLEND = 0.0385f;

constexpr float AGENT_RADIUS = 0.1f;
