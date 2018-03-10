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

using Vector = Matrix<T, dim, 1>;

const int NUM_AGENTS = 50;

constexpr float FRAMES_PER_SECOND = 30;
constexpr float SIMULATION_DURATION = 10;
constexpr float TIME_STEP = 1.f / FRAMES_PER_SECOND;
constexpr float TIME_STEP_SQ = TIME_STEP * TIME_STEP;

constexpr float VELOCITY_BLEND = 0.0385f;

constexpr float AGENT_RADIUS = 0.1f;
constexpr float AGENT_MASS = 1.0f;

constexpr float CONSTRAINT_CA_MAX_TAU = 20.0f; // Collision Avoidance Max Tau
