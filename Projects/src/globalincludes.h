#pragma once

#include <iostream>
#include <vector>
#include <utility>

#include "Eigen/Dense"
#include "Eigen/Core"
#include "Partio.h"

using Eigen::MatrixXd;
using Eigen::Matrix;

using T = float;
const int dim = 2;

using Vector = Matrix<T, dim, 1>;
using VectorPair = std::pair<Vector, Vector>;

const int NUM_AGENTS = 50;

constexpr float GROUND_Y_POS = 0.f;

constexpr float FRAMES_PER_SECOND = 48;
constexpr float SIMULATION_DURATION = 10;
constexpr float TIME_STEP = 1.f / FRAMES_PER_SECOND;
constexpr float TIME_STEP_SQ = TIME_STEP * TIME_STEP;

constexpr float VELOCITY_BLEND = 0.0385f;
constexpr float MAX_VELOCITY = 2.f;
constexpr float MAX_STABILITY_ITERATION = 10;
constexpr float MAX_ITERATION = 5;

constexpr float AGENT_RADIUS = 0.1f;
constexpr float AGENT_MASS = 1.0f;

constexpr float CONSTRAINT_CA_MAX_TAU = 20.0f; // Collision Avoidance Max Tau

constexpr float VISCOSITY_H = 7;
constexpr float VISCOSITY_C = 217;

constexpr float POLY_6_KERNEL = 1.566681471f;
