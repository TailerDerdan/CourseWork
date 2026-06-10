#pragma once
#include "utils/constants.h"

constexpr size_t COUNT_CELL_X = 200;
constexpr size_t COUNT_CELL_Y = 100;

constexpr size_t MAP_WIDTH = SIZE_CELL * COUNT_CELL_X;
constexpr size_t MAP_HEIGHT = SIZE_CELL * COUNT_CELL_Y;

constexpr double RADIUS_A = 35;
constexpr double RADIUS_B = 25;
constexpr double RADIUS_ROBOT = 8;

constexpr size_t MU = 2;
constexpr double EPSILON = 50;
constexpr double DT_SMOOTH_PATH = 2.0;

constexpr double V_SPEED = 3.0;

constexpr double GAMMA_REPULSIVE = 0.1;

constexpr size_t COUNT_STEPS_GRADIENT = 20;
constexpr double STEP_GRADIENT = 0.005;
constexpr double STEP_DESCENT = 3e-7;

constexpr double TIME_FOR_PREDICT = 2.0;

constexpr double LARGE_LAMBDA = 2000.0;
constexpr double WEIGHT_COST_TRACKING = 16.0;
constexpr double WEIGHT_COST_OBSTACLES = 300.0;
constexpr double WEIGHT_COST_REPULSIVE = 500.0;