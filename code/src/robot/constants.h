#pragma once
#include "utils/constants.h"

constexpr double RADIUS_A = 25;
constexpr double RADIUS_B = 18;
constexpr double RADIUS_ROBOT = 15;

constexpr size_t COUNT_CELL_X = 200;
constexpr size_t COUNT_CELL_Y = 90;

constexpr size_t MAP_WIDTH = SIZE_CELL * COUNT_CELL_X;
constexpr size_t MAP_HEIGHT = SIZE_CELL * COUNT_CELL_Y;

constexpr size_t MU = 5;
constexpr double EPSILON = 50.0;
constexpr double DT_SMOOTH_PATH = 1.0;

constexpr double V_SPEED = 3;

constexpr double GAMMA_REPULSIVE = 0.1;

constexpr size_t COUNT_STEPS_GRADIENT = 20;
constexpr double STEP_GRADIENT = 0.03;
constexpr double STEP_DESCENT = 1e-8;

constexpr double TIME_FOR_PREDICT = 4;

constexpr double LARGE_LAMBDA = 10000.0;

constexpr double WEIGHT_COST_TRACKING = 0.1;
constexpr double WEIGHT_COST_REPULSIVE = 50000.0;