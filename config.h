#ifndef CONFIG_H
#define CONFIG_H

const double default_shape_density = 1000;  // kg/m^3
const double default_shape_height = 0.1;    // m
const double default_shape_width = 0.03;    // m

const double default_start_height = 0.4;  // m

const double minimum_start_v = 2.5;  // m/s
const double maximum_start_v = 4.0;  // m/s
const double default_start_v = 3.5;  // m/s

const double minimum_launch_angle = 30.0 * M_PI / 180.0;  // rad
const double maximum_launch_angle = 70.0 * M_PI / 180.0;  // rad
const double default_launch_angle = 45.0 * M_PI / 180.0;  // rad

const double maximum_start_w = 6 * M_PI;  // rad/s
const double default_start_w = 3 * M_PI;  // rad/s

const double default_damping_coefficient = 0.001;

const double default_ground_width = 2;
const double default_wall_thickness = 0.1;
const double default_wall_height = 1;
const double default_spawn_range = 0.9 * default_ground_width / 2;

const double default_restitution = 0.6;

#endif // CONFIG_H
