/* Copyright (c) 2017-2018, Waterloo Autonomous Vehicles Laboratory (WAVELab),
 * Waterloo Intelligent Systems Engineering Lab (WISELab),
 * University of Waterloo.
 *
 * Refer to the accompanying LICENSE file for license information.
 *
 * ############################################################################
 ******************************************************************************
 |                                                                            |
 |                         /\/\__/\_/\      /\_/\__/\/\                       |
 |                         \          \____/          /                       |
 |                          '----________________----'                        |
 |                              /                \                            |
 |                            O/_____/_______/____\O                          |
 |                            /____________________\                          |
 |                           /    (#UNIVERSITY#)    \                         |
 |                           |[**](#OFWATERLOO#)[**]|                         |
 |                           \______________________/                         |
 |                            |_""__|_,----,_|__""_|                          |
 |                            ! !                ! !                          |
 |                            '-'                '-'                          |
 |       __    _   _  _____  ___  __  _  ___  _    _  ___  ___   ____  ____   |
 |      /  \  | | | ||_   _|/ _ \|  \| |/ _ \| \  / |/ _ \/ _ \ /     |       |
 |     / /\ \ | |_| |  | |  ||_||| |\  |||_|||  \/  |||_||||_|| \===\ |====   |
 |    /_/  \_\|_____|  |_|  \___/|_| \_|\___/|_|\/|_|\___/\___/ ____/ |____   |
 |                                                                            |
 ******************************************************************************
 * ############################################################################
 *
 * File: angles.cpp
 * Desc: Implementation file for angle wrapping functions
 * Auth: Michael Smart <michael.smart@uwaterloo.ca>
 *       Pranav Ganti <nav.ganti@uwaterloo.ca>
 *
 * ############################################################################
*/

#include "wave_spatial_utils/angles.hpp"

namespace wave_spatial_utils {

double wrapToPi(double angle) {
    double wrapped_angle = wrapToTwoPi(angle + PI) - PI;
    return wrapped_angle;
}

double wrapToTwoPi(double angle) {
    double wrapped_angle = fmod(angle, 2 * PI);

    if (wrapped_angle < 0) {
        wrapped_angle += 2 * PI;
    }

    return wrapped_angle;
}

}  // namespace wave_spatial_utils
