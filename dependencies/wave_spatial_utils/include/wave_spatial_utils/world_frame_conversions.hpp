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
 * File: world_frame_conversions.hpp
 * Desc: Header file for world frame conversion functions
 * Auth: Michael Smart <michael.smart@uwaterloo.ca>
 *
 * ############################################################################
*/

#ifndef WAVE_SPATIAL_UTILS_WORLD_FRAME_CONVERSIONS_HPP
#define WAVE_SPATIAL_UTILS_WORLD_FRAME_CONVERSIONS_HPP

#include <cmath>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace wave_spatial_utils {

/** Converts a point from LLH (Latitude [deg], Longitude [deg], Height[m]) to
 *  ECEF.
 *
 *  @param[in] llh the input llh point as (Latitude, Longitude, Height). Height
 *  is relative to the WGS84 ellipsoid.
 *  @param[out] ecef the corresponding point in the geocentric ECEF frame.
 */
void ecefPointFromLLH(const double llh[3], double ecef[3]);

/** Converts a point from LLH (Latitude [deg], Longitude [deg], Height[m]) to
 *  ECEF.
 *
 *  @param[in] ecef the input point in the geocentric ECEF frame.
 *  @param[out] llh the corresponding llh point as (Latitude, Longitude,
 *  Height). Height is relative to the WGS84 ellipsoid.
 */
void llhPointFromECEF(const double ecef[3], double llh[3]);

/** Computes the 3D Affine transform from ECEF to a local datum-defined ENU
 *  frame as a 4x4 row-major matrix.
 *
 *  @param[in] datum the LLH datum point defining the transform. If
 *  /p datum_is_LLH is set to false, then the datum values are taken as ECEF
 *  instead.
 *  @param[out] T_enu_ecef the 4x4 row-major transformation matrix converting
 *  column-vector points from ECEF to the local ENU frame defined by the datum
 *  point.
 *  @param[in] datum_is_llh \b true: The given datum values are LLH
 *  (default). <BR>
 *  \b false: The given datum values are ECEF
 */
void enuFromECEFTransformMatrix(const double datum[3],
                                double T_enu_ecef[4][4],
                                bool datum_is_llh = true);

/** Computes the 3D Affine transform from a local datum-defined ENU frame to
 *  ECEF as a 4x4 row-major matrix.
 *
 *  @param[in] datum the LLH datum point defining the transform. If
 *  /p datum_is_LLH is set to false, then the datum values are taken as ECEF
 *  instead.
 *  @param[out] T_ecef_enu the 4x4 row-major transformation matrix converting
 *  column-vector points from the local ENU frame defined by the datum point to
 *  ECEF.
 *  @param[in] datum_is_llh \b true: The given datum values are LLH
 *  (default). <BR>
 *  \b false: The given datum values are ECEF
 */
void ecefFromENUTransformMatrix(const double datum[3],
                                double T_ecef_enu[4][4],
                                bool datum_is_llh = true);

/** Converts a source point from LLH to a target ENU point in the local
 *  Cartesian ENU frame defined by the provided datum.
 *
 *  @param[in] point_llh the source LLH point to be converted to an ENU point.
 *  @param[in] enu_datum the LLH datum point defining the local ENU frame. If
 *  /p datum_is_LLH is set to false, then the datum values are taken as ECEF
 *  instead.
 *  @param[out] point_enu the corresponding target point in the local ENU frame.
 *  @param[in] datum_is_llh \b true: The given datum values are LLH
 *  (default). <BR>
 *  \b false: The given datum values are ECEF
 */
void enuPointFromLLH(const double point_llh[3],
                     const double enu_datum[3],
                     double point_enu[3],
                     bool datum_is_llh = true);

/** Converts a source point from ENU in the local Cartesian ENU frame
 *  defined by the provided datum to a target LLH point.
 *
 *  @param[in] point_enu the source ENU point to be converted to an LLH point.
 *  @param[in] enu_datum the LLH datum point defining the local ENU frame. If
 *  /p datum_is_LLH is set to false, then the datum values are taken as ECEF
 *  instead.
 *  @param[out] point_llh the corresponding target point in LLH.
 *  @param[in] datum_is_llh \b true: The given datum values are LLH
 *  (default). <BR>
 *  \b false: The given datum values are ECEF
 */
void llhPointFromENU(const double point_enu[3],
                     const double enu_datum[3],
                     double point_llh[3],
                     bool datum_is_llh = true);

/** Converts a source point from LLH to a target ENU point in the local
 *  Cartesian ENU frame defined by the provided datum.
 *
 *  @param[in] fix the source LLH point to be converted to an ENU point.
 *  @param[in] datum the LLH datum point defining the local ENU frame.
 *  @param[out] point_ptr the corresponding target point in the local ENU frame.
 */
void fix_to_point(const sensor_msgs::NavSatFix &fix,
                  const sensor_msgs::NavSatFix &datum,
                  geometry_msgs::Point *point_ptr);

/** Converts a source point from ENU in the local Cartesian ENU frame
 *  defined by the provided datum to a target LLH point.
 *
 *  @param[in] point the source ENU point to be converted to an LLH point.
 *  @param[in] datum the datum point defining the local ENU frame.
 *  @param[out] fix_ptr the corresponding target point in LLH.
 */
void point_to_fix(const geometry_msgs::Point &point,
                  const sensor_msgs::NavSatFix &datum,
                  sensor_msgs::NavSatFix *fix_ptr);

/** Creates a tf::Transform taking points from ECEF to ENU for a given datum.
 *
 *  @param[in] datum the datum point defining the local ENU frame.
 *  @return the tf::Transform taking points from ECEF to ENU for a given datum.
 */
tf::Transform makeTfENUFromECEF(const sensor_msgs::NavSatFix &datum);

/** Creates a tf2::Transform taking points from ECEF to ENU for a given datum.
 *
 *  @param[in] datum the datum point defining the local ENU frame.
 *  @return the tf2::Transform taking points from ECEF to ENU for a given datum.
 */
tf2::Transform makeTf2ENUFromECEF(const sensor_msgs::NavSatFix &datum);

/** Creates a tf::Transform taking points from ECEF to ENU for a given datum.
 *
 *  @param[in] datum the LLH datum point defining the transform. If
 *  /p datum_is_LLH is set to false, then the datum values are taken as ECEF
 *  instead.
 *  @param[in] datum_is_llh \b true: The given datum values are LLH
 *  (default). <BR>
 *  \b false: The given datum values are ECEF
 *  @return the tf::Transform taking points from ECEF to ENU for a given datum.
 */
tf::Transform makeTfENUFromECEF(const double datum[3],
                                bool datum_is_llh = true);

/** Creates a tf2::Transform taking points from ECEF to ENU for a given datum.
 *
 *  @param[in] datum the LLH datum point defining the transform. If
 *  /p datum_is_LLH is set to false, then the datum values are taken as ECEF
 *  instead.
 *  @param[in] datum_is_llh \b true: The given datum values are LLH
 *  (default). <BR>
 *  \b false: The given datum values are ECEF
 *  @return the tf2::Transform taking points from ECEF to ENU for a given datum.
 */
tf2::Transform makeTf2ENUFromECEF(const double datum[3],
                                  bool datum_is_llh = true);

/** Creates a tf::Transform taking points from ENU to ECEF for a given datum.
 *
 *  @param[in] datum the datum point defining the local ENU frame.
 *  @return the tf::Transform taking points from ENU to ECEF for a given datum.
 */
tf::Transform makeTfECEFFromENU(const sensor_msgs::NavSatFix &datum);

/** Creates a tf2::Transform taking points from ENU to ECEF for a given datum.
 *
 *  @param[in] datum the datum point defining the local ENU frame.
 *  @return the tf2::Transform taking points from ENU to ECEF for a given datum.
 */
tf2::Transform makeTf2ECEFFromENU(const sensor_msgs::NavSatFix &datum);

/** Creates a tf::Transform taking points from ENU to ECEF for a given datum.
 *
 *  @param[in] datum the LLH datum point defining the transform. If
 *  /p datum_is_LLH is set to false, then the datum values are taken as ECEF
 *  instead.
 *  @param[in] datum_is_llh \b true: The given datum values are LLH
 *  (default). <BR>
 *  \b false: The given datum values are ECEF
 *  @return the tf::Transform taking points from ENU to ECEF for a given datum.
 */
tf::Transform makeTfECEFFromENU(const double datum[3],
                                bool datum_is_llh = true);

/** Creates a tf2::Transform taking points from ENU to ECEF for a given datum.
 *
 *  @param[in] datum the LLH datum point defining the transform. If
 *  /p datum_is_LLH is set to false, then the datum values are taken as ECEF
 *  instead.
 *  @param[in] datum_is_llh \b true: The given datum values are LLH
 *  (default). <BR>
 *  \b false: The given datum values are ECEF
 *  @return the tf2::Transform taking points from ENU to ECEF for a given datum.
 */
tf2::Transform makeTf2ECEFFromENU(const double datum[3],
                                  bool datum_is_llh = true);

}  // namespace wave_spatial_utils
#endif  // WAVE_SPATIAL_UTILS_WORLD_FRAME_CONVERSIONS_HPP
