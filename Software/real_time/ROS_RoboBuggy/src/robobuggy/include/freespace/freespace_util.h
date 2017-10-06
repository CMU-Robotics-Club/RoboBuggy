/* * libfreespace - library for communicating with Freespace devices
 *
 * Copyright 2013-15 Hillcrest Laboratories, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef FREESPACE_UTIL_H_
#define FREESPACE_UTIL_H_

#include "freespace/freespace_codecs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup util Utility Functions API
 *
 * This page describes the utility functions API for supporting
 * communications and operations with Freespace(r) devices.
 */

/** This struct is used to exchange values for a sensor.
 * It contains 4 axes so that it can hold quaternions.
 * It is used as a general container for sensors with 1 to 4 axes.
 * If a coordinate is not used it will be left empty.
 * Check the individual function definitions to see which coordinates are used.
 */
struct MultiAxisSensor {
    /** W-coordinate */
    float w;
    /** X-coordinate */ 
    float x;
    /** Y-coordinate */
    float y;
    /** Z-coordinate */
    float z;
};

/** @ingroup util
 *
 * Get the acceleration values from a MEOut packet.
 *
 * For MEOut Format 0 and 3 units are m/s^2.
 * For MEOut Format 1 units are g.
 *
 * @param meOutPkt A pointer to the MEOut packet to extract the acceleration from.
 * @param sensor A pointer to where to store the extracted values. Uses X, Y, Z coordinates.
 * @return 0 if successful
 *         -1 if the format flag was not set for the acceleration field
 *         -2 if the meOutPkt does not contain acceleration at all
 *         -3 if the format select number is unrecognized
 */
LIBFREESPACE_API int freespace_util_getAcceleration(struct freespace_MotionEngineOutput const * meOutPkt,
                                                    struct MultiAxisSensor * sensor);

/** @ingroup util
 *
 * Get the acceleration without gravity values from a MEOut packet.
 *
 * For MEOut Format 0 and 3 units are m/s^2.
 * For MEOut Format 1 units are g.
 *
 * @param meOutPkt A pointer to the MEOut packet to extract the acceleration no grav from.
 * @param sensor A pointer to where to store the extracted values. Uses X, Y, Z coordinates.
 * @return 0 if successful.
 *         -1 if the format flag was not set for the acceleration field.
 *         -2 if the meOutPkt does not contain acceleration at all.
 *         -3 if the format select number is unrecognized.
 */
LIBFREESPACE_API int freespace_util_getAccNoGravity(struct freespace_MotionEngineOutput const * meOutPkt,
                                                    struct MultiAxisSensor * sensor);

/** @ingroup util
 *
 * Get the angular velocity values from a MEOut packet.
 *
 * For MEOut Format 0 and 3 units are rads/s.
 * For MEOut Format 1 units are deg/s.
 *
 * @param meOutPkt A pointer to the MEOut packet to extract the angular velocity from.
 * @param sensor A pointer to where to store the extracted values. Uses X, Y, Z coordinates.
 * @return 0 if successful.
 *         -1 if the format flag was not set for the angVel field.
 *         -2 if the meOutPkt does not contain angVel at all.
 *         -3 if the format select number is unrecognized.
 */
LIBFREESPACE_API int freespace_util_getAngularVelocity(struct freespace_MotionEngineOutput const * meOutPkt,
                                                       struct MultiAxisSensor * sensor);

/** @ingroup util
 *
 * Get the magnetometer values from a MEOut packet.
 *
 * For MEOut Format 0 and 1 units are gauss.
 * For MEOut Format 3 units are uTesla.
 *
 * @param meOutPkt A pointer to the MEOut packet to extract the magnetometer from.
 * @param sensor A pointer to where to store the extracted values. Uses X, Y, Z coordinates.
 * @return 0 if successful.
 *         -1 if the format flag was not set for the mag field.
 *         -2 if the meOutPkt does not contain mag at all.
 *         -3 if the format select number is unrecognized.
 */
LIBFREESPACE_API int freespace_util_getMagnetometer(struct freespace_MotionEngineOutput const * meOutPkt,
                                                    struct MultiAxisSensor * sensor);

/** @ingroup util
 *
 * Get the temperature values from a MEOut packet.
 *
 * For MEOut Format 0 and 3 units are degrees C.
 *
 * @param meOutPkt A pointer to the MEOut packet to extract the magnetometer from.
 * @param sensor A pointer to where to store the extracted values. Uses W coordinate.
 * @return 0 if successful.
 *         -1 if the format flag was not set for the temperature field.
 *         -2 if the meOutPkt does not contain temperature at all.
 *         -3 if the format select number is unrecognized.
 */
LIBFREESPACE_API int freespace_util_getTemperature(struct freespace_MotionEngineOutput const * meOutPkt,
                                                   struct MultiAxisSensor * sensor);

/** @ingroup util
 *
 * Get the inclination values from a MEOut packet.
 *
 * For MEOut Format 1 units are degrees.
 *
 * @param meOutPkt A pointer to the MEOut packet to extract the inclination from.
 * @param sensor A pointer to where to store the extracted values. Uses X, Y, Z coordinates.
 * @return 0 if successful.
 *         -1 if the format flag was not set for the inclination field.
 *         -2 if the meOutPkt does not contain inclination at all.
 *         -3 if the format select number is unrecognized.
 */
LIBFREESPACE_API int freespace_util_getInclination(struct freespace_MotionEngineOutput const * meOutPkt,
                                                   struct MultiAxisSensor * sensor);

/** @ingroup util
 *
 * Get the compass heading from a MEOut packet.
 *
 * For MEOut Format 1 units are degrees.
 *
 * @param meOutPkt a pointer to the MEOut packet to extract the compass heading from.
 * @param sensor a pointer to where to store the extracted values. Uses X coordinate.
 * @return 0 if successful.
 *         -1 if the format flag was not set for the compass heading field.
 *         -2 if the meOutPkt does not contain compass heading at all.
 *         -3 if the format select number is unrecognized.
 */
LIBFREESPACE_API int freespace_util_getCompassHeading(struct freespace_MotionEngineOutput const * meOutPkt,
                                                      struct MultiAxisSensor * sensor);

/** @ingroup util
 *
 * Get the angular position values from a MEOut packet.
 *
 * @param meOutPkt A pointer to the MEOut packet to extract the angular position from.
 * @param sensor A pointer to where to store the extracted values. Uses W, X, Y, Z coordinates.
 * @return 0 if successful.
 *         -1 if the format flag was not set for the angular position field.
 *         -2 if the meOutPkt does not contain angular position at all.
 *         -3 if the format select number is unrecognized.
 */
LIBFREESPACE_API int freespace_util_getAngPos(struct freespace_MotionEngineOutput const * meOutPkt,
                                              struct MultiAxisSensor * sensor);

/** @ingroup util
 *
 * Get the activity classification from a MEOut packet.
 *
 * For MEOut Format 1 values are unitless - they indicate system states.
 *
 * @param meOutPkt A pointer to the MEOut packet to extract the activity classification from.
 * @param sensor A pointer to where to store the extracted values. The Activity Classification
 * is stored in the X coordinate and the Power Management Flags are stored in the Y coordinate.
 * @return 0 if successful.
 *         -1 if the format flag was not set for the activity classification field.
 *         -2 if the meOutPkt does not contain activity classification at all.
 *         -3 if the format select number is unrecognized.
 */
LIBFREESPACE_API int freespace_util_getActClass(struct freespace_MotionEngineOutput const * meOutPkt,
                                                struct MultiAxisSensor * sensor);

#ifdef __cplusplus
}
#endif

#endif /* FREESPACE_H_ */
