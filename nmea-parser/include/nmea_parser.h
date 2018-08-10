// Copyright 2015-2018 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_types.h"
#include "esp_err.h"
#include "sdkconfig.h"

/**
 * @brief size of the internel buffer used for parser, byte unit
 * 
 */
#define GPS_BUFFER_SIZE CONFIG_NMEA_PARSER_BUFFER_SIZE

/**
 * @brief maximum amount of visible satellites
 * 
 */
#define GPS_MAX_SATS_IN_VIEW CONFIG_MAX_SATELLITES_IN_VIEW

/**
 * @brief typedef of the callback function when new gps statements have been decoded
 * 
 */
typedef void (*update_callback)(void* arg);

/**
 * @brief GPS fix type enumeration
 * 
 */
typedef enum _gps_fix
{
	GPS_Fix_Invalid = 0,
	GPS_Fix_GPS = 1,
	GPS_Fix_DGPS = 2,
	GPS_Fix_PPS = 3
} gps_fix_t;

/**
 * @brief GPS fix mode enumeration
 * 
 */
typedef enum _gps_fixmode
{
	GPS_FixMode_Invalid = 1,
	GPS_FixMode_2D = 2,
	GPS_FixMode_3D = 3
} gps_fix_mode_t;

/**
 * @brief List of optional speed transformation from GPS values (in knots)
 * 
 */
typedef enum _gps_speed
{
	/* Metric values */
    gps_speed_kps,    /*!< Kilometers per second */
    gps_speed_kph,    /*!< Kilometers per hour */
    gps_speed_mps,    /*!< Meters per second */
    gps_speed_mpm,    /*!< Meters per minute */

    /* Imperial values */
    gps_speed_mips,   /*!< Miles per second */
    gps_speed_mph,    /*!< Miles per hour */
    gps_speed_fps,    /*!< Foots per second */
    gps_speed_fpm,    /*!< Foots per minute */

    /* Optimized for runners/joggers */
    gps_speed_mpk,    /*!< Minutes per kilometer */
    gps_speed_spk,    /*!< Seconds per kilometer */
    gps_speed_sp100m, /*!< Seconds per 100 meters */
    gps_speed_mipm,   /*!< Minutes per mile */
    gps_speed_spm,    /*!< Seconds per mile */
    gps_speed_sp100y, /*!< Seconds per 100 yards */

    /* Nautical values */
    gps_speed_smph,  /*!< Sea miles per hour */
} gps_speed_t;

/**
 * @brief GPS Satellite descriptor
 * 
 */
typedef struct _gps_satellite
{
	uint8_t num;	
	uint8_t elevation;
	uint16_t azimuth; 
	uint8_t snr;
} gps_satellite_t;

/**
 * @brief GPS time descriptor
 * 
 */
typedef struct _gps_time
{
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	uint16_t thousands; 
} gps_time_t;

/**
 * @brief GPS date descriptor
 * 
 */
typedef struct _gps_date
{
	uint8_t day; 
	uint8_t month;
	uint16_t year;
} gps_date_t;

/**
 * @brief GPS object descriptor
 * 
 */
typedef struct _gps
{
#if CONFIG_NMEA_STATEMENT_GGA || __DOXYGEN__
	float latitude;	/*!< Latitude in units of degrees */
	float longitude; /*!< Longitude in units of degrees */
	float altitude;	/*!< Altitude in units of meters */
	gps_fix_t fix;	 /*!< Fix status */
	uint8_t sats_in_use; /*!< Number of satellites in use */
	gps_time_t tim; /*!< time in UTC */
#endif

#if CONFIG_NMEA_STATEMENT_GSA || __DOXYGEN__
	gps_fix_mode_t fix_mode;	/*!< Fix mode */
	uint8_t satellites_ids[GPS_MAX_SATS_IN_VIEW]; /*!< List of satellite IDs in use. Valid range is `0` to `sats_in_use` */
	float dop_h;	/*!< Dolution of precision, horizontal */
	float dop_p;	 /*!< Dolution of precision, position */
	float dop_v;	 /*!< Dolution of precision, vertical */
#endif

#if CONFIG_NMEA_STATEMENT_GSV || __DOXYGEN__
	uint8_t sats_in_view;				/*!< Number of satellites in view in gps system */
	gps_satellite_t sats_in_view_desc[GPS_MAX_SATS_IN_VIEW]; 
#endif

#if CONFIG_NMEA_STATEMENT_RMC || __DOXYGEN__
	gps_date_t date; /*!< Fix date */
	uint8_t is_valid;   /*!< GPS valid status */
	float speed;	 /*!< Ground speed in knots */
	float coarse;	 /*!< Ground coarse */
	float variation;/*!< Magnetic variation */
#endif

#if !__DOXYGEN__
	char runtime[0];
#endif
} gps_t;

/**
 * @brief Calculate distance and bearing
 * 
 * @param las Latitude start coordinate, in units of degrees
 * @param los Longitude start coordinate, in units of degrees
 * @param lae Latitude end coordinate, in units of degrees
 * @param loe Longitude end coordinate, in units of degrees
 * @param d Pointer to output distance in units of meters
 * @param b Pointer to output bearing between start and end coordinate in relation to north in units of degrees
 * @return esp_err_t 'ESP_OK' on success,'ESP_FAIL' on error
 */
esp_err_t gps_distance_bearing(float las, float los, float lae, float loe, float* d, float* b);

/**
 * @brief Convert NMEA GPS speed (in knots = nautical mile per hour) to different speed format
 * 
 * @param speed_in_knots Speed in knots, received from GPS NMEA statement
 * @param to_speed Target speed to convert to from knots
 * @return float Speed calculated from knots
 */
float gps_to_speed(float speed_in_knots, gps_speed_t to_speed);

/**
 * @brief start nmea_parser task
 * 
 * @param update callback function
 * @return gps_t* gps handler
 */
gps_t *nmea_parser_start(update_callback update);

/**
 * @brief stop nmea_parser task
 * 
 * @param gps_handle gps handler
 * @return esp_err_t 'ESP_OK' on success,'ESP_FAIL' on error
 */
esp_err_t nmea_parser_stop(gps_t *gps_handle);

#ifdef __cplusplus
}
#endif
