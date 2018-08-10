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

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "nmea_parser.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"

#define _UART_PORT(n) UART_NUM_##n
#define UART_PORT(n) _UART_PORT(n)

#define UART_NUM UART_PORT(CONFIG_UART_PORT_NUM)
#define UART_RX_PIN CONFIG_UART_PINS_RX
#define UART_BAUD_RATE CONFIG_RECEIVER_BAUDRATE

/**
 * @brief callback function when new gps statements were decoded
 * 
 */
static update_callback update_cb = NULL;

/**
 * @brief to Control the NMEA parser task to start and stop
 * 
 */
static uint8_t running = 0;

/**
 * @brief mutex to protect global variable
 * 
 */
SemaphoreHandle_t nmea_mutex = NULL;

/**
 * @brief GPS parser library runtime structure, only for library internal use
 * 
 */
typedef struct _gps_runtime
{
	uint8_t index;			  /*!< Statement index */
	char term_str[13];		  /*!< Current term in string format */
	uint8_t term_pos;		  /*!< Current index position in term */
	uint8_t term_num;		  /*!< Current term number */
	uint8_t star;			  /*!< Star detected flag */
	uint8_t crc_calc;		  /*!< Calculated CRC string */
	uint8_t parsed_statement; /*!< Statements have been parsed in a cycle */
	union {
		uint8_t dummy; //to make sure thers's something in the union
#if CONFIG_NMEA_STATEMENT_GGA
		struct
		{
			float latitude, longitude, altitude;
			uint8_t sats_in_use, fix, hours, minutes, seconds;
			uint16_t thousands;
		} gga;
#endif

#if CONFIG_NMEA_STATEMENT_GSA
		struct
		{
			float dop_h, dop_v, dop_p;
			uint8_t fix_mode;
		} gsa;
#endif

#if CONFIG_NMEA_STATEMENT_GSV
		struct
		{
			uint8_t sats_in_view, stat_num, stat_nums;
		} gsv;
#endif

#if CONFIG_NMEA_STATEMENT_RMC
		struct
		{
			uint8_t is_valid, day, month;
			uint16_t year;
			float speed, coarse, variation;
		} rmc;
#endif
	} data;
} gps_runtime_t;

/**
 * @brief macros of statement type
 * 
 */
#define STATEMENT_UNKNOWN (0)
#ifdef CONFIG_NMEA_STATEMENT_GGA
#define STATEMENT_GGA (1)
#else
#define STATEMENT_GGA (0)
#endif
#ifdef CONFIG_NMEA_STATEMENT_GSA
#define STATEMENT_GSA (2)
#else
#define STATEMENT_GSA (0)
#endif
#ifdef CONFIG_NMEA_STATEMENT_GSV
#define STATEMENT_GSV (3)
#else
#define STATEMENT_GSV (0)
#endif
#ifdef CONFIG_NMEA_STATEMENT_RMC
#define STATEMENT_RMC (4)
#else
#define STATEMENT_RMC (0)
#endif
#define STATEMENT_ALL ((1 << STATEMENT_GGA | 1 << STATEMENT_GSA | 1 << STATEMENT_GSV | 1 << STATEMENT_RMC) & 0xFE)

/**
 * @brief conversion between char and number(decimal or hex)
 * 
 */
#define CHARISNUM(x) ((x) >= '0' && (x) <= '9')
#define CHARISHEXNUM(x) (((x) >= '0' && (x) <= '9') || ((x) >= 'a' && (x) <= 'f') || ((x) >= 'A' && (x) <= 'F'))
#define CHARTONUM(x) ((x) - '0')
#define CHARHEXTONUM(x) (((x) >= '0' && (x) <= '9') ? ((x) - '0') : (((x) >= 'a' && (x) <= 'z') ? ((x) - 'a' + 10) : (((x) >= 'A' && (x) <= 'Z') ? ((x) - 'A' + 10) : 0)))

/**
 * @brief Earth radius in units of kilometers
 * 
 */
#define GPS_EARTH_RADIUS (6371.0f)

/**
 * @brief Degrees to radians
 * 
 */
#define GPS_DEGREES2RADIANS(x) (float)((x)*0.01745329251994f)

/**
 * @brief Radians to degrees
 * 
 */
#define GPS_RADIANS2DEGREES(x) (float)((x)*57.29577951308232f)

/**
 * @brief get GPS runtime structure from GPS handler
 * 
 */
#define RUNTIME(_gh) ((gps_runtime_t *)((_gh)->runtime))

/**
 * @brief push ch into CRC check
 * 
 */
#define CRC_ADD(_gh, ch)                         \
	do                                           \
	{                                            \
		RUNTIME(_gh)->crc_calc ^= (uint8_t)(ch); \
	} while (0)

/**
 * @brief push ch into the term container
 * 
 */
#define TERM_ADD(_gh, ch)                                        \
	do                                                           \
	{                                                            \
		RUNTIME(_gh)->term_str[RUNTIME(_gh)->term_pos++] = (ch); \
		RUNTIME(_gh)->term_str[RUNTIME(_gh)->term_pos] = 0;      \
	} while (0)

/**
 * @brief fresh the container to start a new term
 * 
 */
#define TERM_NEXT(_gh)                 \
	do                                 \
	{                                  \
		RUNTIME(_gh)->term_pos = 0;    \
		RUNTIME(_gh)->term_str[0] = 0; \
		RUNTIME(_gh)->term_num++;      \
	} while (0)

/**
 * @brief convert string to decimal number
 * 
 * @param gh gps handler
 * @param text string to be converted, if NULL, convert string in term_str
 * @return int conversion result
 */
static int parse_decimal_number(gps_t *gh, const char *text)
{
	int res = 0;
	uint8_t minus = 0;

	if (text == NULL)
	{
		text = RUNTIME(gh)->term_str;
	}
	minus = (*text == '-' ? (text++, 1) : 0);
	while (text != NULL && CHARISNUM(*text))
	{
		res = 10 * res + CHARTONUM(*text++);
	}
	return minus ? -res : res;
}

/**
 * @brief convert string to float number
 * 
 * @param gh gps handler
 * @param text string to be converted, if NULL, convert string in term_str
 * @return float conversion result
 */
static float parse_float_number(gps_t *gh, const char *text)
{
	double res = 0;
	if (text == NULL)
	{
		text = RUNTIME(gh)->term_str;
	}
	res = strtof(text, NULL);
	return res;
}

/**
 * @brief parse latitude/longitude NMEA format to float
 * 				NMEA output for latitude is ddmm.sss and longitude is dddmm.sss
 * @param gh gps handler
 * @return float Latitude/Longitude value in degrees
 */
static float parse_lat_long(gps_t *gh)
{
	float ll, deg, min;
	ll = parse_float_number(gh, NULL);
	deg = ((int)ll) / 100;
	min = ll - (deg * 100);
	ll = deg + min / 60.0f;
	return ll;
}

/**
 * @brief Parse received term
 * 
 * @param gh gps handle
 * @return esp_err_t 'ESP_OK' on success,'ESP_FAIL' on error
 */
static esp_err_t parse_term(gps_t *gh)
{
	//Firstly, determin type of the statement, should happen at the first comma of a statement
	if (RUNTIME(gh)->term_num == 0 && *(RUNTIME(gh)->term_str) == '$')
	{
		if (0)
		{
		}
#if CONFIG_NMEA_STATEMENT_GGA
		else if (strncmp(RUNTIME(gh)->term_str + 3, "GGA", 3) == 0)
		{
			RUNTIME(gh)->index = STATEMENT_GGA;
		}
#endif
#if CONFIG_NMEA_STATEMENT_GSA
		else if (strncmp(RUNTIME(gh)->term_str + 3, "GSA", 3) == 0)
		{
			RUNTIME(gh)->index = STATEMENT_GSA;
		}
#endif
#if CONFIG_NMEA_STATEMENT_GSV
		else if (strncmp(RUNTIME(gh)->term_str + 3, "GSV", 3) == 0)
		{
			RUNTIME(gh)->index = STATEMENT_GSV;
		}
#endif
#if CONFIG_NMEA_STATEMENT_RMC
		else if (strncmp(RUNTIME(gh)->term_str + 3, "RMC", 3) == 0)
		{
			RUNTIME(gh)->index = STATEMENT_RMC;
		}
#endif
		else
		{
			RUNTIME(gh)->index = STATEMENT_UNKNOWN;
			return ESP_FAIL;
		}
		return ESP_OK;
	}

	//Afterwards, do specific working depends on the type of the statement, should happended at the comma(except the first one) and the asterisk
	if (RUNTIME(gh)->index == STATEMENT_UNKNOWN)
	{
		return ESP_FAIL;
	}
#if CONFIG_NMEA_STATEMENT_GGA
	else if (RUNTIME(gh)->index == STATEMENT_GGA) /* Process GGA statement */
	{
		switch (RUNTIME(gh)->term_num)
		{
		case 1: /* Process UTC time */
			RUNTIME(gh)->data.gga.hours = 10 * CHARTONUM((RUNTIME(gh)->term_str[0])) + CHARTONUM((RUNTIME(gh)->term_str[1])) + CONFIG_GPS_TIME_ZONE;
			RUNTIME(gh)->data.gga.minutes = 10 * CHARTONUM((RUNTIME(gh)->term_str[2])) + CHARTONUM((RUNTIME(gh)->term_str[3]));
			RUNTIME(gh)->data.gga.seconds = 10 * CHARTONUM((RUNTIME(gh)->term_str[4])) + CHARTONUM((RUNTIME(gh)->term_str[5]));
			if (RUNTIME(gh)->term_str[6] == '.')
			{
				uint16_t tmp = 0;
				uint8_t i = 7;
				while (RUNTIME(gh)->term_str[i])
				{
					tmp = 10 * tmp + CHARTONUM(RUNTIME(gh)->term_str[i]);
					i++;
				}
				RUNTIME(gh)->data.gga.thousands = tmp;
			}
			break;
		case 2: /* Latitude */
			RUNTIME(gh)->data.gga.latitude = parse_lat_long(gh);
			break;
		case 3: /* Latitude north/south information */
			if (RUNTIME(gh)->term_str[0] == 'S' || RUNTIME(gh)->term_str[0] == 's')
			{
				RUNTIME(gh)->data.gga.latitude *= -1;
			}
			break;
		case 4: /* Longitude */
			RUNTIME(gh)->data.gga.longitude = parse_lat_long(gh);
			break;
		case 5: /* Longitude east/west information */
			if (RUNTIME(gh)->term_str[0] == 'W' || RUNTIME(gh)->term_str[0] == 'w')
			{
				RUNTIME(gh)->data.gga.longitude *= -1;
			}
			break;
		case 6: /* Fix status */
			RUNTIME(gh)->data.gga.fix = parse_decimal_number(gh, NULL);
			break;
		case 7: /* Satellites in use */
			RUNTIME(gh)->data.gga.sats_in_use = parse_decimal_number(gh, NULL);
			break;
		case 9: /* Altitude */
			RUNTIME(gh)->data.gga.altitude = parse_float_number(gh, NULL);
			break;
		case 11: /* Altitude above ellipsoid */
			RUNTIME(gh)->data.gga.altitude += parse_float_number(gh, NULL);
			break;
		default:
			break;
		}
	}
#endif
#if CONFIG_NMEA_STATEMENT_GSA
	else if (RUNTIME(gh)->index == STATEMENT_GSA) /* Process GSA statement */
	{
		switch (RUNTIME(gh)->term_num)
		{
		case 2: /* Process fix mode */
			RUNTIME(gh)->data.gsa.fix_mode = parse_decimal_number(gh, NULL);
			break;
		case 15: /* Process PDOP */
			RUNTIME(gh)->data.gsa.dop_p = parse_float_number(gh, NULL);
			break;
		case 16: /* Process HDOP */
			RUNTIME(gh)->data.gsa.dop_h = parse_float_number(gh, NULL);
			break;
		case 17: /* Process VDOP */
			RUNTIME(gh)->data.gsa.dop_v = parse_float_number(gh, NULL);
			break;
		default:
			/* Parse satellite IDs */
			if (RUNTIME(gh)->term_num >= 3 && RUNTIME(gh)->term_num <= 14)
			{
				gh->satellites_ids[RUNTIME(gh)->term_num - 3] = parse_decimal_number(gh, NULL);
			}
			break;
		}
	}
#endif
#if CONFIG_NMEA_STATEMENT_GSV
	else if (RUNTIME(gh)->index == STATEMENT_GSV) /* Process GSV statement */
	{
		switch (RUNTIME(gh)->term_num)
		{
		case 1: /* total GSV numbers */
			RUNTIME(gh)->data.gsv.stat_nums = parse_decimal_number(gh, NULL);
		case 2: /* Current GSV statement number */
			RUNTIME(gh)->data.gsv.stat_num = parse_decimal_number(gh, NULL);
			break;
		case 3: /* Process satellites in view */
			RUNTIME(gh)->data.gsv.sats_in_view = parse_decimal_number(gh, NULL);
			break;
		default:
			if (RUNTIME(gh)->term_num >= 4 && RUNTIME(gh)->term_num <= 19)
			{
				uint8_t term_num = RUNTIME(gh)->term_num - 4; /* Normalize term number from 4-19 to 0-15 */
				uint8_t index;
				uint16_t value;
				index = 4 * (RUNTIME(gh)->data.gsv.stat_num - 1) + term_num / 4; /* Get array index */
				if (index < GPS_MAX_SATS_IN_VIEW)
				{
					value = (uint16_t)parse_decimal_number(gh, NULL);
					switch (term_num % 4)
					{
					case 0:
						gh->sats_in_view_desc[index].num = value;
						break;
					case 1:
						gh->sats_in_view_desc[index].elevation = value;
						break;
					case 2:
						gh->sats_in_view_desc[index].azimuth = value;
						break;
					case 3:
						gh->sats_in_view_desc[index].snr = value;
						break;
					default:
						break;
					}
				}
			}
			break;
		}
	}
#endif
#if CONFIG_NMEA_STATEMENT_RMC
	else if (RUNTIME(gh)->index == STATEMENT_RMC) /* Process GPRMC statement */
	{
		switch (RUNTIME(gh)->term_num)
		{
		case 2: /* Process valid status */
			RUNTIME(gh)->data.rmc.is_valid = RUNTIME(gh)->term_str[0] == 'A';
			break;
		case 7: /* Process ground speed in knots */
			RUNTIME(gh)->data.rmc.speed = parse_float_number(gh, NULL);
			break;
		case 8: /* Process true ground coarse */
			RUNTIME(gh)->data.rmc.coarse = parse_float_number(gh, NULL);
			break;
		case 9: /* Process date */
			RUNTIME(gh)->data.rmc.day = 10 * CHARTONUM(RUNTIME(gh)->term_str[0]) + CHARTONUM(RUNTIME(gh)->term_str[1]);
			RUNTIME(gh)->data.rmc.month = 10 * CHARTONUM(RUNTIME(gh)->term_str[2]) + CHARTONUM(RUNTIME(gh)->term_str[3]);
			RUNTIME(gh)->data.rmc.year = 10 * CHARTONUM(RUNTIME(gh)->term_str[4]) + CHARTONUM(RUNTIME(gh)->term_str[5]) + 2000;
			break;
		case 10: /* Process magnetic variation */
			RUNTIME(gh)->data.rmc.variation = parse_float_number(gh, NULL);
			break;
		default:
			break;
		}
	}
#endif
	else
	{
		return ESP_FAIL;
	}
	return ESP_OK;
}

/**
 * @brief compare calculated CRC with received CRC
 * 
 * @param gh gps handle
 * @return esp_err_t 'ESP_OK' on success,'ESP_FAIL' on error
 */
static esp_err_t check_crc(gps_t *gh)
{
	uint8_t crc = ((CHARHEXTONUM(RUNTIME(gh)->term_str[0]) & 0xF) << 4) | ((CHARHEXTONUM(RUNTIME(gh)->term_str[1]) & 0xF)); /* Convert received CRC from string (hex) to number */
	return RUNTIME(gh)->crc_calc == crc ? ESP_OK : ESP_FAIL;
}

/**
 * @brief copy runtime data to user memory
 * 
 * @param gh gps handle
 * @return esp_err_t 'ESP_OK' on success,'ESP_FAIL' on error
 */
static esp_err_t copy_to_user_from_runtime(gps_t *gh)
{
	switch (RUNTIME(gh)->index)
	{
#if CONFIG_NMEA_STATEMENT_GGA
	case STATEMENT_GGA:
		gh->latitude = RUNTIME(gh)->data.gga.latitude;
		gh->longitude = RUNTIME(gh)->data.gga.longitude;
		gh->altitude = RUNTIME(gh)->data.gga.altitude;
		gh->sats_in_use = RUNTIME(gh)->data.gga.sats_in_use;
		gh->fix = RUNTIME(gh)->data.gga.fix;
		gh->tim.hours = RUNTIME(gh)->data.gga.hours;
		gh->tim.minutes = RUNTIME(gh)->data.gga.minutes;
		gh->tim.seconds = RUNTIME(gh)->data.gga.seconds;
		gh->tim.thousands = RUNTIME(gh)->data.gga.thousands;
		break;
#endif
#if CONFIG_NMEA_STATEMENT_GSA
	case STATEMENT_GSA:
		gh->dop_h = RUNTIME(gh)->data.gsa.dop_h;
		gh->dop_p = RUNTIME(gh)->data.gsa.dop_p;
		gh->dop_v = RUNTIME(gh)->data.gsa.dop_v;
		gh->fix_mode = RUNTIME(gh)->data.gsa.fix_mode;
		break;
#endif
#if CONFIG_NMEA_STATEMENT_GSV
	case STATEMENT_GSV:
		gh->sats_in_view = RUNTIME(gh)->data.gsv.sats_in_view;
		break;
#endif
#if CONFIG_NMEA_STATEMENT_RMC
	case STATEMENT_RMC:
		gh->coarse = RUNTIME(gh)->data.rmc.coarse;
		gh->is_valid = RUNTIME(gh)->data.rmc.is_valid;
		gh->speed = RUNTIME(gh)->data.rmc.speed;
		gh->variation = RUNTIME(gh)->data.rmc.variation;
		gh->date.day = RUNTIME(gh)->data.rmc.day;
		gh->date.month = RUNTIME(gh)->data.rmc.month;
		gh->date.year = RUNTIME(gh)->data.rmc.year;
		break;
#endif
	default:
		return ESP_FAIL;
		break;
	}
	return ESP_OK;
}

/**
 * @brief decode NMEA date from GPS receiver
 * 
 * @param gps_handler gps handle structure pointer
 * @param data received data
 * @param len number of bytes to decode
 * @return esp_err_t 'ESP_OK' on success,'ESP_FAIL' on error
 */
static esp_err_t gps_decode(gps_t *gps_handler, const void *data, size_t len)
{
	const uint8_t *d = data;
	while (len--) /* Process all bytes */
	{
		if (*d == '$') /* Check for beginning of NMEA line */
		{
			uint8_t tmp = RUNTIME(gps_handler)->parsed_statement;
			memset(RUNTIME(gps_handler), 0, sizeof(gps_runtime_t)); /* Reset runtime memory */
			RUNTIME(gps_handler)->parsed_statement = tmp;
			TERM_ADD(gps_handler, *d); /* Add character to term */
		}
		else if (*d == ',') /* Term separator character */
		{
			parse_term(gps_handler);  /* Parse term we have currently in memory */
			CRC_ADD(gps_handler, *d); /* Add character to CRC computation */
			TERM_NEXT(gps_handler);   /* Start with next term */
		}
		else if (*d == '*') /* Start indicates end of data for CRC computation */
		{
			parse_term(gps_handler);		/* Parse term we have currently in memory */
			RUNTIME(gps_handler)->star = 1; /* STAR detected */
			TERM_NEXT(gps_handler);			/* Start with next term */
		}
		else if (*d == '\r')
		{
			if (check_crc(gps_handler) == ESP_OK) /* Check for CRC result */
			{
				copy_to_user_from_runtime(gps_handler); /* Copy memory from temporary to user memory */
				switch (RUNTIME(gps_handler)->index)
				{
#if CONFIG_NMEA_STATEMENT_GGA
				case STATEMENT_GGA:
					RUNTIME(gps_handler)->parsed_statement |= 1 << STATEMENT_GGA;
					break;
#endif
#if CONFIG_NMEA_STATEMENT_GSA
				case STATEMENT_GSA:
					RUNTIME(gps_handler)->parsed_statement |= 1 << STATEMENT_GSA;
					break;
#endif
#if CONFIG_NMEA_STATEMENT_RMC
				case STATEMENT_RMC:
					RUNTIME(gps_handler)->parsed_statement |= 1 << STATEMENT_RMC;
					break;
#endif
#if CONFIG_NMEA_STATEMENT_GSV
				case STATEMENT_GSV:
					if (RUNTIME(gps_handler)->data.gsv.stat_num == RUNTIME(gps_handler)->data.gsv.stat_nums)
					{
						RUNTIME(gps_handler)->parsed_statement |= 1 << STATEMENT_GSV;
					}
					break;
#endif
				default:
					break;
				}
			}
		}
		else if (*d != ' ') /* Any other character which is not space */
		{
			if (!(RUNTIME(gps_handler)->star)) /* Add to CRC only if star not yet detected */
			{
				CRC_ADD(gps_handler, *d); /* Add to CRC */
			}
			TERM_ADD(gps_handler, *d); /* Add character to term */
		}
		if (((RUNTIME(gps_handler)->parsed_statement) & STATEMENT_ALL) == STATEMENT_ALL)
		{
			RUNTIME(gps_handler)->parsed_statement = 0;
			xSemaphoreTake(nmea_mutex, portMAX_DELAY);
			if (update_cb)
			{
				update_cb(gps_handler);
			}
			xSemaphoreGive(nmea_mutex);
		}
		d++; /* Process next character */
	}
	return ESP_OK;
}

/**
 * @brief new GPS handle
 * 
 * @return gps_t* GPS handle structure pointer
 */
static gps_t *gps_new(void)
{
	gps_t *gh = malloc(sizeof(gps_t) + sizeof(gps_runtime_t));
	memset(gh, 0, sizeof(gps_t) + sizeof(gps_runtime_t)); /* Reset structure */
	return gh;
}

/**
 * @brief delete GPS handle
 * 
 * @param gps_handler gps handle
 * @return esp_err_t 'ESP_OK' on success,'ESP_FAIL' on error
 */
static esp_err_t gps_delete(gps_t *gps_handler)
{
	if (gps_handler)
	{
		free(gps_handler);
		return ESP_OK;
	}
	return ESP_FAIL;
}

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
esp_err_t gps_distance_bearing(float las, float los, float lae, float loe, float *d, float *b)
{
	float df, dfi, a;
	if (d == NULL && b == NULL)
	{
		return ESP_FAIL;
	}

	/* Calculate distance between 2 pointes */
	df = GPS_DEGREES2RADIANS(lae - las);
	dfi = GPS_DEGREES2RADIANS(loe - los);
	las = GPS_DEGREES2RADIANS(las);
	lae = GPS_DEGREES2RADIANS(lae);
	los = GPS_DEGREES2RADIANS(los);
	loe = GPS_DEGREES2RADIANS(loe);

	/*
     * Calculate distance
     *
     * Calculated distance is absolute value in meters between 2 points on earth.
     */
	if (d != NULL)
	{
		/**
		 * a = sin(df / 2)^2 + cos(las) * cos(lae) * sin(dfi / 2)^2
		 * *d = RADIUS * 2 * atan(a / (1 - a)) * 1000 (for meters)
		 */
		a = (float)(sin(df * 0.5f) * sin(df * 0.5f) + cos(las) * cos(lae) * sin(dfi * 0.5f) * sin(dfi * 0.5f));
		*d = (float)(GPS_EARTH_RADIUS * 2.0f * atan2(sqrt(a), sqrt(1 - a)) * 1000.0f);
	}

	/*
     * Calculate bearing
     *
     * Bearing is calculated from point 1 to point 2.
     * Result will tell us in which direction (according to north) we should move,
     * to reach point 2.
     *
     * Example: 
     *      Bearing is 0 => move to north
     *      Bearing is 90 => move to east
     *      Bearing is 180 => move to south
     *      Bearing is 270 => move to west
     */
	if (b != NULL)
	{
		df = (float)(sin(loe - los) * cos(lae));
		dfi = (float)(cos(las) * sin(lae) - sin(las) * cos(lae) * cos(loe - los));

		*b = GPS_RADIANS2DEGREES(atan2(df, dfi)); /* Calculate bearing and convert to degrees */
		if (*b < 0)
		{						/* Check for negative angle */
			*b += (float)360.0; /* Make bearing always positive */
		}
	}
	return ESP_OK;
}

/**
 * @brief Convert NMEA GPS speed (in knots = nautical mile per hour) to different speed format
 * 
 * @param speed_in_knots Speed in knots, received from GPS NMEA statement
 * @param to_speed Target speed to convert to from knots
 * @return float Speed calculated from knots
 */
float gps_to_speed(float speed_in_knots, gps_speed_t to_speed)
{
	switch (to_speed)
	{
	case gps_speed_kps:
		return (float)(speed_in_knots * 0.000514L);
	case gps_speed_kph:
		return (float)(speed_in_knots * 0.5144L);
	case gps_speed_mps:
		return (float)(speed_in_knots * 1.852L);
	case gps_speed_mpm:
		return (float)(speed_in_knots * 30.87L);

	case gps_speed_mips:
		return (float)(speed_in_knots * 0.0003197L);
	case gps_speed_mph:
		return (float)(speed_in_knots * 1.151L);
	case gps_speed_fps:
		return (float)(speed_in_knots * 1.688L);
	case gps_speed_fpm:
		return (float)(speed_in_knots * 101.3L);

	case gps_speed_mpk:
		return (float)(speed_in_knots * 32.4L);
	case gps_speed_spk:
		return (float)(speed_in_knots * 1944.0L);
	case gps_speed_sp100m:
		return (float)(speed_in_knots * 194.4L);
	case gps_speed_mipm:
		return (float)(speed_in_knots * 52.14L);
	case gps_speed_spm:
		return (float)(speed_in_knots * 3128.0L);
	case gps_speed_sp100y:
		return (float)(speed_in_knots * 177.7L);

	case gps_speed_smph:
		return (float)(speed_in_knots * 1.0L);
	default:
		return 0;
	}
}

/**
 * @brief nmea_parser task function
 * 
 * @param arg gps handler
 */
static void nmea_parser_task(void *arg)
{
	gps_t *gh = (gps_t *)arg;
	uint8_t *data = (uint8_t *)malloc(CONFIG_NMEA_PARSER_BUFFER_SIZE + 1);
	int length = 0;
	xSemaphoreTake(nmea_mutex, portMAX_DELAY);
	while (running)
	{
		xSemaphoreGive(nmea_mutex);
		uart_get_buffered_data_len(UART_NUM, (size_t *)&length);
		length = uart_read_bytes(UART_NUM, data, length, 100 / portTICK_PERIOD_MS);
		if (length > 0)
		{
			gps_decode(gh, data, length);
		}
		vTaskDelay(200 / portTICK_PERIOD_MS);
		xSemaphoreTake(nmea_mutex, portMAX_DELAY);
	}
	xSemaphoreGive(nmea_mutex);
	vSemaphoreDelete(nmea_mutex);
	nmea_mutex = NULL;
	update_cb = NULL;
	free(data);
	vTaskDelete(NULL);
}

/**
 * @brief start nmea_parser task
 * 
 * @param update callback function
 * @return gps_t* gps handler
 */
gps_t *nmea_parser_start(update_callback update)
{
	gps_t *gh = gps_new();
	if (update)
	{
		update_cb = update;
	}
	nmea_mutex = xSemaphoreCreateMutex();
	/* config parameters of an UART driver */
	uart_config_t uart_config = {
		.baud_rate = UART_BAUD_RATE,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
	uart_param_config(UART_NUM, &uart_config);
	uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	uart_driver_install(UART_NUM, CONFIG_NMEA_PARSER_BUFFER_SIZE, 0, 0, NULL, 0);
	uart_flush(UART_NUM);
	xSemaphoreTake(nmea_mutex, portMAX_DELAY);
	running = 1;
	xSemaphoreGive(nmea_mutex);
	xTaskCreate(
		nmea_parser_task,
		"nmea_parser_task",
		CONFIG_NMEA_TASK_STACK_SIZE,
		gh,
		CONFIG_NMEA_TASK_PRIORITY,
		NULL);
	return gh;
}

/**
 * @brief stop nmea_parser task
 * 
 * @param gps_handle gps handler
 * @return esp_err_t 'ESP_OK' on success,'ESP_FAIL' on error
 */
esp_err_t nmea_parser_stop(gps_t *gps_handle)
{
	xSemaphoreTake(nmea_mutex, portMAX_DELAY);
	running = 0;
	xSemaphoreGive(nmea_mutex);
	gps_delete(gps_handle);
	uart_driver_delete(UART_NUM);
	return ESP_OK;
}
