# nmea-parser

nmea-parser  is a light weight for parsing NMEA-0183 data streams provided by GPS/BDS/GLONASS modules.
Up to Now, it mainly decodes 4 statements:
* GGA
* GSA
* GSV
* RMC



## Currently Supported Modules(tested)

* [ATGM332D-5N](http://www.icofchina.com/pro/mokuai/2016-08-01/5.html)

## How to Use
1. `make menuconfig` :choose the statement you want to decode;choose the UART and related GPIO Pin you want to connect
2. `make flash monitor` :build && flash && watch the result in terminal

## Reference Outcome

```bash
I (2671) nmea_parser: 15:55:4.0 2018-8-9=>latitude=31.201790,longtitude=121.579391,altitude=37.000000
I (3671) nmea_parser: 15:55:5.0 2018-8-9=>latitude=31.201790,longtitude=121.579391,altitude=37.299999
I (4671) nmea_parser: 15:55:6.0 2018-8-9=>latitude=31.201790,longtitude=121.579391,altitude=37.299999
I (5671) nmea_parser: 15:55:7.0 2018-8-9=>latitude=31.201790,longtitude=121.579391,altitude=37.500000
I (6671) nmea_parser: 15:55:8.0 2018-8-9=>latitude=31.201790,longtitude=121.579391,altitude=37.599998
I (7671) nmea_parser: 15:55:9.0 2018-8-9=>latitude=31.201790,longtitude=121.579391,altitude=37.700001
I (8671) nmea_parser: 15:55:10.0 2018-8-9=>latitude=31.201790,longtitude=121.579391,altitude=37.700001
I (9671) nmea_parser: 15:55:11.0 2018-8-9=>latitude=31.201790,longtitude=121.579391,altitude=37.700001
I (10671) nmea_parser: 15:55:12.0 2018-8-9=>latitude=31.201790,longtitude=121.579391,altitude=37.700001
I (17671) nmea_parser: 15:55:19.0 2018-8-9=>latitude=31.201775,longtitude=121.579376,altitude=36.000000
I (18671) nmea_parser: 15:55:20.0 2018-8-9=>latitude=31.201778,longtitude=121.579376,altitude=36.000000
I (19671) nmea_parser: 15:55:21.0 2018-8-9=>latitude=31.201778,longtitude=121.579376,altitude=36.000000
I (20671) nmea_parser: 15:55:22.0 2018-8-9=>latitude=31.201778,longtitude=121.579361,altitude=36.099998
I (21671) nmea_parser: 15:55:23.0 2018-8-9=>latitude=31.201778,longtitude=121.579361,altitude=36.299999
I (22671) nmea_parser: 15:55:24.0 2018-8-9=>latitude=31.201778,longtitude=121.579361,altitude=36.500000
I (23671) nmea_parser: 15:55:25.0 2018-8-9=>latitude=31.201782,longtitude=121.579361,altitude=36.900002
I (24671) nmea_parser: 15:55:26.0 2018-8-9=>latitude=31.201782,longtitude=121.579361,altitude=36.799999
I (25671) nmea_parser: 15:55:27.0 2018-8-9=>latitude=31.201782,longtitude=121.579361,altitude=36.799999
```

