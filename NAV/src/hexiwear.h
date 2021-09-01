/**
 * hexiwear.h
 * definice pro komunikaci s Hexiwear platformou
 * xkoupy00 - Pavel Koupy
 *
 */
#pragma once

/**
 * konstanty  - topics a uuid pro Hexiwear
 */
constexpr auto MQTT_HEXIWEAR_BAT_SERVICE_LEVEL  	= "hexiwear/2A19/battery_service/battery_level";
constexpr auto MQTT_HEXIWEAR_APP_SERVICE_APPMODE	= "hexiwear/2041/appmode_service/app";
constexpr auto MQTT_HEXIWEAR_MOTION_SERVICE_ACC  	= "hexiwear/2001/motion_service/accelerometr";
constexpr auto MQTT_HEXIWEAR_MOTION_SERVICE_GYRO  	= "hexiwear/2002/motion_service/gyroscope";
constexpr auto MQTT_HEXIWEAR_MOTION_SERVICE_MAG 	= "hexiwear/2003/motion_service/magnetometr";
constexpr auto MQTT_HEXIWEAR_WEATHER_SERVICE_AMB	= "hexiwear/2011/weather_service/ambient_light";
constexpr auto MQTT_HEXIWEAR_WEATHER_SERVICE_TEMP	= "hexiwear/2012/weather_service/temperature";
constexpr auto MQTT_HEXIWEAR_WEATHER_SERVICE_HUMID	= "hexiwear/2013/weather_service/humidity";
constexpr auto MQTT_HEXIWEAR_WEATHER_SERVICE_PRESS	= "hexiwear/2014/weather_service/pressure";
constexpr auto MQTT_HEXIWEAR_HEALTH_SERVICE_HR		= "hexiwear/2021/health_service/heart_rate";
constexpr auto MQTT_HEXIWEAR_HEALTH_SERVICE_STEPS	= "hexiwear/2022/health_service/steps";
constexpr auto MQTT_HEXIWEAR_HEALTH_SERVICE_CAL		= "hexiwear/2023/health_service/calories";
constexpr auto MQTT_HEXIWEAR_ALERT_SERVICE_IN		= "hexiwear/2031/alert_service/alert_in";
constexpr auto MQTT_HEXIWEAR_ALERT_SERVICE_OUT		= "hexiwear/2032/alert_service/alert_out";

constexpr auto BT_UUID_HEXIWEAR_BAT_SERVICE_LEVEL		= "00002A19-0000-1000-8000-00805f9b34fb";
constexpr auto BT_UUID_HEXIWEAR_APP_SERVICE_APPMODE		= "00002041-0000-1000-8000-00805f9b34fb";
constexpr auto BT_UUID_HEXIWEAR_MOTION_SERVICE_ACC		= "00002001-0000-1000-8000-00805f9b34fb";
constexpr auto BT_UUID_HEXIWEAR_MOTION_SERVICE_GYRO		= "00002002-0000-1000-8000-00805f9b34fb";
constexpr auto BT_UUID_HEXIWEAR_MOTION_SERVICE_MAG		= "00002003-0000-1000-8000-00805f9b34fb";
constexpr auto BT_UUID_HEXIWEAR_WEATHER_SERVICE_AMB		= "00002011-0000-1000-8000-00805f9b34fb";
constexpr auto BT_UUID_HEXIWEAR_WEATHER_SERVICE_TEMP	= "00002012-0000-1000-8000-00805f9b34fb";
constexpr auto BT_UUID_HEXIWEAR_WEATHER_SERVICE_HUMID	= "00002013-0000-1000-8000-00805f9b34fb";
constexpr auto BT_UUID_HEXIWEAR_WEATHER_SERVICE_PRESS	= "00002014-0000-1000-8000-00805f9b34fb";
constexpr auto BT_UUID_HEXIWEAR_HEALTH_SERVICE_HR		= "00002021-0000-1000-8000-00805f9b34fb";
constexpr auto BT_UUID_HEXIWEAR_HEALTH_SERVICE_STEPS	= "00002022-0000-1000-8000-00805f9b34fb";
constexpr auto BT_UUID_HEXIWEAR_HEALTH_SERVICE_CAL		= "00002023-0000-1000-8000-00805f9b34fb";
constexpr auto BT_UUID_HEXIWEAR_ALERT_SERVICE_IN		= "00002031-0000-1000-8000-00805f9b34fb";
constexpr auto BT_UUID_HEXIWEAR_ALERT_SERVICE_OUT		= "00002032-0000-1000-8000-00805f9b34fb";

