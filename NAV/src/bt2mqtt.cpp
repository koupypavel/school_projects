/**
 * main.cpp
 * hlavni funkce premosteni bt a mqtt
 * xkoupy00 - Pavel Koupy
 */
#include <iostream>
#include <vector>
#include <string>
#include <sys/types.h>
#include <cstdint>
#include <iterator>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <thread>
#include <csignal>
#include <utility>
#include <atomic>
#include <sstream>
#include <iomanip>

#include "mqtt.h"
#include "bt.h"
#include "hexiwear.h"

//parametry pro spusteni
constexpr auto MQTT_CLIENT_ID		= "bt2mqtt";
constexpr auto MQTT_BROKER_ADDRESS 	= "localhost";
constexpr auto MQTT_BROKER_PORT 	= 1883;
constexpr auto MQTT_BROKER_PASSWD 	= "nav";
constexpr auto HEXIWEAR_MAC = "00:1E:50:04:00:03";
constexpr auto HEXIWEAR_ID = 123456;

std::atomic_bool run(true);

/**
 *zpracovani signalu,  pro ukonceni programu
 */
void signalHandler( int signum )
{
	std::cout << "Program exiting.." << signum << std::endl;
	run = false;
}

/**
 *prevod uint8 do retezce v hex tvaru napr. 15 = "0f"
 */
std::string uint8_to_hex_string(const uint8_t v)
{
	std::stringstream ss;

	ss << std::hex << std::setfill('0');

	ss << std::hex << std::setw(2) << static_cast<int>(v);

	return ss.str();
}

int main(int argc, char *argv[])
{

	int rc;
	int mid = HEXIWEAR_ID;
	Bluetooth *bt_client;
	MQTT *mqtt_client;

	signal(SIGINT, signalHandler);

	bt_client = new Bluetooth();

	std::vector<std::pair<std::string, std::string>> hex_topics;
	//vytvoreni seznamu s pary topics a uuid
	hex_topics.push_back(std::make_pair(MQTT_HEXIWEAR_BAT_SERVICE_LEVEL, BT_UUID_HEXIWEAR_BAT_SERVICE_LEVEL));
	hex_topics.push_back(std::make_pair(MQTT_HEXIWEAR_APP_SERVICE_APPMODE, BT_UUID_HEXIWEAR_APP_SERVICE_APPMODE));
	hex_topics.push_back(std::make_pair(MQTT_HEXIWEAR_MOTION_SERVICE_ACC, BT_UUID_HEXIWEAR_MOTION_SERVICE_ACC));
	hex_topics.push_back(std::make_pair(MQTT_HEXIWEAR_MOTION_SERVICE_GYRO, BT_UUID_HEXIWEAR_MOTION_SERVICE_GYRO));
	hex_topics.push_back(std::make_pair(MQTT_HEXIWEAR_MOTION_SERVICE_MAG, BT_UUID_HEXIWEAR_MOTION_SERVICE_MAG));
	hex_topics.push_back(std::make_pair(MQTT_HEXIWEAR_WEATHER_SERVICE_AMB, BT_UUID_HEXIWEAR_WEATHER_SERVICE_AMB));
	hex_topics.push_back(std::make_pair(MQTT_HEXIWEAR_WEATHER_SERVICE_TEMP, BT_UUID_HEXIWEAR_WEATHER_SERVICE_TEMP));
	hex_topics.push_back(std::make_pair(MQTT_HEXIWEAR_WEATHER_SERVICE_HUMID, BT_UUID_HEXIWEAR_WEATHER_SERVICE_HUMID));
	hex_topics.push_back(std::make_pair(MQTT_HEXIWEAR_WEATHER_SERVICE_PRESS, BT_UUID_HEXIWEAR_WEATHER_SERVICE_PRESS));
	hex_topics.push_back(std::make_pair(MQTT_HEXIWEAR_HEALTH_SERVICE_HR, BT_UUID_HEXIWEAR_HEALTH_SERVICE_HR));
	hex_topics.push_back(std::make_pair(MQTT_HEXIWEAR_HEALTH_SERVICE_STEPS, BT_UUID_HEXIWEAR_HEALTH_SERVICE_STEPS));
	hex_topics.push_back(std::make_pair(MQTT_HEXIWEAR_HEALTH_SERVICE_CAL, BT_UUID_HEXIWEAR_HEALTH_SERVICE_CAL));
	hex_topics.push_back(std::make_pair(MQTT_HEXIWEAR_ALERT_SERVICE_IN, BT_UUID_HEXIWEAR_ALERT_SERVICE_IN));
	hex_topics.push_back(std::make_pair(MQTT_HEXIWEAR_ALERT_SERVICE_OUT, BT_UUID_HEXIWEAR_ALERT_SERVICE_OUT));
	// mqtt
	mosqpp::lib_init();

	bt_client->EstablishConnection(HEXIWEAR_MAC);
	mqtt_client = new MQTT(MQTT_CLIENT_ID, MQTT_BROKER_ADDRESS, MQTT_BROKER_PORT);

	std::vector<uint8_t> result;
	std::string str_result;
	//hlavni smycka

	while (true)
	{
		rc = mqtt_client->loop();

		for (auto i = 0; i < hex_topics.size(); i++)
		{
			//cteni dat z bt pomoci uuid
			bt_client->ReadCharByUUID(hex_topics[i].second, result);
			for (auto &res : result)
			{
				str_result.append(uint8_to_hex_string(res));
			}

			if (rc)
				mqtt_client->reconnect();
			else//publikace dat z bt
				mqtt_client->publish(&mid, hex_topics[i].first.c_str(), result.size() + 1, (void*)&str_result[0], 0, false);
			str_result.clear();

		}

		if (!run)
			break;
		//perioda
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}

	mosqpp::lib_cleanup();
	bt_client->CloseConnection();
	return 0;
}
