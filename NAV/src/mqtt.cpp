/**
 * mqtt.cpp
 * prekryti MQTT clienta mosquitopp
 * xkoupy00 - Pavel Koupy
 */
#include "mqtt.h"
/**
 * pripojeni  k brokerovi a nastaveni keepalive
 */
MQTT::MQTT(const char *id, const char *host, int port) : mosquittopp(id)
{
	int keepalive = DEFAULT_KEEP_ALIVE;
	connect(host, port, keepalive);
}
/**
 * // on_connect message
 */
void MQTT::on_connect(int rc)
{
	if (!rc)
	{
		std::cout << "Hexiwear connected..." << rc << std::endl;
	}
}
/**
 * on subscribe message
 */
void MQTT::on_subscribe(int mid, int qos_count, const int *granted_qos)
{
	std::cout << "Subscription succeeded." << std::endl;
}
/**
 * destructor
 */
MQTT::~MQTT()
{
}
