/**
 * mqtt.h
 * 
 * xkoupy00 - Pavel Koupy
 *
 */
#pragma once 
#include <iostream>
#include <mosquittopp.h>
#include <cstring>
#include <cstdio>

#define MAX_PAYLOAD 160
#define DEFAULT_KEEP_ALIVE 60

class MQTT : public mosqpp::mosquittopp
{
public:
    MQTT (const char *id, const char *host, int port);
    ~MQTT();

    void on_connect(int rc);
    void on_subscribe(int mid, int qos_count, const int *granted_qos);
};


