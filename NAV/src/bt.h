/**
 * bt.h
 * 
 * xkoupy00 - Pavel Koupy
 *
 */
#pragma once

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <vector>
#include "gattlib.h"

class Bluetooth
{
public:
	Bluetooth();
	~Bluetooth();
	int EstablishConnection(std::string hexiwear_mac);
	int Str2UUID(std::string uuid, uuid_t *c_uuid);
	int ReadCharByUUID(std::string str_uuid, std::vector<uint8_t> &data);
	int WriteCharByUUID(uint8_t *data, std::string str_uuid);
	void CloseConnection();


private :
	gatt_connection_t* gatt_con;
};

