/**
 * bt.cpp
 * komunikace s bt modulem pomoci gattlib
 * xkoupy00 - Pavel Koupy
 *
 *  Copyright (C) 2016-2019  Olivier Martin <olivier@labapart.org>
 */
#include "bt.h"
/**
 * kostruktor
 */
Bluetooth::Bluetooth()
{

}
/**
 *vytvoreni spojeni
 */
int Bluetooth::EstablishConnection(std::string hexiwear_mac)
{
	int i, ret;
	size_t len;
	//Prevod vstupniho stringu na UUID
	gatt_con = gattlib_connect(NULL, hexiwear_mac.c_str(), GATTLIB_CONNECTION_OPTIONS_LEGACY_DEFAULT);

	if (gatt_con == nullptr)
	{
		std::cerr << "Fail to connect to the bluetooth device.\n";
		return 1;
	}
	return 0;
}
/**
 *  Copyright (C) 2016-2019  Olivier Martin <olivier@labapart.org>
 * zapis gattlib example/read_write.c
 */
int Bluetooth::WriteCharByUUID(uint8_t *data, std::string str_uuid)
{
	int ret = 0;
	uuid_t uuid;

	ret = gattlib_write_char_by_uuid(gatt_con, &uuid, &data, sizeof(data));
	if (ret != GATTLIB_SUCCESS)
	{
		char uuid_str[MAX_LEN_UUID_STR + 1];

		gattlib_uuid_to_string(&uuid, uuid_str, sizeof(uuid_str));

		if (ret == GATTLIB_NOT_FOUND) 
		{
			fprintf(stderr, "Could not find GATT Characteristic with UUID %s. "
			        "You might call the program with '--gatt-discovery'.\n", uuid_str);
		} 
		else 
		{
			fprintf(stderr, "Error while writing GATT Characteristic with UUID %s (ret:%d)\n",
			        uuid_str, ret);
		}
	}
	return 0;
}
/**
 *  Copyright (C) 2016-2019  Olivier Martin <olivier@labapart.org>
 * cteni gattlib example/read_write.c
 */
int Bluetooth::ReadCharByUUID(std::string str_uuid, std::vector<uint8_t>& data)
{
	uuid_t uuid;
	size_t len;
	int ret = 0;
	if (gattlib_string_to_uuid(str_uuid.c_str(), str_uuid.size(), &uuid) < 0)
	{
		return 1;
	}
	uint8_t *buffer = NULL;

	ret = gattlib_read_char_by_uuid(gatt_con, &uuid, (void **)&buffer, &len);
	if (ret != GATTLIB_SUCCESS) {
		char uuid_str[MAX_LEN_UUID_STR + 1];

		gattlib_uuid_to_string(&uuid, uuid_str, sizeof(uuid_str));

		if (ret == GATTLIB_NOT_FOUND) {
			fprintf(stderr, "Could not find GATT Characteristic with UUID %s. "
			        "You might call the program with '--gatt-discovery'.\n", uuid_str);
		} else {
			fprintf(stderr, "Error while reading GATT Characteristic with UUID %s (ret:%d)\n", uuid_str, ret);
		}
	}

	data.assign(&buffer[0], &buffer[len]);
	free(buffer);

	return ret;
}
/**
 * ukoceni spojeni
 */
void Bluetooth::CloseConnection()
{
	gattlib_disconnect(gatt_con);
}
/**
 *
 */
Bluetooth::~Bluetooth()
{
	CloseConnection();
}