# -*- coding: utf-8 -*-
"""
* SEN -  ultrazvukovy sensor/meric stavu vody v nadrzi
*
* @brief skript pro automaticke predavani zprav z topicu do mysql databaze
* @author xkoupy00@stud.fit.vutbr.cz
* @date 6.11.2018
"""
import paho.mqtt.client as paho
import mysql.connector
import time

id_list = []
db_config = {
    'user': 'root',
    'password': 'pi',
    'host': 'localhost',
    'database': 'sen_iot',
    'raise_on_warnings': True,
    'use_pure': False,
}

def on_message(client, userdata, msg):
    """Vola se pri prijeti zpravy"""
    cnx = mysql.connector.connect(**db_config)
    cursor = cnx.cursor()
    topic_list = msg.topic.split("/")
    new_data = ("insert into sen_iot.data (sensor_id, quantity_type, quantity, value) values ('"+str(id_list[topic_list[0]])+"','"+topic_list[1]+"','"+topic_list[2]+"','"+str(msg.payload)+"')")
    cursor.execute(new_data)
    cnx.commit();
    cursor.close()
    cnx.close()

# MQTT klient
client = paho.Client()
client.username_pw_set('pi', 'pi')
client.on_message = on_message
client.connect('localhost', 1883)

# MYSQL
cnx = mysql.connector.connect(**db_config)
cursor = cnx.cursor()
#do lisstu si vlozim id s nazvem senzoru
id_list_query = ("select id as sensor_id, name from sen_iot.sensor")

cursor.execute(id_list_query)

for (sensor_id, name) in cursor:
    id_list.append((name, sensor_id))

id_list = dict(id_list)
cursor.close()
cnx.close()

client.subscribe('uwls/raw/dist', qos=1)
client.subscribe('uwls/raw/temp', qos=1)
client.subscribe('uwls/raw/humid', qos=1)
client.subscribe('uwls/calc/dewPoint', qos=1)
client.subscribe('uwls/calc/heatIndex', qos=1)
client.subscribe('uwls/calc/volume', qos=1)
client.subscribe('uwls/calc/tankLevel', qos=1)

#nekonecny blokujici loop
client.loop_forever()
