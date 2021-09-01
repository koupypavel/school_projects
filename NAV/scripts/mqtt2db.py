# -*- coding: utf-8 -*-
"""
*
*  skript pro automaticke predavani zprav z topicu do mysql databaze
*  xkoupy00@stud.fit.vutbr.cz
*  17.05.2020
"""
import paho.mqtt.client as paho
import mysql.connector
import time

id_list = []
db_config = {
    'user': 'nav',
    'password': 'polda2541',
    'host': 'localhost',
    'database': 'hexiwear',
    'raise_on_warnings': True,
    'use_pure': False,
}

def on_message(client, userdata, msg):
    """Vola se pri prijeti zpravy"""
    cnx = mysql.connector.connect(**db_config)
    cursor = cnx.cursor()
    topic_list = msg.topic.split("/")
    new_data = ("insert into hexiwear.data (device_id, service_id, value_offset, value) values ('"+str('122334')+"','"+str(int(topic_list[1], 16))+"','"+str(0)+"','"+str(msg.payload)+"')")
    print(new_data);
    cursor.execute(new_data)
    cnx.commit();
    cursor.close()
    cnx.close()

# MQTT klient
client = paho.Client()
#client.username_pw_set('pi', 'canlab5586')
client.on_message = on_message
client.connect('localhost', 1883)

# MYSQL
cnx = mysql.connector.connect(**db_config)
cursor = cnx.cursor()
#do lisstu si vlozim id s nazvem senzoru
#id_list_query = ("select id as sensor_id, name from hexiwear.sensor")

#cursor.execute(id_list_query)

#for (sensor_id, name) in cursor:
 #   id_list.append((name, sensor_id))

#id_list = dict(id_list)
cursor.close()
cnx.close()

client.subscribe('hexiwear/2A19/battery_service/battery_level', qos=0)
client.subscribe('hexiwear/2041/appmode_service/app', qos=0)
client.subscribe('hexiwear/2001/motion_service/accelerometr', qos=0)
client.subscribe('hexiwear/2002/motion_service/gyroscope', qos=0)
client.subscribe('hexiwear/2003/motion_service/magnetometr', qos=0)
client.subscribe('hexiwear/2011/weather_service/ambient_light', qos=0)
client.subscribe('hexiwear/2012/weather_service/temperature', qos=0)
client.subscribe('hexiwear/2013/weather_service/humidity', qos=0)
client.subscribe('hexiwear/2014/weather_service/pressure', qos=0)
client.subscribe('hexiwear/2021/health_service/heart_rate', qos=0)
client.subscribe('hexiwear/2022/health_service/steps', qos=0)
client.subscribe('hexiwear/2023/health_service/calories', qos=0)
client.subscribe('hexiwear/2032/alert_service/alert_out', qos=0)


#nekonecny blokujici loop
client.loop_forever()
