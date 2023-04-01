""" ############################################################################################ """
""" ############################################################################################ """
""" WIFI """
""" V1_00, 2022-12-xx, are """

""" ############################################################################################ """
""" ############################################################################################ """

import paho.mqtt.client as mqtt

BROKER_ADDRESS = "localhost" # broker address, IP of the raspberry PI or just localhost
MQTT_PATH = "/test/topic" #this is the name of topic

""" The callback for when the client receives a CONNACK response from the server. """
def on_message(client, userdata, message):
    msg = str(message.payload.decode("utf-8"))
    print("message received: ", msg)
    print("message topic: ", message.topic)

""" The callback for when the client receives a CONNACK response from the server. """
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(MQTT_PATH) # Subscribing in on_connect() means that if we lose the connection and reconnect then subscriptions will be renewed.

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER_ADDRESS)
print("Connected to MQTT Broker: " + BROKER_ADDRESS)

client.loop_forever() # use this line if you don't want to write any further code. It blocks the code forever to check for data
#client.loop_start()  #use this line if you want to write any more code here




