import json
import paho.mqtt.client as mqtt
import threading


### gatesroomba12


def on_connect(client, userdata, flags, reason_code):

    print(f"Connected with result code {reason_code}")
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("vandalrobot")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    payload= json.loads(msg.payload)
    if "requestMessage" in payload and  "brokerManager" in payload and payload['brokerManager']:
        if payload["requestMessage"] =='registration' :
            message = {
                "messageType": "registration",
                "node":"roomba",
                "nodeId":"gatesroomba12",
                "productLine":"moscow",
                }
            mqttc.publish(topic, json.dumps(message))
        # print("msg")

mqttc = mqtt.Client()
mqttc.on_connect = on_connect
mqttc.on_message = on_message

mqttc.connect("mqtt.eclipseprojects.io", 1883, 60)


# Copy and pasted portion from the pub:
topic = "vandalrobot"

# # Create and start the thread
# stop_event = threading.Event()
# server_thread  = threading.Thread(target=mqttc.loop_forever())
# server_thread.start()
