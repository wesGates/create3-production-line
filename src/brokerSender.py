import paho.mqtt.client as mqtt
import json

# The callback for when the client receives a CONNACK response from the server.

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

# data = {

#     "messageType": "Report",
#     "node":"roomba",
#     "nodeId":"0000",
#     "productLine":"moscow",
#     "roombareport":{
#     "isDock":True,
#     "isReady":True,
#     "withDice":True,
#     "isAtBase1":True,
#     "isAtBase2":True,
#     "isAtBase3":True,
#     "ismoving":True,
#     "position":[ "<x>","<y>", "<z>" ],

#     "Fault":{
#     }

#  },

#  "date":"<date>T<time>"

# }
 

# ### Copy and pasted portion from the pub:
topic = "vandalrobot"

# message = {
#     "nodeId":117,
#     "messageType":"registration"
# }

# # Publish the message to the specified topic.
# mqttc.publish(topic, json.dumps(message))

# mqttc.publish(topic, json.dumps(data))

### End copy and pasted portion

# Blocking call that processes network traffic, dispatches callbacks and

# handles reconnecting.

# Other loop*() functions are available that give a threaded interface and a

# manual interface.

mqttc.loop_forever()

print("Got through!")