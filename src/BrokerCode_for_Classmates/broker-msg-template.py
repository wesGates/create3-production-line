""" 
This code enables you to send reports to the broker from within a ROS2 node.
"""

# Required Python Packages
from datetime import datetime
import json

# Required Broker Packages
from brokerSender import mqttc

# Global vars
topic = "vandalrobot"


def reportSender(self, label,
            isAtBase1=False, isAtBase2=False, isAtBase3=False,
            isReady=False, isMoving=False, iswithDice=False
            ):
    """"
    Used to send a report at the beginning and end of every action.
    Defaults to False.
    """

    data = {
        "messageType": "Report",
        "node": "roomba",
        "nodeId": "your_node_ID",
        "productLine": "moscow",
        "roombareport": {
            "isDock": "", # Whether the robot is docked
            "label": label, # e.g. undock_start, undock_done, drive_start, drive_done
            "isReady": isReady,
            "withDice": iswithDice, # Whether the robot is in posession of a dice block
            "isAtBase1": isAtBase1, # Whether the robot is at Base 1
            "isAtBase2": isAtBase2,
            "isAtBase3": isAtBase3,
            "isMoving": isMoving, # Whether the robot is moving
            "position": ["<x_position>", "<y_position>", "<z_position>"], # Positions sent as integers in millimeters.

            "Fault": {
            }
        },
        "date": str(datetime.now())
    }
    
    mqttc.publish(topic, json.dumps(data))