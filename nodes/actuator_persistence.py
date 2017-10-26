#!/usr/bin/env python

from collections import OrderedDict
from std_msgs.msg import String
from openag_lib.config import config as cli_config
from couchdb import Server
from openag_brain.utils import gen_doc_id, read_environment_from_ns


import time
import rospy

ACTUATOR_DATA_POINT = 'actuator_data_point'
DATABASE_SERVER_IP_PORT = 'http://foodcomputer-db.akg.t.u-tokyo.ac.jp:5984/'


actuator_init_state =  OrderedDict([
    ("status", 0),
    ("pump_1_nutrient_a_1", 0.0),
    ("pump_2_nutrient_b_1", 0.0),
    ("pump_3_ph_up_1", False),
    ("pump_4_ph_down_1", False),
    ("pump_5_water_1", False),
    ("chiller_fan_1", False),
    ("chiller_pump_1", False),
    ("heater_core_2_1", False),
    ("air_flush_1", 0.0),
    ("water_aeration_pump_1", False),
    ("water_circulation_pump_1", False),
    ("chamber_fan_1", False),
    ("light_intensity_blue", 0.0),
    ("light_intensity_white", 0.0),
    ("light_intensity_red", 0.0),
    ("heater_core_1_1", False),
    ("chiller_compressor_1", False)
])

class ActuatorPersistence():
    def __init__(
        self, db, topic, topic_type, environment, actuator_state
    ):
        self.db = db
        self.environment = environment
        self.last_actutator_state= actuator_init_state
        self.last_time = time.time()
        self.sub = rospy.Subscriber(topic, topic_type, self.on_data)

    def on_data(self, data):
        message = data.data.strip('\n')
        actuator_values = message.split(',')
        curr_time = time.time()
        #i = 0
        for  i, actuator_name  in enumerate(self.last_actutator_state):
            if self.last_actutator_state[actuator_name] != actuator_values[i]:
                self.save_data(actuator_name, actuator_values[i], curr_time)
                self.last_actutator_state[actuator_name] = actuator_values[i]

    def save_data(self, actuator_name, actuator_value, curr_time):
       # rospy.loginfo('inside func save_data')
        rospy.loginfo('act_name:{},act_val:{}, curr_time:{}'.format(actuator_name,actuator_value, curr_time))

        point ={
            "environment": self.environment,
            "variable": actuator_name,
            "value": actuator_value,
            "timestamp": curr_time
        }
        point_id, point_rev = self.db.save(point)
        rospy.loginfo("data is saved")



if __name__ == "__main__":
    db_server = cli_config["local_server"]["url"]
    if not db_server:
        raise RuntimeError("No local database specified")
    server = Server(DATABASE_SERVER_IP_PORT)
    #server = Server(db_server)
    db = server['actuator_data_point']
    rospy.init_node('actuator_persistence')
    #environment_id = read_environment_from_ns(rospy.get_namespace())
    environment_id = 'environment_1'
    ActuatorPersistence(db, 'actuator_log', String, environment_id, actuator_init_state)

    rospy.spin()
