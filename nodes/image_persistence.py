#!/usr/bin/env python
"""
The `image_persistence.py` module listens for image data from an environment,
converts the images to PNG format and stores them in the CouchDB instance as
attachments to environmental data points. There should be example one instance
of this module for every environment in the system that has camera(s) connected
to it.

It assumes all topics of the type `sensor_msgs/Image` under the namespace
for the environment are streams of images from connected webcams.
"""
import time
import rospy
import requests

import paramiko
from scp import SCPClient

from shutil import copyfile
from PIL import Image
from couchdb import Server
from StringIO import StringIO
from re import match
from std_msgs.msg import String

from openag_lib.db_bootstrap.db_names import ENVIRONMENTAL_DATA_POINT
from openag_lib.config import config as cli_config
from openag_brain.models import EnvironmentalDataPoint
from openag_brain.load_env_var_types import create_variables
from openag_brain import params
from openag_brain.utils import read_environment_from_ns

PLANT_DATA_POINT = 'plant_data_point'
# Filter a list of environmental variables that are specific to camera
CAMERA_VARIABLES = create_variables(rospy.get_param('/var_types/camera_variables'))


DATABASE_SERVER_IP_PORT = 'http://foodcomputer-db.akg.t.u-tokyo.ac.jp:5984/'

scp_server = 'foodcomputer-db.akg.t.u-tokyo.ac.jp'
port = 22
user = 'iizuka'

class ImagePersistence:
    image_format_mapping = {
        "rgb8": "RGB",
        "rgba8": "RGBA"
    }

    def __init__(self, db, topic, variable, environment, min_update_interval,scp_server, port, user, img_database_path):
        self.db = db
        self.variable = variable
        self.environment = environment
        self.min_update_interval = min_update_interval
        self.last_update = 0
        self.sub = rospy.Subscriber(topic, String, self.on_image)
        self.scp_server = scp_server
        self.port = port
        self.user = user

        self.dist_path = '{}/{}'.format(img_database_path, variable)



    def on_image(self, file_path):
        # Rate limit
        curr_time = time.time()
        if (curr_time - self.last_update) < self.min_update_interval:
            return
        self.last_update = curr_time

        rospy.loginfo(file_path.data)
        filename = file_path.data.split('/')[-1]
        dist = '{}/{}'.format(self.dist_path, filename)
        src = file_path.data

        try:
            with paramiko.SSHClient() as ssh:
                ssh.load_system_host_keys()
                ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                ssh.connect(self.scp_server, self.port, self.user)
                self.scp_image(ssh, src, dist)
                point = {
                    "environment": self.environment,
                    "variable": self.variable,
                    "value": dist,
                    "timestamp": curr_time
                }
                point_id, point_rev = self.db.save(point)
                rospy.loginfo('saved in {}'.format(dist))

        except:
            print("fail to scp {}".format(filename))


    def scp_image(self, ssh, src, dist):
        scp = SCPClient(ssh.get_transport())
        scp.put(src, dist)
        scp.close()

def createSSHClient(scp_server, port, user):
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(scp_server, port, user)
    return client

if __name__ == '__main__':
    db_server = cli_config["local_server"]["url"]
    if not db_server:
        raise RuntimeError("No database server specified")
    server = Server(DATABASE_SERVER_IP_PORT)
    #server = Server(db_server)
    rospy.init_node('image_persistence_1')
    environment_id = read_environment_from_ns(rospy.get_namespace())
    try:
        min_update_interval = rospy.get_param("~min_update_interval")
    except KeyError:
        rospy.logwarn(
            "No minimum update interval specified for image persistence module"
        )
        min_update_interval = 3600
    plt_var_db = server[PLANT_DATA_POINT]
    persistence_objs = []
    img_database_path = rospy.get_param('~img_database_path', "/home/iizuka/ImageDatabase")
    for variable in CAMERA_VARIABLES.itervalues():
        topic = "{}/image_updated".format(variable)
        persistence_objs.append(ImagePersistence(
            db=plt_var_db,
            topic=topic,
            variable=variable,
            environment=environment_id,
            min_update_interval=min_update_interval,
            scp_server=scp_server,
            port=port,
            user=user,
            img_database_path=img_database_path
        ))
    rospy.spin()
