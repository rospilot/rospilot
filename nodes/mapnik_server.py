#!/usr/bin/env python

'''
Copyright 2012 the original author or authors.
See the NOTICE file distributed with this work for additional
information regarding copyright ownership.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
'''

import roslib
roslib.load_manifest('rospilot')
from os.path import expanduser
import os
import rospy
import cherrypy
import TileStache
from TileStache.Config import buildConfiguration
from catkin.find_in_workspaces import find_in_workspaces


# TODO: Maybe merge this with WebUI since Cherrypy can run multiple
# applications in one container?
class MapnikNode(object):
    def run(self):
        rospy.init_node('rospilot_mapnik_server')

        os.environ['PGUSER'] = 'rospilot'
        os.environ['PGPASSWORD'] = 'rospilot_password'
        os.environ['PGHOST'] = 'localhost'
        style_file = find_in_workspaces(['share'], 'rospilot',
                                        'share/mapnik-style/style.xml', first_match_only=True)
        if not style_file:
            rospy.logfatal("Cannot find share/mapnik-style/style.xml")
        else:
            style_file = style_file[0]

        config = expanduser(rospy.get_param('~tilestache_config_file'))
        config = buildConfiguration({
            "cache": {"name": "Test"},
            "layers": {
                "ex": {
                    "provider": {"name": "mapnik", "mapfile": "style.xml"},
                    "projection": "spherical mercator"
                }
            }
        }, style_file)
        app = TileStache.WSGITileServer(config=config)

        # Mount the application
        cherrypy.tree.graft(app, "/")

        # Unsubscribe the default server
        cherrypy.server.unsubscribe()

        # Instantiate a new server object
        server = cherrypy._cpserver.Server()

        # Configure the server object
        server.socket_host = "0.0.0.0"
        server.socket_port = int(rospy.get_param('/rospilot/mapnik_server_port'))
        server.thread_pool = 5

        # Subscribe this server
        server.subscribe()

        cherrypy.engine.start()
        rospy.loginfo("Mapnik server is running")
        rospy.loginfo(os.path.dirname(os.path.realpath(__file__)))
        rospy.loginfo(os.getcwd())
        rospy.spin()
        cherrypy.engine.exit()

if __name__ == '__main__':
    node = MapnikNode()
    node.run()
