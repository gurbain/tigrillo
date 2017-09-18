"""
This script allows the user to create SDF and MJCF robot files
that will be used to realize a simulation with Bullet, MuJoCo
or Gazebo. USE WITH PYTHON3 ONLY!!
"""


from copy import copy
from lxml import etree
import math
import numpy as np

__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "1.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "June 1st, 2017"


class Point(object):
    """
    Class representing a 3D point for geometric manipulations
    """

    def __init__(self, x, y, z, model_scale=1):

        self.x = x
        self.y = y
        self.z = z
        self.model_scale = model_scale

    def offset_by(self, x_offset, y_offset, z_offset):

        self.x = self.x + x_offset
        self.y = self.y + y_offset
        self.z = self.z + z_offset

    def distance_to(self, point):

        return np.sqrt((self.x - point.x)**2 + (self.y - point.y)**2 + (self.z - point.z)**2)

    def rotate_in_yz(self, angle):

        self.z, self.y = self.y*math.cos(angle) - self.z*math.sin(angle), self.y*math.sin(angle) + \
                         self.z*math.cos(angle)

    def get_rescaled_text(self):

        return "{0:.5f} {1:.5f} {2:.5f}".format(self.x / self.model_scale, self.y / self.model_scale, self.z /
                                                self.model_scale)


class Leg(object):
    """
    Create the XML for a single leg
    """

    def __init__(self, leg_id, config, tendons, actuators, sensors, model_scale=1):

        self.leg_id = leg_id
        self.config = config
        self.tendons = tendons
        self.actuators = actuators
        self.sensors = sensors
        self.model_scale = model_scale
        self.tibia_x_offset = 1.05*(-1)**(self.leg_id-1)

        self.femur_angle = 0
        self.femur_joint = None
        self.femur_tibia_joint = None
        self.femur_attachment = None
        self.tibia_attachment = None
        self.tibia_foot = None
        self.tibia_foot2 = None
        self.calc_morphology()

    def calc_tibia_spring_attachment(self):

        tibia_y = (self.config['femur_spring_tibia_joint_dst']**2 + self.config['spring_length']**2 -
                   self.config['tibia_spring_to_joint_dst']**2) / (2 * self.config['femur_spring_tibia_joint_dst'])
        tibia_z = -1.0 * math.sqrt(self.config['spring_length']**2 - tibia_y**2)

        tibia_point = Point(self.tibia_x_offset, tibia_y, tibia_z, self.model_scale)
        femur_point = Point(0, self.config['femur_length'] - self.config['femur_spring_tibia_joint_dst'],
                            0, self.model_scale)
        tibia_point.offset_by(0, self.config['femur_length'] - self.config['femur_spring_tibia_joint_dst'], 0)

        # Do rotation
        rotation_angle = self.femur_angle
        tibia_point.rotate_in_yz(rotation_angle)
        femur_point.rotate_in_yz(rotation_angle)

        return femur_point, tibia_point

    def calc_morphology(self):

        self.femur_angle = (self.config['femur_angle'] - 180) / 360 * 2 * math.pi

        self.femur_joint = Point(0, 0, 0, self.model_scale)
        self.femur_tibia_joint = Point(
            0,
            math.sin(self.femur_angle) * self.config['femur_length'],
            math.cos(self.femur_angle) * self.config['femur_length'], self.model_scale)
        self.femur_attachment, self.tibia_attachment = self.calc_tibia_spring_attachment()

        tibia_foot_y = self.femur_tibia_joint.y + \
                       ((self.femur_tibia_joint.y - self.tibia_attachment.y) /
                        self.config['tibia_spring_to_joint_dst'] *
                        (self.config['tibia_length']-self.config['tibia_spring_to_joint_dst']))
        tibia_foot_z = self.femur_tibia_joint.z + \
                       ((self.femur_tibia_joint.z - self.tibia_attachment.z) /
                        self.config['tibia_spring_to_joint_dst'] *
                        (self.config['tibia_length']-self.config['tibia_spring_to_joint_dst']))

        self.tibia_foot = Point(self.tibia_x_offset, tibia_foot_y, tibia_foot_z, self.model_scale)
        self.tibia_foot2 = Point(self.tibia_x_offset, tibia_foot_y-0.2, tibia_foot_z-0.4, self.model_scale)

    def offset_by(self, x, y, z):

        self.femur_joint.offset_by(x, y, z)
        self.femur_tibia_joint.offset_by(x, y, z)
        self.femur_attachment.offset_by(x, y, z)
        self.tibia_attachment.offset_by(x, y, z)
        self.tibia_foot.offset_by(x, y, z)
        self.tibia_foot2.offset_by(x, y, z)

    def generate_xml_leg(self, x, y, z):

        self.offset_by(x, y, z)

        leg = etree.Element('body')
        femur_geom = etree.Element('geom', type='capsule',
                                   fromto=self.femur_joint.get_rescaled_text() + ' ' +
                                          self.femur_tibia_joint.get_rescaled_text())
        femur_joint = etree.Element('joint', pos=self.femur_joint.get_rescaled_text(),
                                    name='shoulder_' + str(self.leg_id), damping=str(self.config['hip_damping']))
        femur_site = etree.Element('site', name='s'+str(self.leg_id)+'_1',
                                   pos=self.femur_attachment.get_rescaled_text())

        tibia = etree.Element('body')
        tibia_geom = etree.Element('geom', type='capsule',
                                   fromto=self.tibia_attachment.get_rescaled_text() +
                                          ' ' + self.tibia_foot.get_rescaled_text())
        tibia_joint = etree.Element('joint', limited='true', range='-60 0',
                                    pos=self.femur_tibia_joint.get_rescaled_text(),
                                    damping=str(self.config['knee_damping']))
        tibia_site = etree.Element('site', name='s'+str(self.leg_id)+'_2',
                                   pos=self.tibia_attachment.get_rescaled_text())

        foot = etree.Element('body', pos=self.tibia_foot.get_rescaled_text())
        foot_geom = etree.Element('geom', friction='1 0.005 0.0001', type='capsule',
                                  fromto=self.tibia_foot.get_rescaled_text() + ' ' +
                                         self.tibia_foot2.get_rescaled_text())
        foot_site = etree.Element('site', name='sensor_'+str(self.leg_id), pos=self.tibia_foot.get_rescaled_text())

        foot.extend([foot_geom, foot_site])
        tibia.extend([tibia_geom, tibia_joint, tibia_site, foot])
        leg.extend([femur_geom, femur_joint, femur_site, tibia])

        tendon_min = self.femur_attachment.distance_to(self.tibia_attachment) / 10
        tendon_max = tendon_min*10

        tendon = etree.Element('tendon')
        spatial = etree.Element('spatial', stiffness=str(self.config['spring_stiffness']),
                                range="{0:.5f} {1:.5f}".format(tendon_min, tendon_max))
        site1 = etree.Element('site', site='s'+str(self.leg_id)+'_1')
        site2 = etree.Element('site', site='s'+str(self.leg_id)+'_2')

        spatial.extend([site1, site2])
        tendon.append(spatial)
        self.tendons.append(tendon)

        self.actuators.append(etree.Element('position', forcelimited="true", forcerange="-50 50",
                                            joint="shoulder_"+str(self.leg_id), gear="1",
                                            kp=str(self.config['actuator_kp'])))
        self.sensors.append(etree.Element('force', site='sensor_'+str(self.leg_id)))

        return leg

    def get_height(self):

        return -self.tibia_foot.z


class MotorLeg(Leg):
    """
    Create the XML for a single leg and add the motors
    """

    def __init__(self, leg_id, config, tendons, actuators, sensors, model_scale=1):

        self.leg_id = leg_id
        self.config = config
        self.tendons = tendons
        self.actuators = actuators
        self.sensors = sensors
        self.model_scale = model_scale
        self.model_scale = model_scale
        super(MotorLeg, self).__init__(leg_id, config, tendons, actuators, sensors, model_scale)

    def generate_xml(self, torso_width, y, z):

        # x and y are the center of the top plane of motor

        width, length, height = self.config['motor']['width'], \
                                self.config['motor']['length'], self.config['motor']['height']

        # leg_height = self.get_height_with_motor()
        leg_xml = self.generate_xml_leg((-1)**((self.leg_id-1) % 2) *
                                        (torso_width + 0.51), y, z - self.config['motor']['leg_attachment_height'])

        motor = etree.Element('body')
        motor_geom = etree.Element('geom', type="box", mass="0.067",
                                   size="{0:.5f} {1:.5f} {2:.5f}".format(width/2/self.model_scale,
                                                                         length/2/self.model_scale,
                                                                         height/2/self.model_scale),
                                   pos="{0:.5f} {1:.5f} {2:.5f}".format((-1)**((self.leg_id-1)%2) *
                                                                        (torso_width - width/2) / self.model_scale, y /
                                                                        self.model_scale, (z-height/2) /
                                                                        self.model_scale))

        motor.extend([motor_geom, leg_xml])
        return motor

    def get_height_with_motor(self):

        return self.get_height() + self.config['motor']['height'] / 2


class MJCFileGenerator(object):
    """
    Generate a MJFC XML model file. Deprecated!
    """

    def __init__(self, model_config, filename='model.xml', model_scale=1, VERSION=2):

        # Options
        self.model_config = model_config
        self.filename = filename
        self.model_scale = model_scale
        self.VERSION = VERSION

        # Element lists
        self.tendons = []
        self.actuators = []
        self.sensors = []

        # XML anchors
        self.xml_root = None
        self.xml_default = None
        self.xml_compiler = None
        self.xml_option = None
        self.xml_assets = None
        self.xml_body = None
        self.xml_actuators = None
        self.xml_sensors = None

    def generate_xml_header(self):
        """ Generate the XML for all the header fields """

        self.xml_default = etree.Element('default')
        self.xml_root = etree.Element('mujoco', model='quadruped')

        geom = etree.Element('geom', rgba=".9 .7 .1 1", size="0.05", density="1")
        site = etree.Element('site', type="sphere", rgba=".9 .9 .9 1", size="0.005")
        joint = etree.Element('joint', type="hinge", axis="1 0 0", limited="true",
                              range="-100 100", solimplimit="0.95 0.95 0.1")
        tendon = etree.Element('tendon', width="0.02", rgba=".95 .3 .3 1", limited="true",
                               range="0.25 1",  stiffness="4000")
        compiler_settings = {'inertiafromgeom': 'true', 'angle': 'degree', 'coordinate': "global"}

        self.xml_compiler = etree.Element('compiler', **compiler_settings)
        self.xml_option = etree.Element('option', gravity='0 0 -98.1')

        self.xml_default.extend([geom, site, joint, tendon])

    def generate_xml_assets(self):
        """ Generate the XML for all the assets like surface and materials """

        self.xml_assets = etree.Element('asset')

        skybox = etree.Element('texture', type="skybox", builtin="gradient", width="128", height="128",
                               rgb1=".4 .6 .8", rgb2="0 0 0")
        texgeom = etree.Element('texture', name="texgeom", type="cube", builtin="flat", mark="cross", width="127",
                                height="1278", rgb1="0.8 0.6 0.4", rgb2="0.8 0.6 0.4", markrgb="1 1 1", random="0.01")
        texplane = etree.Element('texture', name="texplane", type="2d", builtin="checker", rgb1=".2 .3 .4",
                                 rgb2=".1 0.15 0.2", width="512", height="512")
        matplane = etree.Element('material', name='MatPlane', reflectance='0', texture="texplane",
                                 texrepeat="1 1", texuniform="true")
        geom_material = etree.Element('material', name='geom', texture="texgeom", texuniform="true")

        self.xml_assets.extend([skybox, texgeom, texplane, matplane, geom_material])

    def get_box_geom(self, x, y, z, width, length, height, mass, name=None):
        """ Return the geometry element for a box """

        s = self.model_scale
        el = etree.Element('geom',
                           type="box",
                           mass="{0:.5f}".format(mass),
                           size="{0:.5f} {1:.5f} {2:.5f}".format(width/2/s, length/2/s, height/2/s),
                           pos="{0:.5f} {1:.5f} {2:.5f}".format(x/s, y/s, z/s))

        if name:
            el.set('name', name)

        return el

    def generate_body(self):
        """ Generate the XML for the body """

        leg1 = MotorLeg(1, self.model_config['legs']['FL'], self.tendons,
                        self.actuators, self.sensors, self.model_scale)
        leg2 = MotorLeg(2, self.model_config['legs']['FR'], self.tendons,
                        self.actuators, self.sensors, self.model_scale)
        leg3 = MotorLeg(3, self.model_config['legs']['BL'], self.tendons,
                        self.actuators, self.sensors, self.model_scale)
        leg4 = MotorLeg(4, self.model_config['legs']['BR'], self.tendons,
                        self.actuators, self.sensors, self.model_scale)

        front_conf = model_config['body']['front']
        middle_conf = model_config['body']['middle']
        hind_conf = model_config['body']['hind']

        front_center_y = -(front_conf['length']/2 + middle_conf['length']/2)
        hind_center_y = hind_conf['length']/2 + middle_conf['length']/2

        leg_height = max(leg1.get_height_with_motor(), leg2.get_height_with_motor(),
                         leg3.get_height_with_motor(), leg4.get_height_with_motor())

        self.xml_body = etree.Element('worldbody')
        floor = etree.Element('geom', name='floor', pos='0 0 0', size='50 50 .125',
                              type='plane', material="MatPlane", condim='3')

        torso = etree.Element('body', name='torso')
        torso_geom_f = self.get_box_geom(0, front_center_y, leg_height+front_conf['height']/2, front_conf['width'],
                                         front_conf['length'], front_conf['height'], front_conf['mass'])
        torso_geom_m = self.get_box_geom(0, 0, leg_height+middle_conf['height']/2, middle_conf['width'],
                                         middle_conf['length'], middle_conf['height'], middle_conf['mass'],
                                         name="torso_geom")
        torso_geom_h = self.get_box_geom(0, hind_center_y, leg_height+hind_conf['height']/2, hind_conf['width'],
                                         hind_conf['length'], hind_conf['height'], hind_conf['mass'])

        torso_joint = etree.Element('joint', type='free', limited='false', name='gravity')
        torso_sensor_site = etree.Element('site', name='sensor_torso',
                                          pos="0 0 {0:.5f}".format(leg_height / self.model_scale))

        torso.extend([torso_geom_f, torso_geom_m, torso_geom_h, torso_joint, torso_sensor_site])

        torso.append(leg1.generate_xml(front_conf['width'] / 2, front_center_y, leg_height))
        torso.append(leg2.generate_xml(front_conf['width'] / 2, front_center_y, leg_height))
        torso.append(leg3.generate_xml(hind_conf['width'] / 2, hind_center_y, leg_height))
        torso.append(leg4.generate_xml(hind_conf['width'] / 2, hind_center_y, leg_height))

        battery_weight = model_config.get('battery_weight', 0)

        if battery_weight > 0:
            battery = etree.Element('body', name='battery_body')
            # battery_geom = etree.Element('geom', type="box", size=".17 .37 .105",
            # mass="{0:.5f}".format(battery_weight), pos="0 {0:.5f} {1:.5f}".format(-(length/2)/model_scale +
            # .37, leg_height/model_scale - .105), name='battery_geom')
            battery_geom = self.get_box_geom(0, -(middle_conf['length']/2 +
                                                  front_conf['length'] - 3.7), leg_height - 1.05, 1.7, 3.7, 1.05,
                                             battery_weight)
            battery.append(battery_geom)
            torso.append(battery)

        self.xml_body.extend([floor, torso])

    def generate_actuators(self):
        """ Generate the xml for the actuators """

        self.xml_actuators = etree.Element('actuator')
        self.xml_actuators.extend(self.actuators)

    def generate_sensors(self):
        """ Generate the XML for the sensors """

        self.xml_sensors = etree.Element('sensor')
        self.sensors.append(etree.Element('accelerometer', site='sensor_torso'))

        self.xml_sensors.extend(self.sensors)

    def generate(self, config=None):
        """ Main function called to generate a model """

        # Eventually update model configuration
        self.tendons = []
        self.actuators = []
        self.sensors = []
        if config is not None:
            for key, value in config.items():
                config[key] = value

        # Generate all elements
        self.generate_xml_header()
        self.generate_xml_assets()
        self.generate_body()
        self.generate_actuators()
        self.generate_sensors()

        # Construct the elements tree
        self.xml_root.append(self.xml_compiler)
        self.xml_root.append(self.xml_option)
        self.xml_root.append(self.xml_default)
        self.xml_root.append(self.xml_assets)
        self.xml_root.append(self.xml_body)
        for tendon in self.tendons:
            self.xml_root.append(tendon)
        self.xml_root.append(self.xml_actuators)
        self.xml_root.append(self.xml_sensors)

        # Create the XML file
        f = open(self.filename, 'w')
        f.write(etree.tostring(self.xml_root, pretty_print=True).decode('utf-8'))
        f.flush()
        f.close()

    def get_model_generator_version(self):

        return self.VERSION


class SDFileGenerator(object):
    """
    Generate a SDF XML model file
    """

    def __init__(self, model_config, filename='model.sdf', gazebo=False, model_scale=1, VERSION=2):

        # Options
        self.model_config = model_config
        self.filename = filename
        self.model_scale = model_scale
        self.VERSION = VERSION
        self.gazebo = gazebo

        # XML anchors
        self.xml_world = None
        self.xml_sdf = None
        self.xml_model = None
        self.head_array = []
        self.model_configs_array = []
        self.links_array = []
        self.joints_array = []
        self.plugins_array = []

        self.red = "1 0 0 1"
        self.green = "0 1 0 1"
        self.blue = "0 0 1 1"
        self.white = "1 1 1 1"
        self.black = "0 0 0 0"

        # Parameters
        self.z_offset = self.model_config["default_height"] / self.model_scale
        self.density = self.model_config["default_density"]
        self.front_y = (self.model_config["body"]["middle"]["length"] +
                        self.model_config["body"]["front"]["length"]) / 2 / self.model_scale
        self.back_y = - (self.model_config["body"]["middle"]["length"] +
                         self.model_config["body"]["hind"]["length"]) / 2 / self.model_scale
        self.front_x = self.model_config["body"]["front"]["width"] / 2 / self.model_scale
        self.back_x = self.model_config["body"]["hind"]["width"] / 2 / self.model_scale
        self.back_knee_y = 0
        self.back_knee_z = 0
        self.front_knee_y = 0
        self.front_knee_z = 0
        self.knee_angle = 0

    def generate_xml_header(self):
        """ Generate the XML for all the header fields """

        # Nothing here yet

    def generate_xml_model_configs(self):
        """ Generate the XML for the model config """

        xml_static = etree.Element('static')
        xml_static.text = "False"

        xml_collide = etree.Element('self_collide')
        xml_collide.text = "False"

        self.model_configs_array.extend([xml_static, xml_collide])

    def _create_body(self):
        """ Generate the XML for the body link """

        body = []

        for part in ["front", "middle", "hind"]:

            name = "body_" + part

            w = self.model_config["body"][part]["width"] / self.model_scale
            h = self.model_config["body"][part]["height"] / self.model_scale
            l = self.model_config["body"][part]["length"] / self.model_scale
            # vol = w * l * h
            m = self.model_config["body"][part]["mass"] / (self.model_scale * self.model_scale * self.model_scale)
            ih = m / 12 * (l**2 + w**2)  # z
            iw = m / 12 * (h**2 + l**2)  # x
            il = m / 12 * (h**2 + w**2)  # y

            elem = etree.Element('link', name=name)
            col = etree.Element('collision', name=name + "col")
            vis = etree.Element('visual', name=name + "vis")
            pose_elem = etree.Element('pose')
            inertial = etree.Element('inertial')
            mass = etree.Element('mass')
            inertia = etree.Element('inertia')
            ixx = etree.Element('ixx')
            ixy = etree.Element('ixy')
            ixz = etree.Element('ixz')
            iyy = etree.Element('iyy')
            iyz = etree.Element('iyz')
            izz = etree.Element('izz')

            geometry = etree.Element('geometry')
            box = etree.Element('box')
            size = etree.Element('size')

            material = etree.Element('material')
            ambient = etree.Element('ambient')
            diffuse = etree.Element('diffuse')
            specular = etree.Element('specular')
            emissive = etree.Element('emissive')

            size.text = str(w) + " " + str(l) + " " + str(h)
            mass.text = str(m)
            ixx.text = str(iw)
            ixy.text = str(0)
            ixz.text = str(0)
            iyy.text = str(il)
            iyz.text = str(0)
            izz.text = str(ih)

            if part == "middle":
                ambient.text = self.blue
                diffuse.text = self.blue
                specular.text = self.blue
            else:
                ambient.text = self.red
                diffuse.text = self.red
                specular.text = self.red
            emissive.text = self.black

            if part == "middle":
                pose_elem.text = "0 0 " + str(h / 2 + self.z_offset) + " 0 0 0"
            elif part == "front":
                pose_elem.text = "0 " + str(self.front_y) + " " + str(h / 2 + self.z_offset) + " 0 0 0"
            else:
                pose_elem.text = "0 " + str(self.back_y) + " " + str(h / 2 + self.z_offset) + " 0 0 0"

            inertia.extend([ixx, ixy, ixz, iyy, iyz, izz])
            inertial.extend([mass, inertia])
            box.extend([size])
            geometry.extend([box])
            material.extend([ambient, diffuse, specular, emissive])
            vis.extend([geometry, material])
            col.extend([copy(geometry)])
            elem.extend([pose_elem, inertial, col, vis])

            # Add a fixed joint
            if part != "middle":
                joint = etree.Element("joint", name=name + "_J", type="fixed")
                j_parent = etree.Element("parent")
                j_child = etree.Element("child")
                j_parent.text = "body_middle"
                j_child.text = name
                joint.extend([j_parent, j_child])
                self.joints_array.append(joint)

            body.extend([elem])

        return body

    def _create_leg_motor(self, name, leg_id, config):
        """ Generate the XML for a the down leg link """

        name = name + "M"

        motor_w = config["motor"]["width"] / self.model_scale   # x
        motor_l = config["motor"]["length"] / self.model_scale  # y
        motor_h = config["motor"]["height"] / self.model_scale  # z
        m = config["motor"]["mass"] / (self.model_scale * self.model_scale * self.model_scale)
        # vol = motor_w * motor_l * motor_h

        if leg_id == 1 or leg_id == 2:  # Front
            y = self.front_y
            x = (-1)**(leg_id % 2) * (self.front_x - motor_w / 2)
        else:  # Back
            y = self.back_y
            x = (-1)**(leg_id % 2) * (self.back_x - motor_w / 2)
        z = - motor_h / 2 + self.z_offset

        iw = m/12 * (motor_h**2 + motor_l**2)  # x
        il = m/12 * (motor_h**2 + motor_w**2)  # y
        ih = m/12 * (motor_l**2 + motor_w**2)  # z

        motor = etree.Element('link', name=name)
        pose = etree.Element('pose')
        inertial = etree.Element('inertial')
        mass = etree.Element('mass')
        inertia = etree.Element('inertia')
        ixx = etree.Element('ixx')
        ixy = etree.Element('ixy')
        ixz = etree.Element('ixz')
        iyy = etree.Element('iyy')
        iyz = etree.Element('iyz')
        izz = etree.Element('izz')
        collision = etree.Element('collision', name=name + "_col")
        geometry = etree.Element('geometry')
        box = etree.Element('box')
        size = etree.Element('size')
        visual = etree.Element('visual', name=name + "_vis")
        material = etree.Element('material')
        ambient = etree.Element('ambient')
        diffuse = etree.Element('diffuse')
        specular = etree.Element('specular')
        emissive = etree.Element('emissive')

        pose.text = str(x) + " " + str(y) + " " + str(z) + " 0 0 0"
        size.text = str(motor_w) + " " + str(motor_l) + " " + str(motor_h)
        mass.text = str(m)
        ixx.text = str(iw)
        ixy.text = str(0)
        ixz.text = str(0)
        iyy.text = str(il)
        iyz.text = str(0)
        izz.text = str(ih)
        ambient.text = self.green
        diffuse.text = self.green
        specular.text = self.green
        emissive.text = self.black

        inertia.extend([ixx, ixy, ixz, iyy, iyz, izz])
        inertial.extend([mass, inertia])
        box.extend([size])
        geometry.extend([box])
        material.extend([ambient, diffuse, specular, emissive])
        visual.extend([geometry, material])
        collision.extend([copy(geometry)])
        motor.extend([pose, inertial, collision, visual])

        # Joint
        joint_m = etree.Element("joint", name=name + "J", type="fixed")
        jm_parent = etree.Element("parent")
        jm_child = etree.Element("child")
        if leg_id == 1 or leg_id == 2:  # Front
            jm_parent.text = "body_front"
        else:  # Back
            jm_parent.text = "body_hind"
        jm_child.text = name
        joint_m.extend([jm_parent, jm_child])
        self.joints_array.append(joint_m)

        return motor

    def _create_leg_up(self, name, leg_id, config):
        """ Generate the XML for an upper leg link """

        name_m = name + "M"
        name = name + "U"

        h_attach = config["motor"]["leg_attachment_height"] / self.model_scale  # z
        l = config["femur_length"] / self.model_scale
        r = config["radius"] / self.model_scale

        slack = config["joint_slack"] / self.model_scale
        a = config['femur_angle'] / 360 * 2 * math.pi
        vol = l * math.pi * r**2

        # Attachment point 1
        z_1 = - h_attach + self.z_offset
        if leg_id == 1 or leg_id == 2:  # Front
            y_1 = self.front_y
            x_1 = (-1)**(leg_id % 2) * (self.front_x + r + slack)
        else:  # Back
            y_1 = self.back_y
            x_1 = (-1)**(leg_id % 2) * (self.back_x + r + slack)

        # Center point
        # x = x_1
        # y = y_1 + l * math.sin(a) / 2
        # z = z_1 - l * math.cos(a) / 2

        # Attachment point 2
        if leg_id == 1 or leg_id == 2:  # Front
            self.front_knee_y = y_1 + l * math.sin(a)
            self.front_knee_z = z_1 - l * math.cos(a)
        else:  # Back
            self.back_knee_y = y_1 + l * math.sin(a)
            self.back_knee_z = z_1 - l * math.cos(a)

        m = self.density * vol
        il = m / 2 * r**2
        ir = m / 12 * (3 * r**2 + l**2)
        damp = config['hip_damping']

        femur = etree.Element('link', name=name)
        pose = etree.Element('pose')
        pose_vis_col_in = etree.Element('pose')
        inertial = etree.Element('inertial')
        mass = etree.Element('mass')
        inertia = etree.Element('inertia')
        ixx = etree.Element('ixx')
        ixy = etree.Element('ixy')
        ixz = etree.Element('ixz')
        iyy = etree.Element('iyy')
        iyz = etree.Element('iyz')
        izz = etree.Element('izz')
        collision = etree.Element('collision', name=name + "_col")
        geometry = etree.Element('geometry')
        if self.gazebo:
            cylinder = etree.Element('cylinder')
            radius = etree.Element('radius')
            length = etree.Element('length')
        else:
            cylinder = etree.Element('cylinder', radius=str(r), length=str(l))
        visual = etree.Element('visual', name=name + "_vis")
        material = etree.Element('material')
        ambient = etree.Element('ambient')
        diffuse = etree.Element('diffuse')
        specular = etree.Element('specular')
        emissive = etree.Element('emissive')

        pose.text = str(x_1) + " " + str(y_1) + " " + str(z_1) + " " + str(a) + " 0 0"
        pose_vis_col_in.text = "0 0 " + str(-l / 2) + " 0 0 0"
        mass.text = str(m)
        ixx.text = str(ir)
        ixy.text = str(0)
        ixz.text = str(0)
        iyy.text = str(ir)
        iyz.text = str(0)
        izz.text = str(il)
        ambient.text = self.blue
        diffuse.text = self.blue
        specular.text = self.blue
        emissive.text = self.black
        if self.gazebo:
            radius.text = str(r)
            length.text = str(l)
            cylinder.extend([radius, length])

        inertia.extend([ixx, ixy, ixz, iyy, iyz, izz])
        inertial.extend([copy(pose_vis_col_in), mass, inertia])
        geometry.extend([cylinder])
        material.extend([ambient, diffuse, specular, emissive])
        visual.extend([copy(pose_vis_col_in), geometry, material])
        collision.extend([copy(pose_vis_col_in), copy(geometry)])
        femur.extend([pose, inertial, collision, visual])

        # Joint
        joint = etree.Element("joint", name=name + "J", type="revolute")
        j_parent = etree.Element("parent")
        j_child = etree.Element("child")
        j_axis = etree.Element("axis")
        j_xyz = etree.Element("xyz")
        j_dynamics = etree.Element("dynamics")
        j_damping = etree.Element("damping")
        j_limit = etree.Element("limit")
        j_lower = etree.Element("lower")
        j_upper = etree.Element("upper")

        j_parent.text = name_m
        j_child.text = name
        j_xyz.text = "1 0 0"
        j_damping.text = str(damp)
        j_lower.text = str(-math.pi / 2 - a)
        j_upper.text = str(math.pi / 2 - a)

        j_dynamics.extend([j_damping])
        j_limit.extend([j_lower, j_upper])
        j_axis.extend([j_xyz, j_dynamics])
        joint.extend([j_parent, j_child, j_axis])
        self.joints_array.append(joint)

        return femur

    def _create_leg_down(self, name, leg_id, config):
        """ Generate the XML for a down leg link """

        name_u = name + "U"
        name = name + "D"

        # Tibia values
        e = config["spring_length"] / self.model_scale
        f = config["femur_spring_tibia_joint_dst"] / self.model_scale
        g = config["tibia_spring_to_joint_dst"] / self.model_scale
        a_femur = config['femur_angle'] / 360 * 2 * math.pi
        l = config["tibia_length"] / self.model_scale
        r = config["radius"] / self.model_scale
        slack = config["joint_slack"] / self.model_scale
        compression_tol = config["spring_comp_tol"]
        a_radius = math.acos((f**2 + g**2 - e**2) / (2 * f * g))
        a = a_femur + a_radius
        self.knee_angle = a
        k_spring = config["spring_stiffness"]
        vol = l * math.pi * r**2
        m = self.density * vol
        il = m / 2 * r**2
        ir = m / 12 * (3 * r**2 + l**2)
        damp = config['hip_damping']

        # Foot values
        foot_r = config["foot"]["radius"] / self.model_scale
        # foot_vol = 4 * math.pi * r**3 / 3
        # foot_m = self.density * vol
        mu1 = config["foot"]["mu1"]
        mu2 = config["foot"]["mu2"]

        # Attachment point
        if leg_id == 1 or leg_id == 2:  # Front
            x_1 = (-1)**(leg_id % 2) * (self.front_x + r + slack)
            y_1 = self.front_knee_y
            z_1 = self.front_knee_z
        else:  # Back
            x_1 = (-1)**(leg_id % 2) * (self.back_x + r + slack)
            y_1 = self.back_knee_y
            z_1 = self.back_knee_z

        # Center point
        # x = x_1
        # y = y_1 + (l / 2 - g) * math.sin(a)
        # z = z_1 - (l / 2 - g) * math.cos(a)

        leg_down = etree.Element('link', name=name)
        pose = etree.Element('pose')
        tibia_pose = etree.Element('pose')
        tibia_inertial = etree.Element('inertial')
        tibia_mass = etree.Element('mass')
        tibia_inertia = etree.Element('inertia')
        tibia_ixx = etree.Element('ixx')
        tibia_ixy = etree.Element('ixy')
        tibia_ixz = etree.Element('ixz')
        tibia_iyy = etree.Element('iyy')
        tibia_iyz = etree.Element('iyz')
        tibia_izz = etree.Element('izz')
        tibia_collision = etree.Element('collision', name=name + "T_col")
        tibia_geometry = etree.Element('geometry')
        if self.gazebo:
            tibia_cylinder = etree.Element('cylinder')
            tibia_radius = etree.Element('radius')
            tibia_length = etree.Element('length')
        else:
            tibia_cylinder = etree.Element('cylinder', radius=str(r), length=str(l))

        tibia_visual = etree.Element('visual', name=name + "T_vis")
        tibia_material = etree.Element('material')
        tibia_ambient = etree.Element('ambient')
        tibia_diffuse = etree.Element('diffuse')
        tibia_specular = etree.Element('specular')
        tibia_emissive = etree.Element('emissive')

        pose.text = str(x_1) + " " + str(y_1) + " " + str(z_1) + " " + str(a) + " 0 0"
        tibia_pose.text = "0 0 " + str(l / 2 - g) + " 0 0 0"
        tibia_mass.text = str(m)
        tibia_ixx.text = str(ir)
        tibia_ixy.text = str(0)
        tibia_ixz.text = str(0)
        tibia_iyy.text = str(ir)
        tibia_iyz.text = str(0)
        tibia_izz.text = str(il)
        tibia_ambient.text = self.green
        tibia_diffuse.text = self.green
        tibia_specular.text = self.green
        tibia_emissive.text = self.black
        if self.gazebo:
            tibia_radius.text = str(r)
            tibia_length.text = str(l)
            tibia_cylinder.extend([tibia_radius, tibia_length])

        tibia_inertia.extend([tibia_ixx, tibia_ixy, tibia_ixz, tibia_iyy, tibia_iyz, tibia_izz])
        tibia_inertial.extend([copy(tibia_pose), tibia_mass, tibia_inertia])
        tibia_geometry.extend([tibia_cylinder])
        tibia_material.extend([tibia_ambient, tibia_diffuse, tibia_specular, tibia_emissive])
        tibia_visual.extend([copy(tibia_pose), tibia_geometry, tibia_material])
        tibia_collision.extend([copy(tibia_pose), copy(tibia_geometry)])

        # Foot
        foot_pose = etree.Element('pose')
        foot_collision = etree.Element('collision', name=name + "F_col")
        foot_geometry = etree.Element('geometry')
        if self.gazebo:
            foot_sphere = etree.Element('sphere')
            foot_radius = etree.Element('radius')
        else:
            foot_sphere = etree.Element('cylinder', radius=str(foot_r))
        foot_surface = etree.Element("surface")
        foot_friction = etree.Element("friction")
        foot_ode = etree.Element("ode")
        foot_mu1 = etree.Element("mu")
        foot_mu2 = etree.Element("mu2")

        foot_visual = etree.Element('visual', name=name + "F_vis")
        foot_material = etree.Element('material')
        foot_ambient = etree.Element('ambient')
        foot_diffuse = etree.Element('diffuse')
        foot_specular = etree.Element('specular')
        foot_emissive = etree.Element('emissive')

        foot_pose.text = " 0 0 " + str((-g)) + " 0 0 0"
        foot_mu1.text = str(mu1)
        foot_mu2.text = str(mu2)
        foot_ambient.text = self.green
        foot_diffuse.text = self.green
        foot_specular.text = self.green
        foot_emissive.text = self.black
        if self.gazebo:
            foot_radius.text = str(foot_r)
            foot_sphere.extend([foot_radius])

        foot_ode.extend([foot_mu1, foot_mu2])
        foot_friction.extend([foot_ode])
        foot_surface.extend([foot_friction])
        foot_geometry.extend([foot_sphere])
        foot_material.extend([foot_ambient, foot_diffuse, foot_specular, foot_emissive])
        foot_visual.extend([copy(foot_pose), foot_geometry, foot_material])
        foot_collision.extend([copy(foot_pose), copy(foot_geometry), copy(foot_surface)])

        leg_down.extend([pose, tibia_inertial, tibia_collision, tibia_visual, foot_collision, foot_visual])

        # Joint
        joint = etree.Element("joint", name=name + "J", type="revolute")
        j_parent = etree.Element("parent")
        j_child = etree.Element("child")
        j_axis = etree.Element("axis")
        j_xyz = etree.Element("xyz")
        j_dynamics = etree.Element("dynamics")
        j_damping = etree.Element("damping")
        j_spring_stiffness = etree.Element("spring_stiffness")
        j_limit = etree.Element("limit")
        j_lower = etree.Element("lower")
        j_upper = etree.Element("upper")

        j_parent.text = name_u
        j_child.text = name
        j_xyz.text = "1 0 0"
        j_damping.text = str(damp)
        j_spring_stiffness.text = str(k_spring * g * math.acos((e**2 + g**2 - f**2) / (2 * e * g)))
        j_lower.text = str(- compression_tol)
        j_upper.text = str(math.pi/2)

        j_dynamics.extend([j_damping, j_spring_stiffness])
        j_limit.extend([j_lower, j_upper])
        j_axis.extend([j_xyz, j_dynamics])
        joint.extend([j_parent, j_child, j_axis])
        self.joints_array.append(joint)

        return leg_down
 
    def generate_xml_links_and_joints(self):
        """ Generate the XML for the model links and joints"""

        body_array = self._create_body()

        legs_array = []
        for key in self.model_config["legs"]:
            leg_id = 0
            if key.find("FL") == 0:
                leg_id = 1
            if key.find("FR") == 0:
                leg_id = 2
            if key.find("BL") == 0:
                leg_id = 3
            if key.find("BR") == 0:
                leg_id = 4
            legs_array.append(self._create_leg_motor(key, leg_id, self.model_config["legs"][key]))
            legs_array.append(self._create_leg_up(key, leg_id, self.model_config["legs"][key]))
            legs_array.append(self._create_leg_down(key, leg_id, self.model_config["legs"][key]))

        self.links_array.extend(body_array)
        self.links_array.extend(legs_array)

    def generate_xml_plugins(self):
        """ Generate the XML for the model plugins """

        xml_plugin = etree.Element('plugin', name='tigrillo_controller', filename='libtigrillo_plugin.so')

        param_p = etree.Element("p")
        param_i = etree.Element("i")
        param_d = etree.Element("d")

        param_p.text = "10.0"
        param_i.text = "30.0"
        param_d.text = "1.0"

        xml_plugin.append(param_p)
        xml_plugin.append(param_i)
        xml_plugin.append(param_d)

        self.plugins_array.extend([xml_plugin])

    def generate_xml_sdf(self):
        """ Generate the XML for all the sdf model """

        if not self.gazebo:
            self.xml_world = etree.Element('world', name="default")
        self.xml_sdf = etree.Element('sdf', version="1.4")
        self.xml_model = etree.Element("model", name="tigrillo")

        self.generate_xml_model_configs()
        self.generate_xml_links_and_joints()
        self.generate_xml_plugins()

        self.xml_model.extend(self.model_configs_array)
        self.xml_model.extend(self.links_array)
        self.xml_model.extend(self.joints_array)

        if self.gazebo:
            self.xml_model.extend(self.plugins_array)
            self.xml_sdf.append(self.xml_model)
        else:
            self.xml_world.append(self.xml_model)
            self.xml_sdf.append(self.xml_world)

    def generate(self):
        """ Main function called to generate a model """

        # Generate all elements
        self.generate_xml_header()
        self.generate_xml_sdf()

        # Create the XML file
        f = open(self.filename, 'w')
        for i, l in enumerate(self.head_array):
            if i == 0:
                f.write(etree.tostring(l, pretty_print=True, xml_declaration=True).decode('utf-8'))
            else:
                f.write(etree.tostring(l, pretty_print=True).decode('utf-8'))
        f.write(etree.tostring(self.xml_sdf, pretty_print=True).decode('utf-8'))
        f.flush()
        f.close()

    def get_model_generator_version(self):

        return self.VERSION

if __name__ == '__main__':

    # MODEL PARAMETERS
    model_config = {
        'default_height': 17,
        'default_density': 1,
        'body': {
            'front': {
                'width': 14,
                'height': 3,
                'length': 8,
                'mass': 2500,
            },
            'middle': {
                'width': 5,
                'height': 0.25,
                'length': 8,
                'mass': 300,
            },
            'hind': {
                'width': 9,
                'height': 2.5,
                'length': 6,
                'mass': 1500,
            },
        },
        'battery_mass': 1170,
        'legs': {
            'FL': {
                'motor': {
                    'width': 3.6,
                    'length': 3.6,
                    'height': 5.06,
                    'leg_attachment_height': 3.3,
                    'mass': 670
                },
                'foot': {
                    'radius': 0.6,
                    'mu1': 1,
                    'mu2': 1,
                },
                'radius': 0.4,
                'joint_slack': 0.5,
                'position': 'front',
                'femur_length': 7,
                'femur_angle': 25,
                'tibia_length': 8.5,
                'spring_length': 2.5,
                'femur_spring_tibia_joint_dst': 4.5,
                'tibia_spring_to_joint_dst': 3.5,
                'hip_damping': 0.2,
                'knee_damping': 0.2,
                'spring_stiffness': 500,
                'spring_comp_tol': 0.1,
                'actuator_kp': 254,
            },
            'FR': {
                'motor': {
                    'width': 3.6,
                    'length': 3.6,
                    'height': 5.06,
                    'leg_attachment_height': 3.3,
                    'mass': 670
                },
                'foot': {
                    'radius': 0.6,
                    'mu1': 1,
                    'mu2': 1,
                },
                'radius': 0.4,
                'joint_slack': 0.5,
                'position': 'front',
                'femur_length': 7,
                'femur_angle': 25,
                'tibia_length': 8.5,
                'spring_length': 2.5,
                'femur_spring_tibia_joint_dst': 4.5,
                'tibia_spring_to_joint_dst': 3.5,
                'hip_damping': 0.2,
                'knee_damping': 0.2,
                'spring_stiffness': 500,
                'spring_comp_tol': 0.1,
                'actuator_kp': 254,
            },
            'BL': {
                'motor': {
                    'width': 3.6,
                    'length': 3.6,
                    'height': 5.06,
                    'leg_attachment_height': 3.3,
                    'mass': 670
                },
                'foot': {
                    'radius': 0.6,
                    'mu1': 1,
                    'mu2': 1,
                },
                'radius': 0.4,
                'joint_slack': 0.5,
                'position': 'back',
                'femur_length': 7,
                'femur_angle': 25,
                'tibia_length': 8.5,
                'spring_length': 2.5,
                'femur_spring_tibia_joint_dst': 4.5,
                'tibia_spring_to_joint_dst': 3.5,
                'hip_damping': 0.2,
                'knee_damping': 0.2,
                'spring_stiffness': 500,
                'spring_comp_tol': 0.1,
                'actuator_kp': 254,
            },
            'BR': {
                'motor': {
                    'width': 3.6,
                    'length': 3.6,
                    'height': 5.06,
                    'leg_attachment_height': 3.3,
                    'mass': 670
                },
                'foot': {
                    'radius': 0.6,
                    'mu1': 1,
                    'mu2': 1,
                },
                'radius': 0.4,
                'joint_slack': 0.5,
                'position': 'back',
                'femur_length': 7,
                'femur_angle': 25,
                'tibia_length': 8.5,
                'spring_length': 2.5,
                'femur_spring_tibia_joint_dst': 4.5,
                'tibia_spring_to_joint_dst': 3.5,
                'hip_damping': 0.2,
                'knee_damping': 0.2,
                'spring_stiffness': 500,
                'spring_comp_tol': 0.1,
                'actuator_kp': 254,
            },
        },
    }

    folder = "../data/robots/"
    gazebo_folder = "/home/gabs48/.gazebo/models/tigrillo/"
    gazebo_folder_small = "/home/gabs48/.gazebo/models/tigrillo_small/"

    # Generate the model as an MJCF file
    fg = MJCFileGenerator(model_config, folder + "tigrillo.mjcf")
    fg.generate()

    # Generate the model as a SDF file
    fg2 = SDFileGenerator(model_config, folder + "tigrillo.sdf")
    fg2.generate()

    # Generate the model as a SDF file
    fg3 = SDFileGenerator(model_config, gazebo_folder + "model.sdf", model_scale=10, gazebo=True)
    fg3.generate()

    # Generate the small model as a SDF file
    fg4 = SDFileGenerator(model_config, gazebo_folder_small + "model.sdf", model_scale=100, gazebo=True)
    fg4.generate()
