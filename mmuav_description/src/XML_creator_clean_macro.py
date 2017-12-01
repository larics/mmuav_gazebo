import xml.etree.ElementTree as ET
from lxml import etree
import sys

def property_generator(properties, xml_parent):

    params = ""
    for i in properties.keys():
        etree.SubElement(xml_parent, "{http://www.ros.org/wiki/xacro}property", name="{0}".format(i), value="{0}".format(properties[i]))

def params_generator(properties):
    params = "base_link:='base_link'"
    for i in properties.keys():
        params += " {0}:={1}".format(i, properties[i])
    return params

def env_links_generator(xml_parent):

    robot = xml_parent
    # world and base
    world_link = etree.SubElement(robot, "link", name="world")
    # base_link
    base_link = etree.SubElement(robot, "link", name="base_link")
    leg1_link = etree.SubElement(robot, "link", name="leg1")
    leg2_link = etree.SubElement(robot, "link", name="leg2")
    # visual_tag
    visual_base = etree.SubElement(base_link, "visual")
    visual_leg1 = etree.SubElement(leg1_link,"visual")
    visual_leg2 = etree.SubElement(leg2_link, "visual")
    geometry_base = etree.SubElement(visual_base, "geometry")
    geometry_leg1 = etree.SubElement(visual_leg1, "geometry")
    geometry_leg2 = etree.SubElement(visual_leg2, "geometry")
    etree.SubElement(geometry_base, "box", size="${box_param} ${3*box_param} ${box_param}")
    etree.SubElement(geometry_leg1, "box", size="${box_param} ${box_param1}  ${spawn_point+box_param/2}")
    etree.SubElement(geometry_leg2, "box", size="${box_param} ${-box_param1}  ${spawn_point+box_param/2}")
    etree.SubElement(visual_leg1, "origin", xyz="0 0 ${-spawn_point/2}", rpy="0 0 0")
    etree.SubElement(visual_leg2, "origin",xyz="0 0 ${-spawn_point/2}", rpy="0 0 0")
    #collision_tag
    collision_base = etree.SubElement(base_link,"collision")
    geometry_base_c = etree.SubElement(collision_base, "geometry")
    etree.SubElement(geometry_base_c, "box", size="${box_param} ${3*box_param} ${box_param}")
    collision_leg1 = etree.SubElement(leg1_link, "collision")
    collision_leg2 = etree.SubElement(leg2_link, "collision")
    geometry_leg1_c = etree.SubElement(collision_leg1, "geometry")
    geometry_leg2_c = etree.SubElement(collision_leg2, "geometry")
    etree.SubElement(geometry_leg1_c, "box", size="${box_param} ${box_param1}  ${spawn_point+box_param/2}")
    etree.SubElement(geometry_leg2_c, "box", size="${box_param} ${-box_param1} ${spawn_point+box_param/2}")
    etree.SubElement(collision_leg1, "origin", xyz="0 0 ${-spawn_point/2}", rpy="0 0 0")
    etree.SubElement(collision_leg2, "origin", xyz="0 0 ${spawn_point/2}", rpy="0 0 0")
    etree.SubElement(collision_base, "origin", xyz="0 0 ${spawn_point}", rpy="0 0 0")
    #inertial_tag
    inertial_base = etree.SubElement(base_link,"inertial")
    inertial_leg1 = etree.SubElement(leg1_link, "inertial")
    inertial_leg2 = etree.SubElement(leg2_link, "inertial")
    etree.SubElement(inertial_base, "mass", value="${small}")
    etree.SubElement(inertial_leg1, "mass", value="${large}")
    etree.SubElement(inertial_leg2, "mass", value="${large}")
    etree.SubElement(inertial_base,"inertia", ixx="${small}", iyy="${small}", izz="${small}", ixy="0.0", ixz="0.0",   iyz="0.0")
    etree.SubElement(inertial_leg1, "inertia", ixx="${large}", iyy="${large}", izz="${large}", ixy = "0.0", ixz="0.0", iyz="0.0")
    etree.SubElement(inertial_leg2, "inertia", ixx="${large}", iyy="${large}", izz="${large}", ixy = "0.0", ixz="0.0", iyz="0.0")
    etree.SubElement(inertial_base,"origin", xyz="0 0 ${spawn_point}", rpy="0 0 0")
    etree.SubElement(inertial_leg1, "origin", xyz="0 ${3*box_param} ${-spawn_point/2}" ,rpy="0 0 0")
    etree.SubElement(inertial_leg2, "origin", xyz="0 ${-3*box_param} ${-spawn_point/2}", rpy="0 0 0")


def link_generator(n, xml_parent): #tocno odrediti koje argumente ce primati funkcija

    robot = xml_parent
    rope_list = []
    link = []
    visual_link = []
    geometry_link = []
    cylinder = []
    origin = []
    collision_link = []
    geometry_col_link = []
    cylinder_col = []
    origin_col = []
    inertial_link = []
    mass_link = []
    inertia_link = []
    inertia_origin = []

    for i in range(0,n):

        rope_list.append("rope_{0}".format(i+1))       #generiranje liste imena
        link.append(etree.SubElement(robot,"link", name="{0}".format(rope_list[i])))
        visual_link.append(etree.SubElement(link[i], "visual"))
        geometry_link.append(etree.SubElement(visual_link[i],"geometry"))
        cylinder.append(etree.SubElement(geometry_link[i], "cylinder", length="${body_len}", radius="${width_1}"))
        origin.append(etree. SubElement(visual_link[i], "origin", xyz="0 0 ${-body_len/2}", rpy="0.0 0.0 0.0"))
        #collision_link.append(etree.SubElement(link[i], "collision"))
        #geometry_col_link.append(etree.SubElement(collision_link[i], "geometry"))
        #cylinder_col.append(etree.SubElement(geometry_col_link[i], "cylinder", length="${body_len}", radius="${width_1}" ))
        #origin_col.append(etree.SubElement(collision_link[i], "origin", xyz="0 0 ${-body_len/2}", rpy = "0.0 0.0 0.0"))
        inertial_link.append(etree.SubElement(link[i],"inertial"))
        mass_link.append(etree.SubElement(inertial_link[i], "mass",value="${mass_rope}"))
        inertia_link.append(etree.SubElement(inertial_link[i],"inertia", ixx="${inertia_rope1}", iyy="${inertia_rope1}", izz="${inertia_rope2}", ixy="0.0", ixz="0.0", iyz="0.0"))
        inertia_origin.append(etree.SubElement(inertial_link[i], "origin",  xyz="0 0 ${-body_len/2}", rpy="0.0 0.0 0.0"))

    return rope_list

def load_link_generator(xml_parent):
        robot = xml_parent
        link = []
        visual_link = []
        geometry_link = []
        cylinder = []
        origin = []
        collision_link = []
        geometry_col_link = []
        cylinder_col = []
        origin_col = []
        inertial_link = []
        mass_link = []
        inertia_link = []
        inertia_origin = []
        load_inertia="${(2/5)*load_mass*radius*radius}"

        link.append(etree.SubElement(robot,"link", name="load"))
        #pose = etree.SubElement(link[-1],"pose")
        #pose.text = "0.0 0.0 0.0 0.0 pi/2 0.0"
        visual_link.append(etree.SubElement(link[-1],"visual"))
        geometry_link.append(etree.SubElement(visual_link[-1],"geometry"))
        cylinder.append(etree.SubElement(geometry_link[-1],"sphere", radius="${radius}"))
        #cylinder.append(etree.SubElement(geometry_link[-1],"cylinder", radius="${radius}", length="${radius * 5}"))
        origin.append(etree.SubElement(visual_link[-1],"origin", xyz="0 0 ${-radius}", rpy="0.0 0.0 0.0" ))
        collision_link.append(etree.SubElement(link[-1],"collision"))
        geometry_col_link.append(etree.SubElement(collision_link[-1],"geometry"))
        cylinder_col.append(etree.SubElement(geometry_col_link[-1],"sphere", radius="${radius}"))
        origin_col.append(etree.SubElement(collision_link[-1], "origin", xyz="0 0 ${-radius}", rpy="0.0 0.0 0.0"))
        inertial_link.append(etree.SubElement(link[-1],"inertial"))
        mass_link.append(etree.SubElement(inertial_link[-1],"mass", value="${load_mass}"))
        inertia_link.append(etree.SubElement(inertial_link[-1],"inertia", ixx="${load_inertia}", iyy="${load_inertia}", izz="${load_inertia}", ixy="0.0", ixz="0.0", iyz="0.0"))
        inertia_origin.append(etree.SubElement(inertial_link[-1],"origin", xyz="0 0 ${-radius}", rpy="0.0 0.0 0.0"))

def joint_generator(n, xml_parent, rope_list, base_list):

    robot = xml_parent

    rope_Theta = []
    rope_Psi = []
    rope_Theta_links = []
    rope_Psi_links = []
    imaginary_inertiaTheta = []
    imaginary_inertiaPsi = []
    joints=[]
    joints1=[]
    joints2=[]
    joints3=[]

    """
    joint = etree.SubElement(robot,"joint",name="world_base_link", type="fixed")
    etree.SubElement(joint, "parent", link="world")
    etree.SubElement(joint, "child", link="base_link")
    etree.SubElement(joint, "origin", xyz="0 0 ${spawn_point}", rpy="0.0 0.0 0.0")

    jointleg1 = etree.SubElement(robot,"joint",name="leg1_base_link", type="fixed")
    etree.SubElement(jointleg1, "parent", link="base_link")
    etree.SubElement(jointleg1, "child", link="leg1")
    etree.SubElement(jointleg1, "origin", xyz="0 ${3*box_param/2+box_param1/2} ${box_param/4}")

    jointleg2 = etree.SubElement(robot,"joint",name="leg2_base_link", type="fixed")
    etree.SubElement(jointleg2, "parent", link="base_link")
    etree.SubElement(jointleg2, "child", link="leg2")
    etree.SubElement(jointleg2, "origin", xyz="0 ${-(3*box_param/2+box_param1/2)} ${box_param/4}")
"""

    for i in range(0, n+1):

        rope_Theta.append("rope{0}Theta".format(int(i+1)))
        rope_Psi.append("rope{0}Psi".format(int(i+1)))

        rope_Theta_links.append(etree.SubElement(robot, "link", name="{0}".format(rope_Theta[i])))
        imaginary_inertiaTheta.append(etree.SubElement(rope_Theta_links[i],"inertial"))
        etree.SubElement(imaginary_inertiaTheta[i],"mass", value="${mass_link}")
        etree.SubElement(imaginary_inertiaTheta[i],"inertia", ixx="${inertia_link}", iyy="${inertia_link}", izz="${inertia_link}", ixy="0.0", ixz="0.0", iyz="0.0")
        rope_Psi_links.append(etree.SubElement(robot,"link", name="{0}".format(rope_Psi[i])))
        imaginary_inertiaPsi.append(etree.SubElement(rope_Psi_links[i], "inertial"))
        etree.SubElement(imaginary_inertiaPsi[i], "mass", value="${mass_link}")
        etree.SubElement(imaginary_inertiaPsi[i], "inertia", ixx="${inertia_link}", iyy="${inertia_link}",
                      izz="${inertia_link}", ixy="0.0", ixz="0.0", iyz="0.0")
        dmp = (n-i)*0.1/n+0.1
        dmp_text = str(dmp)     
        if i == 0:

            joints.append(etree.SubElement(robot,"joint", name="{0}_to_{1}".format(base_list[i],rope_Psi[i]),type="revolute"))
            etree.SubElement(joints[i],"parent", link="${base_link}", )
            etree.SubElement(joints[i],"child", link="{0}".format(rope_Psi[i]))
            etree.SubElement(joints[i],"origin", xyz="0 0 ${-box_param/2}", rpy="0.0 0.0 0.0")
            etree.SubElement(joints[i],"axis", xyz="0 0 1" )
            etree.SubElement(joints[i],"dynamics", damping="${damping1}", friction="${friction}")
            etree.SubElement(joints[i],"limit", lower="${-pi/2}", upper="${pi/2}", effort="${effort_joint}", velocity="${velocity_joint}")

            joints.append(etree.SubElement(robot,"joint",name="{0}_to_{1}".format(rope_Psi[i],rope_Theta[i]),type="revolute"))
            etree.SubElement(joints[i+1], "parent", link="{0}".format(rope_Psi[i-1]) )
            etree.SubElement(joints[i+1], "child", link="{0}".format(rope_Theta[i-1]))
            etree.SubElement(joints[i+1], "axis", xyz="0 1 0")
            etree.SubElement(joints[i+1], "dynamics", damping="${damping1}", friction="${friction}")
            etree.SubElement(joints[i+1], "limit", lower="${-pi/2}", upper="${pi/2}", effort="${effort_joint}",velocity="${velocity_joint}")

            joints.append(etree.SubElement(robot, "joint", name = "{0}_to_{1}".format(rope_Theta[i],rope_list[i]), type = "revolute"))
            etree.SubElement(joints[i+2], "parent", link="{0}".format(rope_Theta[i]))
            etree.SubElement(joints[i+2], "child", link="{0}".format(rope_list[i]))
            etree.SubElement(joints[i+2], "axis", xyz="1 0 0")
            etree.SubElement(joints[i+2], "dynamics", damping="${damping1}", friction="${friction}")
            etree.SubElement(joints[i+2], "limit", lower="${-pi/16}", upper="${pi/16}", effort="${effort_joint}", velocity="${velocity_joint}")

        elif i<n:

            joints1.append(etree.SubElement(robot, "joint", name="{0}_to_{1}".format(rope_list[i - 1], rope_Psi[i]),type="revolute"))
            etree.SubElement(joints1[i - 1], "parent", link="{0}".format(rope_list[i-1]), )
            etree.SubElement(joints1[i - 1], "child", link="{0}".format(rope_Psi[i]))
            etree.SubElement(joints1[i - 1], "origin", xyz="0 0 ${-body_len}", rpy="0.0 0.0 0.0")
            etree.SubElement(joints1[i - 1 ], "axis", xyz="0 0 1")
            etree.SubElement(joints1[i - 1], "dynamics", damping="${damping1}", friction="${friction}")
            etree.SubElement(joints1[i - 1], "limit", lower="${-pi/2}", upper="${pi/2}", effort="${effort_joint}",velocity="${velocity_joint}")

            joints2.append(etree.SubElement(robot, "joint", name="{0}_to_{1}".format(rope_Psi[i], rope_Theta[i]), type = "revolute"))
            etree.SubElement(joints2[i - 1], "parent", link="{0}".format(rope_Psi[i]))
            etree.SubElement(joints2[i - 1], "child", link="{0}".format(rope_Theta[i]))
            etree.SubElement(joints2[i - 1], "axis", xyz="0 1 0")
            etree.SubElement(joints2[i - 1], "dynamics", damping="${damping1}", friction="${friction}")
            etree.SubElement(joints2[i - 1], "limit", lower="${-pi/2}", upper="${pi/2}", effort="${effort_joint}", velocity="${velocity_joint}")

            joints3.append(etree.SubElement(robot, "joint", name="{0}_to_{1}".format(rope_Theta[i], rope_list[i]),type="revolute"))
            etree.SubElement(joints3[i - 1], "parent", link="{0}".format(rope_Theta[i]))
            etree.SubElement(joints3[i - 1], "child", link="{0}".format(rope_list[i]))
            etree.SubElement(joints3[i - 1], "axis", xyz="1 0 0")
            etree.SubElement(joints3[i - 1], "dynamics", damping="${damping1}", friction="${friction}")
            etree.SubElement(joints3[i - 1], "limit", lower="${-pi/16}", upper="${pi/16}", effort="${effort_joint}", velocity="${velocity_joint}")

        else:

            joints1.append(etree.SubElement(robot, "joint", name="{0}_to_{1}".format(rope_list[i - 1], rope_Psi[i]), type="revolute"))
            etree.SubElement(joints1[i - 1], "parent", link="{0}".format(rope_list[i - 1]), )
            etree.SubElement(joints1[i - 1], "child", link="{0}".format(rope_Psi[i]))
            etree.SubElement(joints1[i - 1], "origin", xyz="0 0 ${-body_len}", rpy="0.0 0.0 0.0")
            etree.SubElement(joints1[i - 1], "axis", xyz="0 0 1")
            etree.SubElement(joints1[i - 1], "dynamics", damping="${damping1}", friction="${friction}")
            etree.SubElement(joints1[i - 1], "limit", lower="${-pi/2}", upper="${pi/2}", effort="${effort_joint}", velocity="${velocity_joint}")

            joints2.append(etree.SubElement(robot, "joint", name="{0}_to_{1}".format(rope_Psi[i], rope_Theta[i]), type="revolute"))
            etree.SubElement(joints2[i - 1], "parent", link="{0}".format(rope_Psi[i]))
            etree.SubElement(joints2[i - 1], "child", link="{0}".format(rope_Theta[i]))
            etree.SubElement(joints2[i - 1], "axis", xyz="0 1 0")
            etree.SubElement(joints2[i - 1], "dynamics", damping="${damping1}", friction="${friction}")
            etree.SubElement(joints2[i - 1], "limit", lower="${-pi/2}", upper="${pi/2}", effort="${effort_joint}", velocity="${velocity_joint}")

            joints3.append(etree.SubElement(robot, "joint", name="{0}_to_{1}".format(rope_Theta[i], base_list[-1]), type="revolute"))
            etree.SubElement(joints3[i - 1], "parent", link="{0}".format(rope_Theta[i]))
            etree.SubElement(joints3[i - 1], "child", link="{0}".format(base_list[-1]))
            etree.SubElement(joints3[i - 1], "axis", xyz="1 0 0")
            etree.SubElement(joints3[i - 1], "dynamics", damping="${damping1}", friction="${friction}")
            etree.SubElement(joints3[i - 1], "limit", lower="${-pi/16}", upper="${pi/16}", effort="${effort_joint}",velocity="${velocity_joint}")
    return {"rope_Theta" : rope_Theta, "rope_Psi" : rope_Psi}

def gazebo_reference(xml_parent, rope_list, base_link):
    robot = xml_parent
    for i in  range(0, len(rope_list)+1):
        if i<len(rope_list):
            gazebo = etree.SubElement(robot, "gazebo", reference="{0}".format(rope_list[i]))
            color = ["Blue", "Green", "Red"]
            material = etree.SubElement(gazebo, "material")
            material.text = "Gazebo/{0}".format(color[i%2])
        else:
            gazebo = etree.SubElement(robot, "gazebo", reference="{0}".format(base_link[-1]))
            material = etree.SubElement(gazebo, "material")
            material.text = "Gazebo/{0}".format(color[-1])

def ft_sensors(xml_parent, rope_Theta, rope_Psi):
    gazebo = etree.SubElement(robot, "gazebo", reference="{0}_to_{1}".format(rope_Theta[-1], base_list[-1]))
    provideFeedback = etree.SubElement(gazebo, "provideFeedback")
    provideFeedback.text = "true"

    gazebo = etree.SubElement(robot, "gazebo")
    plugin = etree.SubElement(gazebo, "plugin", name="ft_load", filename="libgazebo_ros_ft_sensor.so")
    topicName = etree.SubElement(plugin, "topicName")
    topicName.text = "ft_sensor_topic_load"
    jointName = etree.SubElement(plugin, "jointName")
    jointName.text = "{0}_to_{1}".format(rope_Theta[-1], base_list[-1])

    gazebo = etree.SubElement(robot, "gazebo", reference="{0}_to_{1}".format(base_list[0], rope_Psi[0]))
    provideFeedback = etree.SubElement(gazebo, "provideFeedback")
    provideFeedback.text = "true"

    gazebo = etree.SubElement(robot, "gazebo")
    plugin = etree.SubElement(gazebo, "plugin", name="ft_base", filename="libgazebo_ros_ft_sensor.so")
    topicName = etree.SubElement(plugin, "topicName")
    topicName.text = "ft_sensor_topic_base"
    jointName = etree.SubElement(plugin, "jointName")
    jointName.text = "{0}_to_{1}".format(base_list[0], rope_Psi[0])


if __name__ == '__main__':

    if len(sys.argv) < 5:
        print 'Usage: xml_creator_clean.py rope_length no_links load_mass spawn_point'
    else:
        length=float(sys.argv[1])
        n=int(sys.argv[2])
        load_mass=float(sys.argv[3])
        spawn_point=float(sys.argv[4])

        ROBOT_NAMESPACE = "http://www.ros.org/wiki/xacro"
        ROBOT = "{%s}" % ROBOT_NAMESPACE
        NSMAP = {"xacro" : ROBOT_NAMESPACE} # the default namespace prefix xacro
        robot = etree.Element("robot", nsmap=NSMAP, name="myrobot") # lxml only!
        #urdf/xml_creator

        width_1 = length/300
        body_len = length/n

        mass_rope = 0.1 #ukloniti ovisnost o broju clanaka
        mass_link=0.01

        mass_rope = 0.05 * length / n
#        mass_link= mass_rope * 0.1

        inertia_rope1=(1/12.0)*mass_rope*body_len**2+(1/4.0)*mass_rope*width_1**2
        inertia_rope2=(1/2.0)*mass_rope*width_1**2
        #inertia_rope1 = 0.01
        #inertia_rope2 = 0.02

        #mass_link = load_mass/4000*(n/5)

        inertia_link = (load_mass/400)*(n/5.0)
        load_inertia = (2.0/5.0)*load_mass*(load_mass/100.0)*(load_mass/100.0)

        base_list = ["base_link","load"]
        properties = {
            'load_mass' : load_mass,
            'spawn_point' : spawn_point,
            'width_1' : width_1,
            'body_len' : body_len,
            'mass_rope' : mass_rope,
            'mass_link' : mass_link,
            'inertia_rope1' : inertia_rope1,
            'inertia_rope2' : inertia_rope2,
            'inertia_link' : inertia_link,
            'load_inertia' : load_inertia,
            'velocity_joint' : 300,
            'box_param' : 0.0,
            'box_param1' : 0.25,
            'radius' : 0.1+load_mass/2000,
            'small' : 0.005,
            'large' : 100,
            'friction' : 0.02,
            'damping1' : 0.5,
            'effort_joint' : 1000}

        params_string = params_generator(properties)

        robot_macro = etree.SubElement(robot, "{http://www.ros.org/wiki/xacro}macro", name="robot_macro", params=params_string)
        xml_parent = robot_macro
        #property_generator(properties, xml_parent)
        #env_links_generator(xml_parent)
        rope_list = link_generator(n, xml_parent)
        load_link_generator(xml_parent)
        ropes_lists = joint_generator(n, xml_parent, rope_list, base_list)
        gazebo_reference(xml_parent, rope_list, base_list)
        ft_sensors(xml_parent, ropes_lists["rope_Theta"], ropes_lists["rope_Psi"])
        tree = etree.tostring(robot, pretty_print=True, encoding="utf-8", xml_declaration=True)
        with open("rope_macro.xacro",'w') as f:
            f.write(tree)
