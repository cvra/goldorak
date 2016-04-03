import os.path
import re

import uavcan
import uavcan.node

import cvra_msgs.msg
import rospy
import rospkg


def load_uavcan_msg_dsdl():
    rospack = rospkg.RosPack()
    BASE_PATH = rospack.get_path('cvra_msgs')
    uavcan.load_dsdl(os.path.join(BASE_PATH, 'uavcan', 'cvra'))

def find_uavcan_msg(name):
    return uavcan.TYPENAMES[name]

def uavcan_subscribe(node, topic_name, callback_function):
    node.add_handler(find_uavcan_msg(topic_name), callback_function)

def uavcan_msg_fields(msg):
    return uavcan.get_uavcan_data_type(msg).fields

def get_uavcan_msg_values(msg):
    fields = uavcan_msg_fields(msg)
    result = {f.name: getattr(msg, f.name) for f in fields}

    return result

def uavcan_type_name(msg):
    return uavcan.get_uavcan_data_type(msg).full_name

def uavcan_publish(node, type_name, data):
    Message = find_uavcan_msg(type_name)
    msg = Message(**data)

    node.broadcast(msg)

def get_uavcan_msg_type(msg):
    return msg._type

def get_ros_msg_type(msg):
    return msg.__class__

def ros_type_from_uavcan_type(uavcan_type):
    name = uavcan_type.full_name

    # Convert uavcan name convention to cvra/ROS
    name = "".join(s.capitalize() for s in name.split('.')[1:])
    return getattr(cvra_msgs.msg, name)

def uavcan_type_from_ros_type(ros_type):
    name = str(cvra_msgs.msg.MotorControlVelocity).split('.')[-1][:-2]
    fields = re.sub("([A-Z])", "_\\1", name).lower().lstrip("_").split("_")
    name = 'cvra.'
    name += '.'.join(fields[:-1])
    name += ''.join(['.', fields[-1].capitalize()])
    return find_uavcan_msg(name)

def get_ros_msg_values(ros_msg):
    ros_type = ros_msg.__class__
    return {field: getattr(ros_msg, field) for field in ros_type.__slots__}

def uavcan_node_names(nodes):
    return {nodes[node]['id']: node for node in nodes}

def find_node_uavcan_msgs(node_type):
    return set([msg \
                for msg in uavcan.TYPENAMES \
                if msg.split('.')[0] == 'cvra' and msg.split('.')[1] == node_type])

def uavcan_is_subscribed(node, topic):
    node_subscriptions = node._handler_dispatcher._handlers
    for subscription in node_subscriptions:
        if str(subscription[0]) == topic:
            return True

    return False
