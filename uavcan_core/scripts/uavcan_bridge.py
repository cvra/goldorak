import os.path
import re

import uavcan
import uavcan.node

import cvra_msgs.msg
import rospy

BASE_PATH = os.path.join(os.path.dirname(__file__), '..', '..')

def load_uavcan_msg_dsdl():
    uavcan.load_dsdl(os.path.join(BASE_PATH, 'cvra_msgs', 'uavcan', 'cvra'))

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

def ros_type_from_uavcan_type(uavcan_type):
    name = uavcan_type.full_name

    # Convert uavcan name convention to cvra/ROS
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


def main():
    import argparse, json
    parser = argparse.ArgumentParser(description="Example node that receives motor topics.")
    parser.add_argument('topic')
    parser.add_argument('--interface', '-i', default='vcan0')
    parser.add_argument("--id", default=None, type=int)
    parser.add_argument('--publish', '-p', help="Publish to UAVCAN")

    args = parser.parse_args()

    load_uavcan_msg_dsdl()
    node = uavcan.node.make_node(args.interface, node_id=args.id)
    rospy.init_node('uavcan_bridge', anonymous=True)

    ros_type = ros_type_from_uavcan_type(find_uavcan_msg(args.topic))
    pub = rospy.Publisher('/'.join(s for s in args.topic.split('.')[1:]), ros_type, queue_size=10)

    def callback(transfer):
        msg = transfer.message
        data = get_uavcan_msg_values(msg)
        print("UAVCAN: {}({})".format(uavcan_type_name(msg), data))
        pub.publish(ros_type(**data))
        print("ROS: {}({})".format(ros_type._type, data))

    if args.publish:
        data = json.loads(args.publish)
        uavcan_publish(node, args.topic, data)
        node.spin(0.5)
    else:
        uavcan_subscribe(node, args.topic, callback)
        node.spin()

if __name__ == '__main__':
    main()
