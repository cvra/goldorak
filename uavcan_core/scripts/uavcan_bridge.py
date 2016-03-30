import os.path

import uavcan
import uavcan.node

import cvra_msgs.msg
import rospy

BASE_PATH = os.path.join(os.path.dirname(__file__), '..', '..')

def load_dsdl():
    uavcan.load_dsdl(os.path.join(BASE_PATH, 'cvra_msgs', 'uavcan', 'cvra'))

def find_msg(name):
    return uavcan.TYPENAMES[name]

def subscribe(node, topic_name, callback_function):
    node.add_handler(find_msg(topic_name), callback_function)

def fields_for_msg(msg):
    return uavcan.get_uavcan_data_type(msg).fields

def get_msg_values(msg):
    fields = fields_for_msg(msg)
    result = {f.name: getattr(msg, f.name) for f in fields}

    return result

def type_name(msg):
    return uavcan.get_uavcan_data_type(msg).full_name

def publish(node, type_name, data):
    Message = find_msg(type_name)
    msg = Message(**data)

    node.broadcast(msg)

def ros_type_from_uavcan_type(uavcan_type):
    name = uavcan_type.full_name

    # Convert uavcan name convention to cvra/ROS
    name = "".join(s.capitalize() for s in name.split('.')[1:])
    return getattr(cvra_msgs.msg, name)

def ros_msg_from_uavcan_msg(uavcan_type, uavcan_msg):
    msg_type = ros_type_from_uavcan_type(uavcan_type)
    values = get_msg_values(uavcan_msg)
    return msg_type(**values)

def main():
    import argparse, json
    parser = argparse.ArgumentParser(description="Example node that receives motor topics.")
    parser.add_argument('topic')
    parser.add_argument('--interface', '-i', default='vcan0')
    parser.add_argument("--id", default=None, type=int)
    parser.add_argument('--publish', '-p')

    args = parser.parse_args()

    load_dsdl()
    node = uavcan.node.make_node(args.interface, node_id=args.id)
    rospy.init_node('uavcan_bridge', anonymous=True)

    ros_type = ros_type_from_uavcan_type(find_msg(args.topic))
    pub = rospy.Publisher('/'.join(s for s in args.topic.split('.')[1:]), ros_type, queue_size=10)

    def callback(transfer):
        msg = transfer.message
        print("UAVCAN: {}({})".format(type_name(msg), get_msg_values(msg)))
        ros_msg = ros_msg_from_uavcan_msg(find_msg(type_name(msg)), msg)
        print("ROS: {} ({})".format(ros_type._type, ros_msg))
        pub.publish(ros_msg)

    if args.publish:
        data = json.loads(args.publish)
        publish(node, args.topic, data)
        node.spin(0.5)
    else:
        subscribe(node, args.topic, callback)
        node.spin()

if __name__ == '__main__':
    main()
