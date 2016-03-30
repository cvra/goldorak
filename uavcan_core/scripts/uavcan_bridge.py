import uavcan
import uavcan.node
import os.path

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


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Example node that receives motor topics.")
    parser.add_argument('topic')
    parser.add_argument('--interface', '-i', default='vcan0')

    args = parser.parse_args()

    load_dsdl()
    node = uavcan.node.make_node(args.interface)

    def callback(transfer):
        msg = transfer.message
        print("{}({})".format(type_name(msg), get_msg_values(msg)))

    subscribe(node, args.topic, callback)

    node.spin()

if __name__ == '__main__':
    main()
