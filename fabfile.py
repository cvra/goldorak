from fabric.api import *
from fabric.context_managers import shell_env


def _bash_run(command_string):
    """
    Creates a new local command to enforce /bin/bash as shell
    """
    if len(env.hosts) > 0:
        run(command_string, shell="/bin/bash")
    else:
        local(command_string, shell="/bin/bash")

def goldorak():
    """
    Setup the environment to use it on Goldorak
    """
    env.hosts += ['192.168.8.2']
    env.user = 'ubuntu'
    env.password = 'temppwd'

def build():
    """
    Compiles the project
    """
    with cd('~/catkin_ws/src/goldorak'):
        _bash_run('./build.sh')

def rebuild():
    """
    Makes a clean build of the project
    """
    _bash_run('catkin clean -y')
    build()

def launch():
    """
    Launch robot's full ROS stack
    """
    if len(env.hosts) > 0:
        ip = env.hosts[-1]
    else:
        ip = '127.0.0.1'

    with shell_env(ROS_IP=ip, ROS_MASTER_URI='http://{}:11311'.format(ip)):
        _bash_run('source ~/catkin_ws/devel/setup.bash')
        _bash_run('roslaunch goldorak_bringup goldorak.launch')

def monitor():
    """
    Monitor running robot state
    """
    if len(env.hosts) > 0:
        robot_ip = env.hosts[-1]
        local_ip = '192.168.8.1'
    else:
        robot_ip = '127.0.0.1'
        local_ip = '127.0.0.1'

    with shell_env(ROS_IP=local_ip, ROS_MASTER_URI='http://{}:11311'.format(robot_ip)):
        _bash_run('source ~/catkin_ws/devel/setup.bash')
        _bash_run('roslaunch goldorak_bringup monitor.launch')

def vcan(vcan_name):
    """
    Creates a virtual CAN interface with given name
    """
    run('sudo modprobe can')
    run('sudo modprobe can_raw')
    run('sudo modprobe can_bcm')
    run('sudo modprobe vcan')

    run('sudo ip link add dev {} type vcan'.format(vcan_name))
    run('sudo ip link set up {}'.format(vcan_name))

    run('sudo ifconfig {} up'.format(vcan_name))
