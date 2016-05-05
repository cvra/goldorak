from fabric.api import *
from fabric.context_managers import shell_env


def _bash_local(command_string):
    """
    Creates a new local command to enforce /bin/bash as shell
    """
    local(command_string, shell="/bin/bash")


def goldorak():
    """
    Setup the environment to use it on Goldorak
    """
    env.hosts += ['192.168.8.2']
    env.user = 'ubuntu'

def build():
    """
    Compiles the project
    """
    _bash_local('./build.sh')

def rebuild():
    """
    Makes a clean build of the project
    """
    _bash_local('catkin clean -y')
    _bash_local('./build.sh')

def launch():
    """
    Launch robot's full ROS stack
    """
    if len(env.hosts) > 0:
        ip = env.hosts[-1]
    else:
        ip = '127.0.0.1'

    with shell_env(ROS_IP=ip, ROS_MASTER_URI='http://{}:11311'.format(ip)):
        _bash_local('source ~/catkin_ws/devel/setup.bash')
        _bash_local('roslaunch goldorak_bringup goldorak.launch')

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
        _bash_local('source ~/catkin_ws/devel/setup.bash')
        _bash_local('roslaunch goldorak_bringup monitor.launch')
