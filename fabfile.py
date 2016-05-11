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
        _bash_run('source ~/catkin_ws/devel/setup.bash && \
                   roslaunch goldorak_bringup goldorak.launch')

@parallel
def strat():
    """
    Launch robot's strategy node
    """
    if len(env.hosts) > 0:
        robot_ip = env.hosts[-1]
        local_ip = '192.168.8.1'
    else:
        robot_ip = '127.0.0.1'
        local_ip = '127.0.0.1'

    with shell_env(ROS_IP=local_ip, ROS_MASTER_URI='http://{}:11311'.format(robot_ip)):
        local('source ~/catkin_ws/devel/setup.bash && \
                   python ~/catkin_ws/src/goldorak/goldorak_strategy/scripts/smach_demo.py', shell='/bin/bash')

@parallel
def strat_view():
    """
    Launch robot's strategy node
    """
    if len(env.hosts) > 0:
        robot_ip = env.hosts[-1]
        local_ip = '192.168.8.1'
    else:
        robot_ip = '127.0.0.1'
        local_ip = '127.0.0.1'

    with shell_env(ROS_IP=local_ip, ROS_MASTER_URI='http://{}:11311'.format(robot_ip)):
        local('source ~/catkin_ws/devel/setup.bash && \
               rosrun smach_viewer smach_viewer.py', shell='/bin/bash')

def monitor():
    """
    Monitor robot using Rviz
    """
    if len(env.hosts) > 0:
        robot_ip = env.hosts[-1]
        local_ip = '192.168.8.1'
    else:
        robot_ip = '127.0.0.1'
        local_ip = '127.0.0.1'

    with shell_env(ROS_IP=local_ip, ROS_MASTER_URI='http://{}:11311'.format(robot_ip)):
        local('source ~/catkin_ws/devel/setup.bash && \
               roslaunch goldorak_bringup monitor.launch',
               shell="/bin/bash")

def rqt():
    """
    Opens RQT on shared ROS network
    """
    if len(env.hosts) > 0:
        robot_ip = env.hosts[-1]
        local_ip = '192.168.8.1'
    else:
        robot_ip = '127.0.0.1'
        local_ip = '127.0.0.1'

    with shell_env(ROS_IP=local_ip, ROS_MASTER_URI='http://{}:11311'.format(robot_ip)):
        local('source ~/catkin_ws/devel/setup.bash && \
               rqt',
               shell="/bin/bash")

def vcan(vcan_name):
    """
    Creates a virtual CAN interface with given name
    """
    _bash_run('sudo modprobe can && \
               sudo modprobe can_raw && \
               sudo modprobe can_bcm && \
               sudo modprobe vcan && \
               sudo ip link add dev {0} type vcan && \
               sudo ip link set up {0} && \
               sudo ifconfig {0} up'.format(vcan_name))

def share_internet():
    """
    Share internet with robot through Ethernet
    """
    local('sudo iptables -A POSTROUTING -t nat -j MASQUERADE')
    local('sudo echo 1 | sudo tee /proc/sys/net/ipv4/ip_forward > /dev/null')

    run('sudo /sbin/route add default gw 192.168.8.1')
    run('sudo -s && echo "nameserver 8.8.8.8" >> /etc/resolv.conf && exit')
    run('sudo ntpdate -b -s -u pool.ntp.org')

def beacon():
    """
    Builds the beacon firmware and flashes it
    """
    with lcd('proximity-beacon-firmware'):
        local('pwd')
        local('make dsdlc')
        local('packager/packager.py')
        local('make')

        put('build/motor-control-firmware.bin', '/tmp/beacon.bin')

    # wait for user input before flashing
    flash_command = "read &&"
    flash_command += " bootloader_flash"
    # Base adress
    flash_command += " -a 0x08003800"
    flash_command += " -i can1"
    flash_command += " --device-class proximity-beacon"
    flash_command += " -b /tmp/beacon.bin"
    flash_command += " --run"
    flash_command += " 54"
    run(flash_command)

    run('rm /tmp/beacon.bin')
