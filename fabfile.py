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
