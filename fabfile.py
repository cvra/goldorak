from fabric.api import *

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
    local('./build.sh')

def rebuild():
    """
    Makes a clean build of the project
    """
    local('catkin clean -y')
    local('./build.sh')
