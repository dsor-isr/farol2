# This python script uses contributions from the Niryo One ROS
# repository: https://github.com/NiryoRobotics/niryo_one_ros.git
# It mimics the beahavior of Niryo robot launch. Thank you.
########################################################################
# niryo_one_ros_setup.py
# Copyright (C) 2017 Niryo
# All rights reserved.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
########################################################################

import rclpy
from rclpy.node import Node
from fileinput import FileInput
import time
import subprocess
import fnmatch
import os
import shutil
import yaml

from std_msgs.msg import String, Int8, Bool

from farol_msgs.msg import ProcessState
from farol_msgs.srv import ManageProcess

PROCESS_TIMEOUT_RESTART = 5.0  # sec

class ProcessNotFound(Exception): pass

class ProcessActionType(object):
  START = 1
  STOP = 2
  RESTART = 3
  KILL = 4
  START_ALL = 5
  STOP_ALL = 6

class Process:
  def __init__(self, name, cmd, vehicle_name, vehicle_id, vehicle_ns, 
               config_package_path, args=None, launch_on_startup=False,
               delay_before_start=0.0, dependencies=None):
    self.name = name
    self.config_package_path = config_package_path
    self.cmd = cmd
    self.args = args if args is not None else []
    self.dependencies = dependencies if dependencies is not None else []
    self.launch_on_startup = launch_on_startup
    self.delay_before_start = delay_before_start
    self.process = None
    self.vehicle_name = vehicle_name
    self.vehicle_id = vehicle_id
    self.vehicle_ns = vehicle_ns

  def start(self):
    if not self.isActive():
      cmd = self.cmd.split(' ') + self.args + ["vehicle_ns:=" + self.vehicle_ns] + ["config_package_path:=" + self.config_package_path]

      if self.delay_before_start:
        time.sleep(self.delay_before_start)
      self.process = subprocess.Popen(cmd)


  def restart(self):
    self.stop()
    timeout = time.time() + PROCESS_TIMEOUT_RESTART
    while self.isActive():
      if time.time() > timeout:
        break
      time.sleep(0.1)
    self.start()

  def stop(self):
    if self.process:
      self.process.terminate()
      self.process = None

  def kill(self):
    if self.process:
      self.process.kill()
      self.process = None

  def isActive(self):
    if not self.process:
      return False

    return_code = self.process.poll()
    if return_code is None or return_code < 0:
      return True
    return False

class FarolBringup(Node):
  def __init__(self):
    # Call the parent constructor
    super().__init__('farol_bringup')

    self.loadParams()

    # create vehicle namespace
    self.vehicle_ns = self.vehicle_name + str(self.vehicle_id)

    self.initialiseSubscribers()
    self.initialisePublishers()
    self.initialiseServices()
    self.initialiseTimers()

    # create temporary ROS config files
    self.createROSTempConfigs()

    # populate process_list
    self.createProcesses()

    # start processes in process_list
    self.startInitProcesses()

  ## Load parameters
  def loadParams(self):
    # declare all parameters
    self.declare_parameter('id', 0)
    self.declare_parameter('name', 'vehicle')
    self.declare_parameter('config_package_path', 'medusa_bringup')
    self.declare_parameter('farol_bringup_package_path', 'farol_bringup')
    self.declare_parameter('processes_path', '')

    # actually get the parameters
    self.vehicle_id = self.get_parameter('id').get_parameter_value().integer_value
    self.vehicle_name = self.get_parameter('name').get_parameter_value().string_value
    self.config_package_path = self.get_parameter('config_package_path').get_parameter_value().string_value
    self.farol_bringup_package_path = self.get_parameter('farol_bringup_package_path').get_parameter_value().string_value
    self.processes_path = self.get_parameter('processes_path').get_parameter_value().string_value

    # check process.yaml path exists
    if not self.processes_path or not os.path.exists(self.processes_path):
      self.get_logger().error(f"Invalid config file path: {self.processes_path}")
      return
    
    # get process.yaml dict from path
    with open(self.processes_path, 'r') as f:
      self.processes = yaml.safe_load(f)['processes']
  
  ## Initialise subscribers
  def initialiseSubscribers(self):
    return
  
  ## Initialise publishers
  def initialisePublishers(self):
    process_state_topic = 'process_state'
    self.process_state_pub = self.create_publisher(ProcessState, process_state_topic, 1)
  
  ## Initialise services
  def initialiseServices(self):
    manage_process_service = 'manage_process'
    self.manage_process_srv = self.create_service(ManageProcess, manage_process_service, self.callbackManageProcess)
  
  ## Initialise timers
  def initialiseTimers(self):
    timer_period = 0.5  # seconds
    self.timer = self.create_timer(timer_period, self.timerCallback)

  def createROSTempConfigs(self):
    # get path to ros config file (personal and default) under install/
    personal_config_path = os.path.join(self.config_package_path, 'config_personal/ros.yaml')
    default_config_path = os.path.join(self.farol_bringup_package_path, 'config_default/ros.yaml')

    # check if code was compiled with --symlink-install,
    # i.e., the config file is linked to the one under src/
    if not os.path.islink(personal_config_path):
      self.get_logger().warn(f"No symlink detected (configs won't be updated upon changes without compilation): {self.processes_path}")

    # get real path (the one which the symlink points to
    personal_real_path = os.path.realpath(personal_config_path)

    # create .ros_tmp folder under src/[personal_bringup]/config_personal if it does not exist yet
    ros_tmp_folder = personal_real_path.removesuffix('ros.yaml') + '.ros_tmp/'
    if not os.path.exists(ros_tmp_folder):
      os.mkdir(ros_tmp_folder)

    # save paths for tmp files
    self.tmp_personal_config = ros_tmp_folder + 'personal_ros_' + self.vehicle_ns + '.yaml'
    self.tmp_default_config = ros_tmp_folder + 'default_ros_' + self.vehicle_ns + '.yaml'

    # copy both personal and default config files to .ros_tmp/
    shutil.copy2(personal_config_path, self.tmp_personal_config)
    shutil.copy2(default_config_path, self.tmp_default_config)

    # replace #vehicle# with appropriate vehicle namespace
    self.find_replace(ros_tmp_folder, 'personal_ros_' + self.vehicle_ns + '.yaml', '#vehicle#' , self.vehicle_ns)
    self.find_replace(ros_tmp_folder, 'default_ros_' + self.vehicle_ns + '.yaml', '#vehicle#' , self.vehicle_ns)

  @staticmethod
  def createResponse(status, message):
    response = ManageProcess.Response()
    response.status = status
    response.message = message
    return response

  def publishProcessState(self):
    msg = ProcessState()
    for p in self.process_list:
      msg.name.append(p.name)
      msg.is_active.append(p.isActive())
    self.process_state_pub.publish(msg)

  def callbackManageProcess(self, req, resp):
    process_name = req.name
    action = req.action

    try:
      if action == ProcessActionType.START_ALL:
        self.startAllProcesses()
        return self.createResponse(200, "All processes have been started")

      if action == ProcessActionType.STOP_ALL:
        self.stopAllProcesses()
        return self.createResponse(200, "All processes have been stopped")

      if action == ProcessActionType.START:
        self.startProcessFromName(process_name, start_dependencies=True)
        return self.createResponse(200, "Process has been started")
      elif action == ProcessActionType.STOP:
        self.stopProcessFromName(process_name)  # also stop processes that depends on this process ?   
        return self.createResponse(200, "Process has been stopped")
      elif action == ProcessActionType.RESTART:
        self.restartProcessFromName(process_name)
        return self.createResponse(200, "Process has been restarted")
      elif action == ProcessActionType.KILL:
        self.killProcessFromName(process_name)
        return self.createResponse(200, "Process has been killed")

    except ProcessNotFound as e:
        return self.createResponse(400, str(e))

  def cleanRosProcesses(self):
    # delete the temporary files created in .ros_tmp folder
    os.remove(self.tmp_personal_config)
    os.remove(self.tmp_default_config)
    self.get_logger().info(self.tmp_personal_config)
    self.stopAllProcesses()
  
  def createProcesses(self):
    self.get_logger().info("Start creating processes from process.yaml")
    self.process_list = []

    for p in self.processes:
      self.process_list.append(Process(name=p['name'], cmd=p['cmd'], args=p['args'],
                                       launch_on_startup=p['launch_on_startup'],
                                       delay_before_start=p['delay_before_start'],
                                       dependencies=p['dependencies'], 
                                       vehicle_name=self.vehicle_name,
                                       vehicle_id=self.vehicle_id,
                                       vehicle_ns=self.vehicle_ns,
                                       config_package_path=self.config_package_path))

  def startInitProcesses(self):
    for process in self.process_list:
      if process.launch_on_startup:
        self.startProcess(process, start_dependencies=True)

  def startAllProcesses(self):
    for process in self.process_list:
      self.startProcess(process, start_dependencies=True)

  def stopAllProcesses(self):
    for p in self.process_list:
      self.stopProcess(p)

  def getProcessFromName(self, name):
    p = None
    for process in self.process_list:
      if process.name == name:
        p = process
        break
    if p is None:
      raise ProcessNotFound("Process not found : " + str(name))
    return p

  def getDependencyProcessList(self, process):
    dep_name_list = process.dependencies
    try:
      return list(map(lambda dep_name: self.getProcessFromName(dep_name), dep_name_list))
    except ProcessNotFound as e:  # should never happen if yaml file is correct
      self.get_logger().warn("Some dependency names are incorrect. Check your setup.yaml file to fix it")
      return []

  def areDependenciesMet(self, process):
    process_dep_list = self.getDependencyProcessList(process)
    for p in process_dep_list:
      if not p.isActive():  # isActive doesn't mean all the nodes are fully started (not a problem if nodes wait for services and actions)
        self.get_logger().info("Unmet dependency for " + str(process.name) + " (depends on : " + str(p.name) + ") !")
        return False
    return True

  # CAREFUL : recursion - todo mettre une securite (pas plus de 5 depth)
  def checkAndStartDependencies(self, process):
    process_dep_list = self.getDependencyProcessList(process)
    for p in process_dep_list:
      if not p.isActive():  # isActive doesn't mean all the nodes are fully started (not a problem if nodes wait for services and actions)
        self.get_logger().info("Unmet dependency for " + str(process.name) + " (depends on : " + str(p.name) + ") !")
        self.get_logger().info("Starting dependency process...")
        self.startProcess(p, start_dependencies=True)

  def startProcess(self, p, start_dependencies=False):
    self.get_logger().info("Handle process : " + str(p.name))
    if start_dependencies:
      self.checkAndStartDependencies(p)
      self.get_logger().info("Start process : " + str(p.name))
      p.start()
    else:
      if self.areDependenciesMet(p):
        self.get_logger().info("Start process : " + str(p.name))
        p.start()

  @staticmethod
  def stopProcess(p):
    p.stop()

  @staticmethod
  def restartProcess(p):
    p.restart()

  @staticmethod
  def killProcess(p):
    p.kill()

  def startProcessFromName(self, name, start_dependencies=False):
    p = self.getProcessFromName(name)
    self.startProcess(p, start_dependencies=start_dependencies)

  def stopProcessFromName(self, name):
    p = self.getProcessFromName(name)
    self.stopProcess(p)

  def restartProcessFromName(self, name):
    p = self.getProcessFromName(name)
    self.restartProcess(p)

  def killProcessFromName(self, name):
    p = self.getProcessFromName(name)
    self.killProcess(p)

  ## Timer callback for this node
  #  Where the algorithtms will constantly run
  def timerCallback(self):
    # publish processes' state
    self.publishProcessState()
  
  def find_replace(self, topdir, file_pattern, text, replacement):
    """
    Replace the string #vehicle# with the desired vehicle namespace
    :param topdir: directory with rostopics name parameters to be checked
    :param file_pattern: In this case .yaml
    :param text: string to be replaced
    :param replacement: chosen string (vehicle_name)
    """
    # Check if folder is not empty
    if self.isNotEmpty(topdir):
      for dirpath, dirs, files in os.walk(topdir, topdown=True):
        files = [os.path.join(dirpath, filename) for filename in fnmatch.filter(files, file_pattern)]
        for line in FileInput(files, inplace=True):
          print(line.replace(text, replacement), end='')
  
  @staticmethod
  def isNotEmpty(path):
    """
    Check if folder exists and is not empty
    :param path: directory with parameters to be checked
    """
    if os.path.exists(path) and not os.path.isfile(path):
      # Checking if the directory is empty or not
      if not os.listdir(path):
        return False
      else:
        return True
    else:
      return False


def main(args=None):
  rclpy.init(args=args)

  farol_bringup = FarolBringup()

  rclpy.spin(farol_bringup)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  farol_bringup.cleanRosProcesses()
  farol_bringup.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()