#!/usr/bin/env python
import sys
import signal
import os
import socket
import traceback

import yaml
import time

# Threads
import threading

from functools import partial

import xml.etree.ElementTree as ET
import re

# ROS stuff
# import roslib.message
# import rospy
# import rosgraph

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.duration import Duration
import importlib

# Messages
from std_msgs.msg import *
from farol_msgs.msg import *
from geometry_msgs.msg import *

# Services
# from waypoint.srv import *
from waypoint.srv import SendWpType1

from collections.abc import Mapping, Sequence

# HTTP Server
import cgi, cgitb, urllib
#from urlparse import urlparse
# from future.standard_library import install_aliases
# install_aliases()

from urllib.parse import urlparse, urlencode
from urllib.request import urlopen, Request
from urllib.error import HTTPError

#from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
from http.server import HTTPServer, BaseHTTPRequestHandler

from socketserver import ThreadingMixIn
#from SocketServer import ThreadingMixIn
from _socket import error as SocketError

import array

# MISSION_PATH ="/home/cog/cog-sw/Missions_FOLDER" # TODO: Change this to a PARAMETER
#MISSION_PATH ="/home/jorge/Farol_CODE/Missions_FOLDER" # TODO: Change this to a PARAMETER
# pages_folder = ""
NFILE=0 # Temp filename

from operator import itemgetter



class ROSTopicException(Exception):
  """
  Base exception class of rostopic-related errors
  """
  pass
class ROSTopicIOException(ROSTopicException):
  """
  rostopic errors related to network I/O failures
  """
  pass
  
class Topic_Struct(object):
  # TODO: Include Mutexs to this struct
  def __init__(self):
    self.time_rcv = []
    self.topics = []
    self.topics_data = []
  
  # Print all the topics to a string
  def __str__(self):
    text = ""
    for i in range(len(self.topics)):
      tnow = Clock().now()
      if((tnow-self.time_rcv[i]).nanoseconds / 1e9 < 10):
        print ("---->" + text)
        text += ros2xml(self.topics_data[i], self.topics[i],0)
    return text
  
  # Add a topic or substitute one
  def add_value(self, topic_name, data):
    tnow = Clock().now()
    if not topic_name.startswith("/"):
      topic_name = "/" + topic_name
    if self.topics.count(topic_name) == 0 : # new topic
      self.topics.append(topic_name)
      self.topics_data.append(data)
      self.time_rcv.append(tnow)
    else: # update existing
      self.topics_data[self.topics.index(topic_name)] = data
      self.time_rcv[self.topics.index(topic_name)] = tnow

  
  # Add a topic or substitute one
  def del_value(self, topic_name):
    if self.topics.count(topic_name) == 0 :
      return
    ind = self.topics.index(topic_name)
    self.topics.pop(ind)
    self.topics_data.pop(ind)
    self.time_rcv.pop(ind)

  # Search for a topic name
  def data_topic(self, topic_name):
    tnow = Clock().now()
    if not topic_name.startswith("/"):
      topic_name = "/" + topic_name
    if self.count(topic_name) > 0 :
      if((tnow-self.time_rcv[self.topics.index(topic_name)]).nanoseconds / 1e9 < 10):
        return [self.topics_data[self.topics.index(topic_name)], self.time_rcv[self.topics.index(topic_name)]]
    return [0, 0]
  
  # Number of times that a topic name appears
  def count(self, topic_name):
    if not topic_name.startswith("/"):
      topic_name = "/" + topic_name
    #print ("count:" + topic_name + " " + str(self.topics.count(topic_name)))
    #print (self.topics)
    count_topic = self.topics.count(topic_name)
    tnow = Clock().now()
    # if(count_topic>0):
    #   if((tnow-self.time_rcv[self.topics.index(topic_name)]).nanoseconds / 1e9 >= 10):
    #     return 0
    return count_topic
    
  # Returns a String with all the topics saved
  def str_list(self):
    msg = ""
    for i in self.topics:
      msg += i + "\n"
    return msg

ALL_TOPICS = Topic_Struct()
SUBSCRIBED_TOPICS = Topic_Struct()
UNKNOWN_TOPICS = Topic_Struct()

# ERRORS List for rosout
class ERROR_Struct(object):
  # TODO: Include Mutexs to this struct
  def __init__(self):
    self.node_name = []
    self.line = []
    self.msg = []
    self.time = []
    self.time_rcv = []
  
  def __clear__(self):
    self.node_name = []
    self.line = []
    self.msg = []
    self.time = []
    self.time_rcv = []
  # 
  def __clearitem__(self, i):
    tnow = Clock().now()
    if( self.time[i]!=-1 and ( tnow - self.time[i]).nanoseconds / 1e9 > 1):
      self.node_name.pop(i)
      self.line.pop(i)
      self.msg.pop(i)
      self.time.pop(i)
      self.time_rcv.pop(i)
      return True
    elif ( self.time[i] == -1):
      self.time[i] = tnow
      return False
    return False

  # Print all the error messages to a string and deletes everything
  def __str__(self):
    if len(self.node_name)==0:
      return ""
    # self.__clear__()
    text="\t<ROSMessage name=\"Errors\">"
    # do the range from the end to the beginning
    for i in range(len(self.node_name)-1,-1,-1):
      if self.__clearitem__(i):
        continue
      text += "\t\t<VAR>\n\t\t\t<NODE>"+self.node_name[i]+"</NODE>\n"
      text += "\t\t\t<TIME>"+"%.2f"%(self.time_rcv[i].nanoseconds / 1e9)+"</TIME>\n"
      text += "\t\t\t<MSG>"+self.msg[i]+"</MSG>\n\t\t</VAR>"
    text+="</ROSMessage>"
    #print text
    return text
  
  # Add an error message to the list
  def add_error(self, topic_name, data):
    if topic_name.endswith("rosout"):
      if(int(data.level)>=8): # Only Error and Fatal messages
        for i in range(len(self.node_name)):
          if(data.name==self.node_name and data.line==self.line):
            return
        self.node_name.append(data.name)
        self.msg.append(data.msg)
        self.line.append(data.line)
        self.time.append(-1)
        self.time_rcv.append(Clock().now())

ALL_ERRORs = ERROR_Struct()
     
# ROS Sleep
def _sleep(duration):
  #rospy.rostime.wallsleep(duration)
  time.sleep(duration)
  # rclpy.sleep(Duration(seconds=duration))


def get_topic_class(topic, console_server: Node):
  """
  Get the topic message class
  :returns: message class for topic, real topic
    name, and function for evaluating message objects into the subtopic
    (or ``None``). ``(Message, str, str)``
  :raises: :exc:`ROSTopicException` If topic type cannot be determined or loaded
  """
  # Get list of all topics and their types
  pubs = console_server.get_topic_names_and_types()

  # console_server.get_logger().info("PLSSIR: " + str(pubs))

  # search for msg class for specific topic
  for topic_name, class_type in pubs:
    if topic_name == topic:
      msg_class = class_type
      return msg_class
  
  return None

def get_topic_type(topic, blocking=False):
  """
  Get the topic type.

  :param topic: topic name, ``str``
  :param blocking: (default False) block until topic becomes available, ``bool``
  
  :returns: topic type, real topic name and fn to evaluate the message instance
    if the topic points to a field within a topic, e.g. /rosout/msg. fn is None otherwise. ``(str, str, fn)``
  :raises: :exc:`ROSTopicException` If master cannot be contacted
  """
  topic_type, real_topic, msg_eval = _get_topic_type(topic)
  if topic_type:
    return topic_type, real_topic, msg_eval
  elif blocking:
    sys.stderr.write("WARNING: topic [%s] does not appear to be published yet\n"%topic)
    while rclpy.ok():
      topic_type, real_topic, msg_eval = _get_topic_type(topic)
      if topic_type:
        return topic_type, real_topic, msg_eval
      else:
        _sleep(0.1)
  return None, None, None

def _get_topic_type(topic):
  """
  subroutine for getting the topic type
  :returns: topic type, real topic name and fn to evaluate the message instance
  if the topic points to a field within a topic, e.g. /rosout/msg, ``(str, str, fn)``
  """
  try:
    val = _master_get_topic_types(rosgraph.Master('/rostopic'))
  except socket.error:
    raise ROSTopicIOException("Unable to communicate with master!")

  # exact match first, followed by prefix match
  matches = [(t, t_type) for t, t_type in val if t == topic]
  if not matches:
    matches = [(t, t_type) for t, t_type in val if topic.startswith(t+'/')]
    # choose longest match
    matches.sort(key=itemgetter(0), reverse=True)
  if matches:
    t, t_type = matches[0]
    if t_type == rosgraph.names.ANYTYPE:
      return None, None, None
    return t_type, t, msgevalgen(topic[len(t):])
  else:
    return None, None, None

# NOTE: this is used externally by rxplot

def _master_get_topic_types(master):
  try:
    val = master.getTopicTypes()
  except xmlrpclib.Fault:
    #TODO: remove, this is for 1.1
    sys.stderr.write("WARNING: rostopic is being used against an older version of ROS/roscore\n")
    val = master.getPublishedTopics('/')
  return val

def _rostopic_cmd_echo(argv):
  def expr_eval(expr):
    def eval_fn(m):
      return eval(expr)
    return eval_fn

  args = argv[2:]
  if len(args) == 0:
    sys.stderr.write("You have to mention the topic\n")
  elif len(args) == 1:
    topic = rosgraph.names.script_resolve_name('rostopic', args[0])
    # suppressing output to keep it clean
    #if not options.plot:
    #  print "rostopic: topic is [%s]"%topic
      
    filter_fn = None
    
    
    # callback_echo = CallbackEcho(topic);""", None, False, None, False, False, False, None, None)"""
    try:
      # _rostopic_echo(topic, callback_echo)
      pass
    except socket.error:
      sys.stderr.write("Network communication failed. Most likely failed to communicate with master.\n")

def msgevalgen(pattern):
  """
  Generates a function that returns the relevant field (aka 'subtopic') of a Message object
  :param pattern: subtopic, e.g. /x. Must have a leading '/' if specified, ``str``
  :returns: function that converts a message into the desired value, ``fn(Message) -> value``
  """
  if not pattern or pattern == '/':
    return None
  def msgeval(msg):
    # I will probably replace this with some less beautiful but more efficient
    try:
      return eval('msg'+'.'.join(pattern.split('/')))
    except AttributeError as e:
      sys.stdout.write("no field named [%s]"%pattern+"\n")
      return None
  return msgeval

# def _rostopic_echo(topic, callback_echo):
#   """
#   Print new messages on topic to screen.
  
#   :param topic: topic name, ``str``
#   :param bag_file: name of bag file to echo messages from or ``None``, ``str``
#   """

#   rospy.init_node(NAME, anonymous=True)
#   msg_class, real_topic, msg_eval = get_topic_class(topic, blocking=True)
#   if msg_class is None:
#     # occurs on ctrl-C
#     return
#   callback_echo.msg_eval = msg_eval

#   use_sim_time = rospy.get_param('/use_sim_time', False)
#   sub = rospy.Subscriber(real_topic, msg_class, callback_echo.callback, topic)

#   if use_sim_time:
#     # #2950: print warning if nothing received for two seconds

#     timeout_t = time.time() + 2.
#     while time.time() < timeout_t and \
#         callback_echo.count == 0 and \
#         rclpy.ok() and \
#         not callback_echo.done:
#       _sleep(0.1)

#     if callback_echo.count == 0 and \
#         rclpy.ok() and \
#         not callback_echo.done:
#       sys.stderr.write("WARNING: no messages received and simulated time is active.\nIs /clock being published?\n")

#   while rclpy.ok() and not callback_echo.done:
#     _sleep(0.1)
 
class CallbackEcho(object):
  """
  Callback instance that can print callback data in a variety of
  formats. Used for all variants of rostopic echo
  """

  def __init__(self, topic, console_server: Node):
    """
    :param plot: if ``True``, echo in plotting-friendly format, ``bool``
    :param filter_fn: function that evaluates to ``True`` if message is to be echo'd, ``fn(topic, msg)``
    :param echo_all_topics: (optional) if ``True``, echo all messages in bag, ``bool``
    :param offset_time: (optional) if ``True``, display time as offset from current time, ``bool``
    :param count: number of messages to echo, ``None`` for infinite, ``int``
    :param field_filter_fn: filter the fields that are strified for Messages, ``fn(Message)->iter(str)``
    """
    # remove "/" in the end if exists
    if topic and topic[-1] == '/':
      topic = topic[:-1]
    self.topic = topic
    #self.msg_eval = msg_eval
    #self.plot = plot
    #self.filter_fn = filter_fn
    self.count = 0
    self.prefix = ''
    self.suffix = '\n---' #if not plot else ''# same as YAML document separator, bug #3291
    
    #self.echo_all_topics = echo_all_topics
    #self.offset_time = offset_time

    # done tracks when we've exceeded the count
    self.done = False

    #if echo_clear:
    self.prefix = '\033[2J\033[;H'

    #self.field_filter=field_filter_fn
    
    # first tracks whether or not we've printed anything yet. Need this for printing plot fields.
    self.first = True

    self.console_server = console_server

    # cache
    #self.last_topic = None
    #self.last_msg_eval = None

  def callback(self, data, topic):
    """
    Callback to pass to rclpy.create_subscription or to call
    manually. rclpy.create_subscription constructor must also pass in the
    topic name as an additional arg
    :param data: Message
    :param topic: topic name, ``str``
    """

    try:
      if topic == self.topic:
        pass
      elif self.topic.startswith(topic + '/'):
        # self.topic is actually a reference to topic field, generate msgeval
        # generate msg_eval and cache
        self.last_msg_eval = msg_eval = msgevalgen(self.topic[len(topic):])
        self.last_topic = topic
      elif not self.echo_all_topics:
        return
      msg_eval = None
      
      # if msg_eval is a function returned by msgevalgen, get attribute from topic
      if msg_eval is not None:
        data = msg_eval(data)
      else:
        val = data
        
      # data can be None if msg_eval returns None
      if data is not None:
        # NOTE: we do all prints using direct writes to sys.stdout, which works better with piping
        
        self.count += 1
        
        if topic.endswith('rosout'):
          ALL_ERRORs.add_error(topic, data)
        else:
          ALL_TOPICS.add_value(topic, data)
          # self.console_server.get_logger().info("ADDED " + str(topic))
        
        
        #sys.stdout.write(self.prefix+\
        #        ros2xml(data, topic[1:],0) + \
        #        self.suffix + '\n')
        # we have to flush in order before piping to work
        sys.stdout.flush()
      # #2778 : have to check count after incr to set done flag
      #if self.max_count is not None and self.count >= self.max_count:
      #  self.done = True

    except IOError:
      self.done = True
    except:
      # set done flag so we exit
      self.done = True
      traceback.print_exc()

def ros2xml(msg, name, depth=0):
  xml = ""
  tabs = "\t"*depth

  if hasattr(msg, "__slots__"):
    type = msg._type
    xml = xml + tabs + "<" + name.replace("/", "_") + " type=\"" + type + "\">\n"
    
    try:
      for slot in msg.__slots__:
        xml = xml + ros2xml(getattr(msg, slot), slot, depth=depth+1)
        
    except:
      xml = xml + tabs + str(msg)
    xml = xml + tabs + "</" + name.replace("/", "_") + ">\n"
  else:
    xml = xml + tabs + "<" + name.replace("/", "_") + ">" + str(msg) + "</" + name.replace("/", "_") + ">\n"
  
  return xml
  
def moos2xml(msg, name, time, depth=0):
  xml =""
  tabs = "\t"*depth

  if hasattr(msg, "__slots__"):
    name = name.replace("/", "_")
    if (name.startswith("_")):
      name = name[1:]

    xml = xml + tabs + "<ROSMessage name=\"" + name + "\">\n"
    try:
      for slot in msg.__slots__:
        if slot!="stamp":# and slot!="seq":
          xml = xml + moos2xml(getattr(msg, slot), slot, time, depth=depth+1)
    except:
      pass
    xml = xml + tabs + "</ROSMessage>\n"
  else:
    if name!= "stamp":# and name!= "seq":
      xml = xml + tabs + "<VAR>\n"+ tabs + "\t<KEY>"+ name.replace("/", "_") + "</KEY>\n"
      xml = xml + tabs + "<TIME>\n"+ str(time.nanoseconds / 1e9) + "</TIME>\n"
      xml = xml + tabs + "\t<DOUBLE>"+ str(msg) + "</DOUBLE>\n"+ tabs + "</VAR>\n"

  return parseXMLStateConsoleAttributesFromROS2NamingToROS1(xml)

def _fullusage():
  print("""rostopic is a command-line tool for printing information about ROS Topics.

Commands:
\tconsole echo\tprint messages to screen
\tconsole list\tlist active topics
\tconsole console\tstarts the http server 

Type rostopic <command> -h for more detailed usage, e.g. 'rostopic echo -h'
""")
  sys.exit(getattr(os, 'EX_USAGE', 1))

# ChatGPT-san helped here
def get_message_class(type_str: str):
  """
  ROS 2 equivalent of roslib.message.get_message_class.
  
  Args:
    type_str (str): Message type string, e.g. "std_msgs/String" or "geometry_msgs/msg/Twist".
  
  Returns:
    type: The message class (Python type).
  
  Raises:
    ValueError, ImportError, AttributeError if the type cannot be loaded.
  """
  # Normalize type string
  type_str = type_str.replace('/', '.')
  
  # Handle ROS 1 style "pkg/Msg" → "pkg.msg.Msg"
  parts = type_str.split('.')
  
  # Example 1: std_msgs/String → std_msgs.msg.String
  # Example 2: geometry_msgs.msg.Twist → geometry_msgs.msg.Twist
  if len(parts) == 2:
    pkg, msg = parts
    module_name = f"{pkg}.msg"
    class_name = msg
  elif len(parts) == 3 and parts[1] == "msg":
    module_name = f"{parts[0]}.msg"
    class_name = parts[2]
  else:
    raise ValueError(f"Invalid message type string: {type_str}")
  
  module = importlib.import_module(module_name)
  return getattr(module, class_name), module_name+"."+class_name

def safe_assign(msg, field_name, value):
    # If assigning to a string field but value is not a string → convert
    field_type = type(getattr(msg, field_name))
    if field_type == str and not isinstance(value, str):
        value = str(value)

    # If assigning to a float/int field from string → convert
    elif isinstance(getattr(msg, field_name), (float, int)) and isinstance(value, str):
        try:
            value = float(value) if '.' in value else int(value)
        except ValueError:
            pass  # leave as-is

    setattr(msg, field_name, value)

def fill_message(msg, args, console_server: Node):
  """
  ROS 2 equivalent of genpy.message.fill_message_args(msg, args).
  
  Args:
    msg: ROS 2 message instance (e.g. std_msgs.msg.String()).
    args: dict, list, or primitive value to fill into msg.
      - dict: assigns by field name.
      - list/tuple: fills fields in order.
      - scalar: assigns directly if msg has only one field.
  """
  # If args is a dict — assign by field names
  if isinstance(args, Mapping):
    for key, value in args.items():
      if hasattr(msg, key):
        field = getattr(msg, key)
        if hasattr(field, '__slots__'):  # Nested message
          fill_message(field, value, console_server)
        else:
          safe_assign(msg, key, value)
    return msg

  # If args is a list or tuple — assign fields by order
  elif isinstance(args, Sequence) and not isinstance(args, (str, bytes)):
    fields = msg.__slots__

    try:
      fields.remove("_check_fields")
    except:
      pass

    for i, val in enumerate(args):
      if i >= len(fields):
        break
      field_name = fields[i][1:] if fields[i].startswith('_') else fields[i]
      field = getattr(msg, field_name)
      if hasattr(field, '__slots__'):  # nested message
        fill_message(field, val, console_server)
      else:
        # safe_assign(msg, field_name, val)
        setattr(msg, field_name, val[field_name])
    return msg

  # If scalar — assign directly if possible
  else:
    
    fields = msg.__slots__
    fields.remove("_check_fields")

    if len(fields) == 1:
      field_name = fields[0][1:] if fields[0].startswith('_') else fields[0]
      safe_assign(msg, field_name, args)
    else:
      raise ValueError(f"Cannot assign scalar to multi-field message: {msg}")
    return msg

def _publish_latched(pub, msg):
  """
  Publish and latch message. Subroutine of L{publish_message()}.
  
  :param pub: :class:`rospy.Publisher` instance for topic
  :param msg: message instance to publish
  :param once: if ``True``, publish message once and then exit after sleep interval, ``bool``
  :param verbose: If ``True``, print more verbose output to stdout, ``bool``
  """
  try:
    pub.publish(msg)
  except Exception as e:
    console_server.get_logger().warn(str(e))

def publish_message(pub, msg_class, pub_args, console_server: Node):
  """
  Create new instance of msg_class, populate with pub_args, and publish. This may
  print output to screen.
  
  :param pub: :class:`rclpy.publisher.Publisher` instance for topic
  :param msg_class: Message type, ``Class``
  :param pub_args: Arguments to initialize message that is published, ``[val]``
  """

  msg = msg_class()
  try:
    msg = fill_message(msg, pub_args, console_server)
  except Exception as e:
    console_server.get_logger().warn(str(e))
    return 0
  
  try:
    _publish_latched(pub, msg)
  except Exception as e:
    console_server.get_logger().warn("Unable to publish message. One of the fields has an incorrect type\n" + str(e))
    return 0

def cmd_set_topic(args, console_server: Node):
  args=args[5:]
  args = args.replace("%27", "")
  args = args.replace("%7B", "{")
  args = args.replace("%7D", "}")
  args = args.replace("%7C", "|")
  args = args.replace("%20", " ")
  args = args.replace("%22", "\"")
  args = args.replace("%23", "#")
  args = args.replace("%09", "\n")
  # args = args.replace("%09", "\r\n")
  args = args.replace("%0D", "\n")
  args = args.replace("%3C", "<")
  args = args.replace("%3E", ">")
  topics = args.split("|")

  if (topics[0].strip().split(' ')[0] == 'Mission_String'):
    print("debug: {}".format(args))

  try:
    topics.remove('')
  except:
    pass
    
  #print("topic: %s"%(topics))
  for k in topics:
    set_args = k.split(" ")
    
    try:
      set_args.remove('')
    except:
      pass 
    try:
      # EXAMPLES FOR THE FUTURE
      # magicelectric0/addons/Mission_String std_msgs/String {data: \"3\n\n481844.386 4282309.864 29S\n\nLINE 258.23 131.86 -226.06 11.69 0.30 -1 \"}
      # /magicelectric0/controls/send_wp_standard geometry_msgs/PointStamped {point: {x: 481439.5371431402, y: 4282384.428882044}}
      topic_name = set_args[0]
      topic_type = set_args[1]

      # Important because msg and srv types have different syntax in ROS2!!
      msg_class, msg_name = get_message_class(topic_type)

      args=''
      for i in set_args[2:]:
        #print("ARG: %s"%(args))
        args += i + ' '

      if(args==''):
        continue
    except:
      continue

    try:
      pub_args = []
      pub_args.append(yaml.safe_load(args))
    except Exception as e:
      console_server.get_logger().warn("Argument error: %s"% str(e))
      return 0
    
    # print("Publishing topic: %s type: %s value: %s"%(topic_name, topic_type, pub_args))

    if(console_server.get_parameter('addons.console_server.topics.console.waypoint').get_parameter_value().string_value in topic_name):
      try:
        actual_service_topic = console_server.get_parameter('addons.console_server.topics.services.wp_standard').get_parameter_value().string_value

        send_wp_standard = console_server.create_client(SendWpType1, actual_service_topic)

        # create request
        req = SendWpType1.Request()
        req.x = float(pub_args[0]['point']['y'])
        req.y = float(pub_args[0]['point']['x'])
        req.yaw = float(0)
        
        # call client service
        send_wp_standard.call_async(req)
        # console_server.get_logger().warn("top!")
        return 0
      except:
        console_server.get_logger().warn("WP service call failed")
    else:
      # Add / to start of topic name if it doesn't have it
      if (not topic_name.startswith("/")):
        topic_name = "/" + topic_name

      pub = console_server.create_publisher(msg_class, topic_name, 1)

      try:
        publish_message(pub, msg_class, pub_args, console_server)
      except Exception as e:
        console_server.get_logger().warn(str(e))
        return 0

  # console_server.get_logger().warn("EYO end?")

  return 1
  
# def _list_topics(print_out = True, console_server = None):
#   # def topic_type(t, topic_types):
#   #   matches = [t_type for t_name, t_type in topic_types if t_name == t]
#   #   if matches:
#   #     return matches[0]
#   #   return 'unknown type'
    
#   try:
#     # Get list of all topics and their types
#     topics, types = console_server.get_topic_names_and_types()

#     console_server.get_logger().info("TOPICsssssss: " + str(topics))

#     #we want only the published topics
#     # topic_types = _master_get_topic_types(master)
#     if print_out:
#       print("\nPrinting published topics NOT Migrated to ROS2")
#       # print("\nPublished topics:")
#       # for t, l in pubs:
#       #   if len(l) > 1:
#       #     print(" * %s [%s] %s publishers"%(t, topic_type(t, topic_types), len(l)))
#       #   else:
#       #     print(" * %s [%s] 1 publisher"%(t, topic_type(t, topic_types)))
#   except Exception as e:
#     # print("_list_topics: " + str(e))
#     return []
    
#   return topics

def _Resume_Page():
  #print pages_folder + "index.html"
  f = open(pages_folder + "index.html")
  str = f.read()
  #str.replace("IPTOBEEDITED", "")
  #print str
  f.close()
  return str
     

# global g_list_topic_thrsh
# g_list_topic_thrsh = 30
# global g_list_topic_stamp 
# g_list_topic_stamp = 0

# I hope no one has to look at this ever
# But I had no time to do better
# I was leaving
# I am currently chained to the pc until I finish this on a Saturday night
# Sorry future generations
# ass: ebil
# PS: ROS2 made it so attributes in a message need to be lower case
# so I had to change the message for the console state
# In order to keep the message parsable on the side of the web console,
# this function was created.
def parseXMLStateConsoleAttributesFromROS2NamingToROS1(msg):
  conversion = {
    "gps_good" : "GPS_Good",
    "imu_good" : "IMU_Good",
    "depth" : "Depth",
    "x" : "X",
    "y" : "Y",
    "z" : "Z",
    "vx" : "Vx",
    "vy" : "Vy",
    "vz" : "Vz",
    "surge" : "u",
    "yaw" : "Yaw",
    "pitch" : "Pitch",
    "roll" : "Roll",
    "yaw_rate" : "Yaw_rate",
    "pitch_rate" : "Pitch_rate",
    "roll_rate" : "Roll_rate",
    "in_pressure" : "In_Press",
    "in_pressure_dot" : "In_Press_dot",
    "battery_level" : "battery_level",
    "altitude" : "altitude"
  }

  for key, value in conversion.items():
    msg = msg.replace("<KEY>_" + key + "</KEY>", "<KEY>" + value + "</KEY>")

  return msg

def fix_ros_header_and_end(xml_string):
  pattern = r'(<ROSMessage name="header">.*?<ROSMessage name="stamp">.*?</ROSMessage>.*?</ROSMessage>)'

  new_header = """<ROSMessage name="header">
		<VAR>
			<KEY>seq</KEY>
		<TIME>
1761433187.4541357</TIME>
			<DOUBLE>1550</DOUBLE>
		</VAR>
		<VAR>
			<KEY>frame_id</KEY>
		<TIME>
1761433187.4541357</TIME>
			<DOUBLE>magicelectric0_base_link</DOUBLE>
		</VAR>
	</ROSMessage>"""

  first = re.sub(pattern, new_header, xml_string, flags=re.DOTALL)

  pattern2 = r'(<VAR>\n\t\t<KEY>_check_fields</KEY>.*?<TIME>.*?</TIME>.*?<DOUBLE>.*?</DOUBLE>.*?</VAR>\n)'

  end = ""

  second = re.sub(pattern2, end, first, flags=re.DOTALL)

  third = re.sub(r'\t</ROSMessage>$', "</ROSMessage>", second, flags=re.DOTALL)

  return third.replace("_status", "status")

# Handles the HTTP Requests for a list of topics
def _Ask_Topics(topics_list, console_server: Node):
  # global g_list_topic_stamp
  # global g_list_topic_thrsh
  topics = topics_list.replace("%20", " ").split(" ")

  msg = ""

  for i in topics[1:]: # aka don't iterate over "VAR", in index 0
    # Add / to start of topic name if it doesn't have it
    if (not i.startswith("/")):
      i = "/" + i
    
    # get the information if the topic is not new
    if ALL_TOPICS.count(i)>0:
      # console_server.get_logger().info("GET TOPIC: " + str(i))
      #print("Encontrou %s %s"%(i, msg))
      name=i
      [topic_data, topic_time] = ALL_TOPICS.data_topic(i)
      # console_server.get_logger().info("DATA from " + str(i) + ": " + str(type(topic_data)))
      if(topic_time != 0):
        out = moos2xml(topic_data, name, topic_time, 0)
        msg += fix_ros_header_and_end(out)
        # console_server.get_logger().info("MSG from " + str(i) + ": " + msg)
      continue

    try:
      if (SUBSCRIBED_TOPICS.count(i)==0): # Register for the new topics
        #print("Subscribing to topic %s"%topic)
        # console_server.get_logger().info("NEW TOPIC: " + str(i))
        subscriber_Thread(i, console_server).start()
    except:
      if(UNKNOWN_TOPICS.count(i)==0 and SUBSCRIBED_TOPICS.count(i)==0):
        UNKNOWN_TOPICS.add_value(i, "")
  
  # include the errors in the message
  msg += ALL_ERRORs.__str__()

#   msg="""<ROSMessage name="magicelectric0_State">
# 	<ROSMessage name="header">\n
# 		<VAR>
# 			<KEY>seq</KEY>
# 		<TIME>
# 1761433187.4541357</TIME>
# 			<DOUBLE>1550</DOUBLE>
# 		</VAR>
# 		<VAR>
# 			<KEY>frame_id</KEY>
# 		<TIME>
# 1761433187.4541357</TIME>
# 			<DOUBLE>magicelectric0_base_link</DOUBLE>
# 		</VAR>
# 	</ROSMessage>
# 	<VAR>
# 		<KEY>GPS_Good</KEY>
# 	<TIME>
# 1761438722.086214</TIME>
# 		<DOUBLE>0</DOUBLE>
# 	</VAR>
# 	<VAR>
# 		<KEY>IMU_Good</KEY>
# 	<TIME>
# 1761438722.086214</TIME>
# 		<DOUBLE>0</DOUBLE>
# 	</VAR>
# 	<VAR>
# 		<KEY>Depth</KEY>
# 	<TIME>
# 1761438722.086214</TIME>
# 		<DOUBLE>0.0</DOUBLE>
# 	</VAR>
# 	<VAR>
# 		<KEY>X</KEY>
# 	<TIME>
# 1761438722.086214</TIME>
# 		<DOUBLE>481439.5371431402</DOUBLE>
# 	</VAR>
# 	<VAR>
# 		<KEY>Y</KEY>
# 	<TIME>
# 1761438722.086214</TIME>
# 		<DOUBLE>4282384.428882044</DOUBLE>
# 	</VAR>
# 	<VAR>
# 		<KEY>Z</KEY>
# 	<TIME>
# 1761438722.086214</TIME>
# 		<DOUBLE>0.0</DOUBLE>
# 	</VAR>
# 	<VAR>
# 		<KEY>Vx</KEY>
# 	<TIME>
# 1761438722.086214</TIME>
# 		<DOUBLE>0.0</DOUBLE>
# 	</VAR>
# 	<VAR>
# 		<KEY>Vy</KEY>
# 	<TIME>
# 1761438722.086214</TIME>
# 		<DOUBLE>0.0</DOUBLE>
# 	</VAR>
# 	<VAR>
# 		<KEY>Vz</KEY>
# 	<TIME>
# 1761438722.086214</TIME>
# 		<DOUBLE>0.0</DOUBLE>
# 	</VAR>
# 	<VAR>
# 		<KEY>u</KEY>
# 	<TIME>
# 1761438722.086214</TIME>
# 		<DOUBLE>0.0</DOUBLE>
# 	</VAR>
# 	<VAR>
# 		<KEY>Yaw</KEY>
# 	<TIME>
# 1761438722.086214</TIME>
# 		<DOUBLE>0.0</DOUBLE>
# 	</VAR>
# 	<VAR>
# 		<KEY>Pitch</KEY>
# 	<TIME>
# 1761438722.086214</TIME>
# 		<DOUBLE>0.0</DOUBLE>
# 	</VAR>
# 	<VAR>
# 		<KEY>Roll</KEY>
# 	<TIME>
# 1761438722.086214</TIME>
# 		<DOUBLE>0.0</DOUBLE>
# 	</VAR>
# 	<VAR>
# 		<KEY>Yaw_rate</KEY>
# 	<TIME>
# 1761438722.086214</TIME>
# 		<DOUBLE>0.0</DOUBLE>
# 	</VAR>
# 	<VAR>
# 		<KEY>Pitch_rate</KEY>
# 	<TIME>
# 1761438722.086214</TIME>
# 		<DOUBLE>0.0</DOUBLE>
# 	</VAR>
# 	<VAR>
# 		<KEY>Roll_rate</KEY>
# 	<TIME>
# 1761438722.086214</TIME>
# 		<DOUBLE>0.0</DOUBLE>
# 	</VAR>
# 	<VAR>
# 		<KEY>In_Press</KEY>
# 	<TIME>
# 1761438722.086214</TIME>
# 		<DOUBLE>0.0</DOUBLE>
# 	</VAR>
# 	<VAR>
# 		<KEY>In_Press_dot</KEY>
# 	<TIME>
# 1761438722.086214</TIME>
# 		<DOUBLE>0.0</DOUBLE>
# 	</VAR>
# 	<VAR>
# 		<KEY>battery_level</KEY>
# 	<TIME>
# 1761438722.086214</TIME>
# 		<DOUBLE>0</DOUBLE>
# 	</VAR>
# 	<VAR>
# 		<KEY>altitude</KEY>
# 	<TIME>
# 1761438722.086214</TIME>
# 		<DOUBLE>0.0</DOUBLE>
# 	</VAR>
# 	<VAR>
# 		<KEY>status</KEY>
# 	<TIME>
# 1761438722.086214</TIME>
# 		<DOUBLE>0</DOUBLE>
# 	</VAR>
# </ROSMessage>"""

  # console_server.get_logger().info("BRUH: " + msg)
  
  #print msg;
  if msg == "":
    return "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<content>\n\n</content>\n\n"
  else:
    return "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<content>\n" + msg + "\n</content>\n\n"

def getfiles(dirpath):
  a = [s for s in os.listdir(dirpath)
     if os.path.isfile(os.path.join(dirpath, s))]
  return a

# Gerenerates an html code with all the missions in the mission FOLDER
def _List_Missions():
  # version 1 alphabetic sort
  missions = os.listdir(MISSION_PATH) # Directory Listing
  missions = sorted(missions, key=str.lower,reverse=True)
  # version 2 modified sort
  # missions = sorted(os.listdir(MISSION_PATH),key=lambda x: os.stat(os.path.join(MISSION_PATH,x)).st_mtime, reverse=True)
  # version 3 modified sort
  # missions=getfiles(MISSION_PATH)
  # missions = sorted(missions, key=lambda x: os.path.getmtime(os.path.join(MISSION_PATH,x)), reverse=True)
  res =   "<html>"+\
      "<head>"+\
      "   <meta charset=\"utf-8\">"+\
      "   <meta http-equiv=\"Content-Type\" content=\"text/html; charset=utf-8\">"+\
      "   <style>"+\
      "       * { margin: 0; padding: 0; }"+\
      "       html { height: 90%; }"+\
      "       body {font-size: 62.5%; font-family: Arial, Tahoma, sans-serif; color: #343434; padding-bottom: 0px; }"+\
      "       p { font-size: 1.3em; line-height: 1.5em; margin-bottom: 10px; }"+\
      "       a { text-decoration: none; }"+\
      "       img { border: 0; }"+\
      "       /** Navigation list **/"+\
      "       ul#nlists { list-style: none; font-family: \"Calibri\", Arial, sans-serif; }"+\
      "       ul#nlists li { "+\
      "         border-bottom: 1px solid #d9e2eb; "+\
      "         background: #fff; "+\
      "         display: block;"+\
      "         background-image: -webkit-gradient(linear, 0% 0%, 0% 100%, from(#ffffff) to(#ecf1f5));"+\
      "         background-image: -webkit-linear-gradient(top, #ffffff, #ecf1f5);"+\
      "         background-image:  -moz-linear-gradient(top, #ffffff, #ecf1f5);"+\
      "         background-image:    -o-linear-gradient(top, #ffffff, #ecf1f5);"+\
      "         background-image:     linear-gradient(top, #ffffff, #ecf1f5); "+\
      "       }"+\
      "       ul#nlists li a { "+\
      "         position: relative; "+\
      "         display: block;"+\
      "         box-sizing: border-box;"+\
      "         width: 100%;"+\
      "         padding: 15px 0px; "+\
      "         padding-left: 10px;"+\
      "         text-decoration: none; "+\
      "         font-size: 1.4em; "+\
      "         color: #787878;"+\
      "         font-weight: bold;"+\
      "         overflow: hidden;"+\
      "       }"+\
      "       ul#nlists li a:hover { color: #999; }"+\
      "       ul#nlists li a:hover::after { border-color: #c3ca80; }"+\
      "       #dropMission {"+\
      "         position: relative;"+\
      "         border: 1px solid black;"+\
      "         text-align: center;"+\
      "         vertical-align: middle;"+\
      "         display: inline-block;"+\
      "         width:69%;"+\
      "         height: 40px;"+\
      "         background: #DDDDDD;"+\
      "         -webkit-border-radius: 3px; -moz-border-radius: 3px; border-radius: 3px;"+\
      "       }"+\
      "       #dropMission input {"+\
      "         position: absolute;"+\
      "         vertical-align: middle;"+\
      "         width: 100%;"+\
      "         height: 100%;"+\
      "         top: 0;"+\
      "         left: 0;"+\
      "         opacity: 0;"+\
      "       }"+\
      "   </style>"+\
      "</head>"+\
      "<body>"+\
      "<form action=\"/\" enctype=\"multipart/form-data\" method=\"post\">"+\
      "   <input type=\"submit\" id=\"btUpload\" value='Upload' style=\"height: 40px; width: 20%; display: inline-block; \" >"+\
      "   <div id=\"dropMission\">"+\
      "     <p id=\"Mission_p\" style=\"font-weight: bold\">Drop Mission</p>"+\
      "     <input type=\"file\"  class=\"file\" id=\"MissionINP\" name=\"upfile\"/>"+\
      "   </div>"+\
      "</form>" +\
      "   <ul id=\"nlists\">"
  #print("MISSSIONS = %s"%missions)
  for f in missions:
    if f.endswith(".txt"):
      #print("Mission = [%s]"%(f));
      # Creating the list
      res += "<li><a href=\"/RSET Mission_Filename std_msgs/String "+MISSION_PATH+"/"+f+"\">"+f+"</a></li>\n"
  res += "   </ul>"+\
      "</body>"+\
      "</html>"
      
  return res
  
# One Thread for each subscription, it should take one of the mutexs
class subscriber_Thread( threading.Thread):
  def __init__(self, topic, console_server: Node):
    self.topic = topic
    self.console_server = console_server
    self.callback_echo = CallbackEcho(self.topic, self.console_server)
    # self.console_server.get_logger().info("SUB THREAD -> TOPIC: " + str(self.topic))

    threading.Thread.__init__(self)
      
  def run(self):
    try:
      self.msg_class = get_topic_class(self.topic, self.console_server)

      if self.msg_class is None:
        return None

      if self.msg_class is not None:
        actual_ros_class, msg_name = get_message_class(self.msg_class[0])
        # self.console_server.get_logger().info("BRO MSG CLASS for " + str(self.topic) + ": " + str(type(actual_ros_class)))
        
        sub = self.console_server.create_subscription(actual_ros_class, self.topic, partial(self.callback_echo.callback, topic=self.topic), 1)
        SUBSCRIBED_TOPICS.add_value(self.topic, "")
        UNKNOWN_TOPICS.del_value(self.topic)
    except:
      return None

    while rclpy.ok() and not self.callback_echo.done:
      _sleep(0.1)
      # self.console_server.get_logger().info("BRO is alive: " + str(self.topic))

def populenv(self):
    path = self.path
    dir, rest = '.', 'ciao'

    # find an explicit query string, if present.
    i = rest.rfind('?')
    if i >= 0:
      rest, query = rest[:i], rest[i+1:]
    else:
      query = ''

    # dissect the part after the directory name into a script name &
    # a possible additional path, to be stored in PATH_INFO.
    i = rest.find('/')
    if i >= 0:
      script, rest = rest[:i], rest[i:]
    else:
      script, rest = rest, ''

    # Reference: http://hoohoo.ncsa.uiuc.edu/cgi/env.html
    # XXX Much of the following could be prepared ahead of time!
    env = {}
    env['SERVER_SOFTWARE'] = self.version_string()
    env['SERVER_NAME'] = self.server.server_name
    env['GATEWAY_INTERFACE'] = 'CGI/1.1'
    env['SERVER_PROTOCOL'] = self.protocol_version
    env['SERVER_PORT'] = str(self.server.server_port)
    env['REQUEST_METHOD'] = self.command
    uqrest = urllib.unquote(rest)
    env['PATH_INFO'] = uqrest
    env['SCRIPT_NAME'] = 'ciao'
    if query:
      env['QUERY_STRING'] = query
    host = self.address_string()
    if host != self.client_address[0]:
      env['REMOTE_HOST'] = host
    env['REMOTE_ADDR'] = self.client_address[0]
    authorization = self.headers.getheader("authorization")
    if authorization:
      authorization = authorization.split()
      if len(authorization) == 2:
        import base64, binascii
        env['AUTH_TYPE'] = authorization[0]
        if authorization[0].lower() == "basic":
          try:
            authorization = base64.decodestring(authorization[1])
          except binascii.Error:
            pass
          else:
            authorization = authorization.split(':')
            if len(authorization) == 2:
              env['REMOTE_USER'] = authorization[0]
    # XXX REMOTE_IDENT
    if self.headers.typeheader is None:
      env['CONTENT_TYPE'] = self.headers.type
    else:
      env['CONTENT_TYPE'] = self.headers.typeheader
    length = self.headers.getheader('content-length')
    if length:
      env['CONTENT_LENGTH'] = length
    referer = self.headers.getheader('referer')
    if referer:
      env['HTTP_REFERER'] = referer
    accept = []
    for line in self.headers.getallmatchingheaders('accept'):
      if line[:1] in "\t\n\r ":
        accept.append(line.strip())
      else:
        accept = accept + line[7:].split(',')
    env['HTTP_ACCEPT'] = ','.join(accept)
    ua = self.headers.getheader('user-agent')
    if ua:
      env['HTTP_USER_AGENT'] = ua
    co = filter(None, self.headers.getheaders('cookie'))
    if co:
      env['HTTP_COOKIE'] = ', '.join(co)
    # XXX Other HTTP_* headers
    # Since we're setting the env in the parent, provide empty
    # values to override previously set values
    for k in ('QUERY_STRING', 'REMOTE_HOST', 'CONTENT_LENGTH',
          'HTTP_USER_AGENT', 'HTTP_COOKIE', 'HTTP_REFERER'):
      env.setdefault(k, "")
    os.environ.update(env)


# Handles the HTTP Requests calling the specific functions
class HTTP_Handler(BaseHTTPRequestHandler):
  console_server = None

  def do_GET(self):
    try:
      #print("One Connection [GET %s]\n"% self.path)
      if self.path.strip()=="/" or self.path.startswith("/index"): 
        self.send_response(200)
        self.send_header('Content-type',  'text/html')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('cache-control', 'private, s-maxage=0, max-age=0, must-revalidate') #'cache-control': 'private, s-maxage=0, max-age=0, must-revalidate'
        self.end_headers()
        self.wfile.write(_Resume_Page().encode())

      elif self.path.endswith(".js"):
        f = open(pages_folder + self.path)  
        self.send_response(200)
        self.send_header('Content-type', 'application/javascript')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('cache-control', 'private, s-maxage=0, max-age=0, must-revalidate') #'cache-control': 'private, s-maxage=0, max-age=0, must-revalidate'
        self.end_headers()
        self.wfile.write(f.read().encode())
        f.close()

      elif self.path.endswith(".css"):
        f = open(pages_folder + self.path) 
        self.send_response(200)
        self.send_header('Content-type', 'text/css')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('cache-control', 'private, s-maxage=0, max-age=0, must-revalidate') #'cache-control': 'private, s-maxage=0, max-age=0, must-revalidate'
        self.end_headers()
        self.wfile.write(f.read().encode())
        f.close()
      
      elif self.path.startswith("/VAR" ): 
        self.send_response(200, "ok")
        self.send_header('Content-type',  'text/xml')
        self.send_header('Access-Control-Allow-Origin', '*') #("Cache-Control: no-cache, must-revalidate");
        self.send_header('Expires', 'Sat, 01 Jan 2005 00:00:00 GMT')
        self.send_header('Cache-Control', 'private, s-maxage=0, max-age=0, must-revalidate') #'cache-control': 'private, s-maxage=0, max-age=0, must-revalidate'
        self.end_headers()
        if (self.console_server is not None):
          res = _Ask_Topics(self.path, self.console_server)
        self.wfile.write(res.encode())
      
      elif self.path.startswith("/RSET" ): # Set one variable as a rostopic pub format
        if (self.console_server is not None):
          topic_send = cmd_set_topic(self.path, self.console_server)
        self.send_response(200)
        self.send_header('Content-type',  'text/html')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('cache-control', 'private, s-maxage=0, max-age=0, must-revalidate') #'cache-control': 'private, s-maxage=0, max-age=0, must-revalidate'
        self.end_headers()
        if topic_send: self.wfile.write('OK'.encode())
        else: self.wfile.write('NOK'.encode())

      # elif (self.path.startswith("/action.html")):
      #   f = open(pages_folder + "action.html")
      #   query = urlparse(self.path).query
      #   query_components = dict(qc.split("=") for qc in query.split("&"))
      #   self.send_response(200)
      #   self.send_header('Content-type',  'text/html')
      #   self.send_header('Access-Control-Allow-Origin', '*')
      #   self.send_header('action', query_components['action'])
      #   self.end_headers()
      #   self.wfile.write(f.read().encode())
      #   f.close()
        
      # elif (self.path.endswith(".html") or self.path.endswith(".jpg") or self.path.endswith(".png") or self.path.endswith(".gif")): # Path Files
      #   #self.path has /test.html
      #   f = open(pages_folder+ "." + self.path)
      #   #print "Getting "+pages_folder+ "." + self.path
      #   #note that this potentially makes every file on your computer readable by the internet
      #   self.send_response(200)
      #   self.send_header('Content-type',  'text/html')
      #   self.send_header('Access-Control-Allow-Origin', '*')
      #   self.end_headers()
      #   self.wfile.write(f.read().encode())
      #   f.close()
        
      # elif self.path.endswith("/Missions"): # List Missions
      #   res = _List_Missions()
      #   self.send_response(200)
      #   self.send_header('Content-type',  'text/html')
      #   self.send_header('Access-Control-Allow-Origin', '*')
      #   self.end_headers()
      #   self.wfile.write(res.encode())     
      else:
        print("File not found %s]\n"% self.path)
        self.send_error(404,'File Not Found: %s' % self.path)
      """if self.path.endswith(".esp"):   #our dynamic content
      self.send_response(200)
      self.send_header('Content-type',  'text/html')
      self.end_headers()
      self.wfile.write("hey, today is the" + str(time.localtime()[7]))
      self.wfile.write(" day in the year " + str(time.localtime()[0]))
      return
      """     
    except IOError:
      self.send_error(408,'Request timeout: %s' % self.path)

  def do_POST(self):
    try:
      populenv(self)
      form = cgi.FieldStorage(fp=self.rfile)
      upfilecontent = form['upfile'].value
      if upfilecontent:
        self.send_response(301)
        #print "filecontent:\n", upfilecontent
        print("[HTTP Server] Mission Uploaded " + MISSION_PATH+"/"+str(form['upfile'].filename))
        f = open(MISSION_PATH + "/" + str(form['upfile'].filename), "wb")
        self.end_headers()
        #fout = open(os.path.join('tmp', form['upfile'].filename), 'wb')
        f.write(upfilecontent)
        f.close()
        self.wfile.write("<HTML>Mission Uploaded! <br>".encode());
        _sleep(0.2)
        self.wfile.write(_List_Missions().encode());
        #self.wfile.write(upfilecontent[0]);
    except :
      self.wfile.write("<HTML>ERROR!</HTML>".encode());
      pass
  def do_HEAD(self):
    self.send_response(200)
    self.send_header('Content-type',  'text/html')
    self.send_header('Access-Control-Allow-Origin', '*')
    self.end_headers()

  def address_string(self):
    host,port = self.client_address[:2]
    return host
  
  # Disabling Log Messages and screen Output
  def log_request(self, inputs):
    return ""

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
  """Handle requests in a separate thread."""

# Dictionary to convert from signalnum to signal name (2 = SIGINT)
SIGNALS_TO_NAMES_DICT = dict((getattr(signal, n), n) \
  for n in dir(signal) if n.startswith('SIG') and '_' not in n )

# Function to handle signals
def signal_handler(signal, frame):
  print('[HTTP Server] Handling %s, closing HTTP_Server'%(str(SIGNALS_TO_NAMES_DICT[signal])))
  #sys.exit(0)
  os._exit(0)

class ConsoleServer(Node):
  def __init__(self):
    # Call the parent constructor
    super().__init__('console_server')

    self.loadParams()

    # initialise variables
    self.i = 0

    signal.signal(signal.SIGINT, signal_handler) # handle SIGINT

    # force command to launch console
    command = 'console'
    
    _Ask_Topics("VAR rosout gps/data imu/data bat_monit/data ThrusterR/Status ThrusterL/Status Safety_Feature Leak1 Leak2 Pressure", self)
    try:
      # command = argv[1]
      if command == 'echo':
        # _rostopic_cmd_echo(argv)
        pass
      elif command in 'list':
        # _list_topics()
        pass
      elif command in 'console':
        # self.get_logger().warn('Got in')
        try:
          self.server_thread = threading.Thread(target=self.start_http_server, daemon=True)
          self.server_thread.start()
        except:
          self.get_logger().warn('HTTP thread failed')
      else:
        _fullusage()
    except socket.error:
      sys.stderr.write("Network communication failed. Most likely failed to communicate with master.\n")
      sys.exit(1)
    # except rosgraph.MasterException as e:
    #   # mainly for invalid master URI/rosgraph.masterapi
    #   sys.stderr.write("ERROR: %s\n"%str(e))
    #   server.shutdown()
    #   sys.exit(1)
    except ROSTopicException as e:
      sys.stderr.write("ERROR: %s\n"%str(e))
      self.server.shutdown()
      sys.exit(1)
    except KeyboardInterrupt: 
      self.server.shutdown()
    # except rospy.ROSInterruptException: 
    #   server.shutdown()
  
  def start_http_server(self):
    handler = HTTP_Handler
    handler.console_server = self
    self.server = ThreadedHTTPServer(('', server_port), handler)
    sys.stdout.write("[HTTP Server] Try http://127.0.0.1:"+str(server_port)+"/\n")
    #server.serve_forever()
    while rclpy.ok():
      self.server.handle_request()

    self.server.shutdown()

  ## Load parameters
  def loadParams(self):
    global MISSION_PATH
    global pages_folder
    global server_port

    self.declare_parameter('addons.console_parser.path_folder', '~/paths_from_console')
    self.declare_parameter('pages_folder', "/home/farol/farol-sw/farol_common/http_server/pages/") # should point to the package's own directory
    self.declare_parameter('addons.console_server.PORT', 7080)
    self.declare_parameter('addons.console_server.topics.console.waypoint', 'wp_standard')
    self.declare_parameter('addons.console_server.topics.services.wp_standard', 'wp_standard')

    MISSION_PATH = self.get_parameter('addons.console_parser.path_folder').get_parameter_value().string_value
    pages_folder = self.get_parameter('pages_folder').get_parameter_value().string_value
    server_port = self.get_parameter('addons.console_server.PORT').get_parameter_value().integer_value

    return

def main(args=None):
  rclpy.init(args=args)

  # Set variables as global, such that their values can be changed
  global console_server
  # global g_list_topic_stamp

  # variable not to ros2 topic list for every request
  # g_list_topic_stamp = Clock().now() - Duration(seconds = g_list_topic_thrsh*2)
  
  # Create console server node
  console_server = ConsoleServer()
  rclpy.spin(console_server)

  # Handle shutdown
  console_server.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()