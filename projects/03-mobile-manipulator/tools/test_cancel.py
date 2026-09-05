#!/usr/bin/env python3
"""Integration check: cancel a fresh demo during navigation and verify stop."""
import json
import time
import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger

rclpy.init(); node=rclpy.create_node('mm_cancel_test'); state={}; physics={}
node.create_subscription(String,'/mm/status',lambda m:state.update(json.loads(m.data)),10)
node.create_subscription(String,'/mm/physics',lambda m:physics.update(json.loads(m.data)),10)
client=node.create_client(Trigger,'/mission/cancel')

def wait_for(predicate, timeout):
    until=time.monotonic()+timeout
    while not predicate():
        rclpy.spin_once(node,timeout_sec=.05)
        if time.monotonic()>until: raise TimeoutError('Cancellation integration test timed out')

try:
    wait_for(lambda:state.get('phase')=='NAVIGATE_SOURCE' and physics.get('base_speed',0)>.04 and client.service_is_ready(),100.)
    before=list(physics['base'])
    future=client.call_async(Trigger.Request()); wait_for(future.done,10.)
    assert future.result().success
    wait_for(lambda:state.get('status')=='CANCELLED',10.)
    wait_for(lambda:physics.get('base_speed',999)<.005,8.)
    sim=physics['sim_time']; stopped=list(physics['base'])
    wait_for(lambda:physics['sim_time']>sim+1.,8.)
    assert abs(physics['base'][0]-stopped[0])<.01
    assert abs(physics['base'][1]-stopped[1])<.01
    print(json.dumps({'result':'PASS','cancel_acknowledged':True,'stopped_and_held':True,'before':before,'after':physics['base']}))
finally:
    node.destroy_node(); rclpy.shutdown()
