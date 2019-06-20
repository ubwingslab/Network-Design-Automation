# Import DroneKit-Python
from dronekit import connect, Command, LocationGlobal, VehicleMode from pymavlink import mavutil
import time, sys, argparse, math,random



def hoverAt(height, duration):
connection_string = 'tcp:127.0.0.1:5760'


vehicle = connect(connection_string, wait_ready=False)
vehicle.armed = True


local_pos = vehicle.location.local_frame time.sleep(1)
#trigger OFFBOARD mode
msg = vehicle.message_factory.set_attitude_target_encode(
0, # time_boot_ms
0, # Target system
0, # Target component
0b00000000, # Type mask: bit 1 is LSB [1, 0, 0, 0], # Quaternion
0, # Body roll rate in radian
0, # Body pitch rate in radian
0, # Body yaw rate in radian
0.1  # Thrust
)
vehicle.send_mavlink(msg)
vehicle.mode = VehicleMode("OFFBOARD")


#TAKEOFF at height start = time.time()
while time.time() - start < duration:
msg = vehicle.message_factory.set_position_target_local_ned_encode(
0,       # time_boot_ms (not used)
0, 0,    # target system, target component mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
0b0000101111111000, # type_mask (only positions enabled)
local_pos.north, local_pos.east, local_pos.down-height, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
0, 0, 0, # x, y, z velocity in m/s  (not used)
0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink) 
0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
# send command to vehicle vehicle.send_mavlink(msg) time.sleep(0.1)



#go back to 30cm start = time.time()
while time.time() - start < 5:
msg = vehicle.message_factory.set_position_target_local_ned_encode(
0,       # time_boot_ms (not used)
0, 0,    # target system, target component mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
0b0000101111111000, # type_mask (only positions enabled)
local_pos.north, local_pos.east, local_pos.down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
0, 0, 0, # x, y, z velocity in m/s  (not used)
0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
# send command to vehicle vehicle.send_mavlink(msg) time.sleep(0.1)



#commencing landing gently start = time.time()
while time.time() - start < 5:
msg = vehicle.message_factory.set_attitude_target_encode(
0, # time_boot_ms
0, # Target system
0, # Target component
0b00000000, # Type mask: bit 1 is LSB [1, 0, 0, 0], # Quaternion
0, # Body roll rate in radian
0, # Body pitch rate in radian
0, # Body yaw rate in radian
0.55  # Thrust
) vehicle.send_mavlink(msg) time.sleep(0.1)


vehicle.armed = False 

def line(height, length, duration):
connection_string = 'tcp:127.0.0.1:5760'


vehicle = connect(connection_string, wait_ready=False)
vehicle.armed = True


local_pos = vehicle.location.local_frame next_step= False
time.sleep(1)


#trigger OFFBOARD mode
msg = vehicle.message_factory.set_attitude_target_encode(
0, # time_boot_ms
0, # Target system
0, # Target component
0b00000000, # Type mask: bit 1 is LSB [1, 0, 0, 0], # Quaternion
0, # Body roll rate in radian
0, # Body pitch rate in radian
0, # Body yaw rate in radian
0.1  # Thrust
)
vehicle.send_mavlink(msg)
vehicle.mode = VehicleMode("OFFBOARD")


#TAKEOFF at height start = time.time()
while time.time() - start < duration:
msg = vehicle.message_factory.set_position_target_local_ned_encode(
0,       # time_boot_ms (not used)
0, 0,    # target system, target component mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
0b0000101111111000, # type_mask (only positions enabled)
local_pos.north, local_pos.east, local_pos.down-height, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
0, 0, 0, # x, y, z velocity in m/s  (not used)
0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
# send command to vehicle vehicle.send_mavlink(msg) 
#uncomment this for immediate depart
#if vehicle.location.local_frame.down <= ((local_pos.down-height)*0.9):
#    break time.sleep(0.1)


#Go to next position start = time.time()
while time.time() - start < duration:
msg = vehicle.message_factory.set_position_target_local_ned_encode(
0,       # time_boot_ms (not used)
0, 0,    # target system, target component mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
0b0000101111111000, # type_mask (only positions enabled)
local_pos.north+length, local_pos.east, local_pos.down-height, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
0, 0, 0, # x, y, z velocity in m/s  (not used)
0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
# send command to vehicle vehicle.send_mavlink(msg)
#uncomment this for immediate return
#if vehicle.location.local_frame.north >= ((local_pos.north+length)*0.9):
#    break time.sleep(0.1)



#Go back to first position start = time.time()
while time.time() - start < duration:
msg = vehicle.message_factory.set_position_target_local_ned_encode(
0,       # time_boot_ms (not used)
0, 0,    # target system, target component mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
0b0000101111111000, # type_mask (only positions enabled)
local_pos.north, local_pos.east, local_pos.down-height, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
0, 0, 0, # x, y, z velocity in m/s  (not used)
0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
# send command to vehicle


vehicle.send_mavlink(msg)
time.sleep(0.1) 



#Land gently
start = time.time()
while time.time() - start < 20:
msg = vehicle.message_factory.set_attitude_target_encode(
0, # time_boot_ms
0, # Target system
0, # Target component
0b00000000, # Type mask: bit 1 is LSB [1, 0, 0, 0], # Quaternion
0, # Body roll rate in radian
0, # Body pitch rate in radian
0, # Body yaw rate in radian
0.55  # Thrust
) vehicle.send_mavlink(msg) time.sleep(0.1)
vehicle.armed = False



if    name    == '   main   ':
hoverAt(1, 120)
#line(2, 2, 10)
