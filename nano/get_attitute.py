
from pymavlink import mavutil
import time



### Start Connection ###

# Create the connection
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()




while True:
    msg = master.recv_match()
    if not msg:
        continue
    if msg.get_type() == 'ATTITUDE':
        print('yaw:',msg.yaw)

#    else:
#        xacc = master.messages["RAW_IMU"].xacc # * 9.81 * 0.001
#        yacc = master.messages["RAW_IMU"].yacc # * 9.81 * 0.001
#        zacc = master.messages["RAW_IMU"].zacc # z* 9.81 * 0.001
#        yaw = master.messages["ATTITUDE"].yaw
#        yaw_speed = master.messages["ATTITUDE"].yawspeed
#        print("%.5f" % xacc,' ',"%.5f" % yacc,' ',"%.5f" % zacc,' ',yaw,' ',yaw_speed)
  #      h = master.messages["VFR_HUD"].alt
  #      prs = master.messages["SCALED_PRESSURE"].press_abs
  #      a = master.messages["GPS_RAW_INT"].alt
  #      print(prs)
  #      print(h)
 # #      print(a)
