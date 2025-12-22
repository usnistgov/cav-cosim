"""/*
 * NIST-developed software is provided by NIST as a public service. You may use,
 * copy, and distribute copies of the software in any medium, provided that you
 * keep intact this entire notice. You may improve, modify, and create
 * derivative works of the software or any portion of the software, and you may
 * copy and distribute such modifications or works. Modified works should carry
 * a notice stating that you changed the software and should note the date and
 * nature of any such change. Please explicitly acknowledge the National
 * Institute of Standards and Technology as the source of the software. 
 *
 * NIST-developed software is expressly provided "AS IS." NIST MAKES NO WARRANTY
 * OF ANY KIND, EXPRESS, IMPLIED, IN FACT, OR ARISING BY OPERATION OF LAW,
 * INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, NON-INFRINGEMENT, AND DATA ACCURACY. NIST
 * NEITHER REPRESENTS NOR WARRANTS THAT THE OPERATION OF THE SOFTWARE WILL BE
 * UNINTERRUPTED OR ERROR-FREE, OR THAT ANY DEFECTS WILL BE CORRECTED. NIST DOES
 * NOT WARRANT OR MAKE ANY REPRESENTATIONS REGARDING THE USE OF THE SOFTWARE OR
 * THE RESULTS THEREOF, INCLUDING BUT NOT LIMITED TO THE CORRECTNESS, ACCURACY,
 * RELIABILITY, OR USEFULNESS OF THE SOFTWARE.
 * 
 * You are solely responsible for determining the appropriateness of using and
 * distributing the software and you assume all risks associated with its use,
 * including but not limited to the risks and costs of program errors,
 * compliance with applicable laws, damage to or loss of data, programs or
 * equipment, and the unavailability or interruption of operation. This software 
 * is not intended to be used in any situation where a failure could cause risk
 * of injury or damage to property. The software developed by NIST employees is
 * not subject to copyright protection within the United States.
 *
 * Author: Hadhoum Hajjaj <hadhoum.hajjaj@nist.gov>
*/"""

#TODO: get initial positions, velocities, time period from CARLA

import socket
import time

# === Step 1: Start the ns-3 listener (on port 8100) ===
ns3_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
ns3_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
ns3_server.bind(('localhost', 8100))
ns3_server.listen(1)
print("Listening for ns-3 on port 8100...")

ns3_conn, _ = ns3_server.accept()
print("ns-3 connected.")

# === Main Loop (formatted output for ns-3) ===
try:
    
    time_s = 0
    time_ns = 0

    pos_x = -84.98 
    pos_y = -20.00
    pos_z = 0.5
    vel_x = 1
    vel_y = 0
    vel_z = 0
    
    # time between traffic light signal
    time_period = 0.    1

    # time between steps
    time_sleep = 1

    while True:
        pos_x += vel_x / time_period
        pos_y += vel_y / time_period
        pos_z += vel_z / time_period
        send_flag = 0 # ??
        light_status = 0 # ??
        time_remaining = 0 # ??
        
        ns3_message = f"{time_s} {time_ns} {pos_x} {pos_y} {pos_z} {vel_x} {vel_y} {vel_z} {send_flag} {light_status} {time_remaining}\r\n"
            
        try:
            ns3_conn.sendall(ns3_message.encode())
            print(f"[Intermediate] INFO: Sent fake message to ns-3: {ns3_message.strip()}")
        except BrokenPipeError:
            print("[Intermediate] ERROR: ns-3 disconnected.")
            raise SystemExit(1)  # Clean exit

        # Wait for ns-3 response
        response = ''
        while not response.endswith('\r\n'):
            chunk = ns3_conn.recv(1024).decode()
            if not chunk:
                break
            response += chunk

        print(f"[Intermediate] Received from ns-3: {response.strip()}")

        time_s += time_period

        time.sleep(time_sleep)


except KeyboardInterrupt:
    print("Shutting down due to Ctrl+C...")

finally:
    ns3_conn.close()
    ns3_server.close()

