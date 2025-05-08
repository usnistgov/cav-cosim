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

# === Step 2: Wait for CARLA on port 7000 ===
carla_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
carla_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
carla_server.bind(('localhost', 7000))
carla_server.listen(1)
print("Listening for CARLA on port 7000...")

carla_conn, _ = carla_server.accept()
print("CARLA connected.")

# === Step 3: Wait for Simulink on port 9000 ===
matlab_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
matlab_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
matlab_server.bind(('localhost', 9000))
matlab_server.listen(1)
print("Listening for Simulink on port 9000...")

matlab_sock, _ = matlab_server.accept()
print("Simulink connected.")


# === Main Loop (formatted output for ns-3) ===
try:
    buffer = ''
    last_packet_sent = None

    while True:
        chunk = carla_conn.recv(1024).decode()
        if not chunk:
            break
        buffer += chunk

        while ';' in buffer:
            packet, buffer = buffer.split(';', 1)
            packet = packet.strip()
            if not packet:
                continue

            # Ignore duplicate packets
            if packet == last_packet_sent:
                continue
            last_packet_sent = packet

            print(f"[Intermediate] Received from CARLA: {packet}")

            fields = packet.split(',')
            if len(fields) < 11:
                print("[Intermediate] Invalid data length")
                continue

            try:
                time_s = int(float(fields[0]))
                time_ns = int(float(fields[1]))
                pos_x = float(fields[2])
                pos_y = float(fields[3])
                pos_z = float(fields[4])
                vel_x = float(fields[5])
                vel_y = float(fields[6])
                vel_z = float(fields[7])
                send_flag = int(float(fields[8]))
                light_status = int(float(fields[9]))
                time_remaining = float(fields[10])
            except ValueError:
                print("[Intermediate] Failed to convert fields to correct types")
                continue

            ns3_message = f"{time_s} {time_ns} {pos_x} {pos_y} {pos_z} {vel_x} {vel_y} {vel_z} {send_flag} {light_status} {time_remaining}\r\n"
            
            try:
                ns3_conn.sendall(ns3_message.encode())
                print(f"[Intermediate] Sent to ns-3: {ns3_message.strip()}")
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

            try:
                # Extract lastRecvTime, light_status, time_remaining from ns-3 response
                parts = response.strip().split(',')
                if len(parts) == 6:
                    recv_time = float(parts[0])
                    light_status = int(parts[1])
                    time_remain = float(parts[2])
                    tl_x = float(parts[3])
                    tl_y = float(parts[4])
                    tl_z = float(parts[5])


                    formatted = f"{recv_time:.4f},{light_status},{time_remain:05.2f},{tl_x:.2f},{tl_y:.2f},{tl_z:.2f}\n"

                    matlab_sock.sendall(formatted.encode())
                    print(f"[Intermediate] Sent to MATLAB: {formatted.strip()}")
                    # time.sleep(0.1)  # Sleep 10 ms (adjust based on how fast you send)

                else:
                    print("[Intermediate] Invalid response format from ns-3")

            except Exception as e:
                print(f"[Intermediate] Failed to send to MATLAB: {e}")

except KeyboardInterrupt:
    print("Shutting down due to Ctrl+C...")

finally:
    carla_conn.close()
    carla_server.close()
    ns3_conn.close()
    ns3_server.close()
    matlab_sock.close()

