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

# === Step 3: Connect to MATLAB on port 9000 ===
matlab_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
while True:
    try:
        matlab_sock.connect(('localhost', 9000))  # Make sure MATLAB is listening on this port
        print("Connected to MATLAB on port 9000.")
        break
    except ConnectionRefusedError:
        print("Waiting for MATLAB...")
        time.sleep(1)


# === Main Loop (formatted output for ns-3) ===
try:
    while True:
        data = ''
        while not data.endswith(';'):
            chunk = carla_conn.recv(1024).decode()
            if not chunk:
                break
            data += chunk

        if not data:
            continue

        print(f"[Intermediate] Received from CARLA: {data.strip()}")

        # Expected CARLA format: seconds,nanoseconds,x,y,z,vx,vy,vz,send_flag;
        fields = data.strip().strip(';').split(',')
        if len(fields) < 9:
            print("[Intermediate] Invalid data length")
            continue

        try:
            # Convert values to integers (truncate floats)
            time_s = int(float(fields[0]))
            time_ns = int(float(fields[1]))
            pos_x = int(float(fields[2]))
            pos_y = int(float(fields[3]))
            pos_z = int(float(fields[4]))
            vel_x = int(float(fields[5]))
            vel_y = int(float(fields[6]))
            vel_z = int(float(fields[7]))
            send_flag = int(float(fields[8]))
        except ValueError:
            print("[Intermediate] Failed to convert fields to integers")
            continue

        # Construct message for ns-3 (space-separated + \r\n)
        ns3_message = f"{time_s} {time_ns} {pos_x} {pos_y} {pos_z} {vel_x} {vel_y} {vel_z} {send_flag}\r\n"
        ns3_conn.sendall(ns3_message.encode())
        print(f"[Intermediate] Sent to ns-3: {ns3_message.strip()}")

        # Wait for ns-3 response
        response = ''
        while not response.endswith('\r\n'):
            chunk = ns3_conn.recv(1024).decode()
            if not chunk:
                break
            response += chunk

        print(f"[Intermediate] Received from ns-3: {response.strip()}")

        # Forward response to MATLAB
        try:
            matlab_sock.sendall((response.strip() + '\n').encode())
            print("[Intermediate] Sent to MATLAB")
        except Exception as e:
            print(f"[Intermediate] Failed to send to MATLAB: {e}")


        except KeyboardInterrupt:
            print("Shutting down...")

finally:
    carla_conn.close()
    carla_server.close()
    ns3_conn.close()
    ns3_server.close()
    matlab_sock.close()

