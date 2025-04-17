# V2I Communication Example

##  File Included and implementation

- `send_data_from_carla.py`: Sends real-time vehicle telemetry (position, velocity, etc.) over TCP
- `intermediate_server.py`: Forwards vehicle data to ns-3 and receives broadcast result (e.g., from traffic light); then sends the result to MATLAB
- `ns3_gateway_v2i.cc`: Simulates network behavior: traffic light broadcasts, vehicles receive
- `receive_ns3_data_matlab.m`: 	Receives and processes the response data from ns-3 for decision making

![V2I Co-simulation with CARLA, ns-3, and MATLAB](docs/implementation.png)


##  How to use

1. Clone `ns3-cosim` repository:

   ```bash
   git clone https://github.com/usnistgov/ns3-cosim.git
   cd ns3-cosim
   ```
   ```bash
   cp /path/to/ns3_gatewayv2i.cc examples/
   ```
   
   Follow the build instructions provided in the `ns3-cosim` GitHub repository to compile this example.

2. Run Carla

   ```bash
   ./CarlaUE4.sh
   ```

3. Run the Intermediate server:

   ```bash
   pyhton3 /path/to/intermediate_server.py
   ```


4. Run Matlab /path/to/receive_ns3_data_matlab.m
5. Run ns3:

   ```bash
   cd /path/to/ns3_cosim
   ./ns3 run "ns3_gateway_v2i --verbose"
   ```

6. Run the carla data sender

   ```bash
   pyhton3 /path/to/send_data_from_carla.py
   ```


