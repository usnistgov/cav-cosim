% MATLAB server to receive messages from the intermediate server
serverPort = 9000;

% Create a TCP server that listens on localhost:9000
server = tcpserver("127.0.0.1", serverPort, "ConnectionChangedFcn", @(s,~)disp("MATLAB: Connection established"));

disp("MATLAB: Waiting for connection on port 9000...");

% Give some time for connection to establish
pause(2);  % wait to ensure intermediate connects

disp("MATLAB: Ready to receive messages.");

% Continuously read and display incoming messages
while true
    if server.NumBytesAvailable > 0
        data = readline(server);
        disp("Received from Intermediate:");
        disp(strtrim(data));  % remove any trailing newline
    end
    pause(0.1);  % small delay to avoid CPU overuse
end
