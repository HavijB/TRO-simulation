function [q_m, q_dot_m] = sendJointTorques(xi, q, f_m, time_Step)
    % Create a TCP/IP client socket
    t = tcpip('localhost', 1234); % Replace 'localhost' with the IP address of the C++ code
    set(t, 'OutputBufferSize', 4096);
    
    % Connect to the C++ code
    fopen(t);
    
    % Convert joint torques to a string
    torqueStr = sprintf('%f ', f_m);
    
    % Send joint torques to the C++ code
    fwrite(t, torqueStr);
    
    % Wait for a response from the C++ code
    response = fread(t, 1);

    % Receive vectors q_m and q_dot_m from the C++ code
    q_m = fread(t, 7, 'double');
    q_dot_m = fread(t, 7, 'double');

    % Process the response if needed
    
    % Pause for a short duration before sending the next joint torque
    pause(time_Step);
    
    % Close the TCP/IP connection
    fclose(t);
end


