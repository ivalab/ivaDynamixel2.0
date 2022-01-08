% 
% Test script: test_scan_dxl_port.m
% 
% Description: 
%   Scan (ping) for all Dynamixel motors IDs (0 - 253).
% 

% [0] == Script parameter(s)
PORT_NAME = '/dev/ttyUSB0';
PORT_BAUD = 1000000;

MOTOR_IDS = 0:253;

DXL_PROTOCOL = 2.0;


% [1] == Script setup
%   Update Matlab path
addpath('../');


% [2] == Instantiate & exercise base functionality
%   Setup
dxlio = DXL_IO_Impl( DXL_PROTOCOL );
fprintf('\n');

fprintf('Loading DXL library.\n\n');
dxlio.load_library();

fprintf('Opening port: %s at baud: %d ... \n', PORT_NAME, PORT_BAUD);
openPortResult = dxlio.openPort( PORT_NAME, PORT_BAUD );
fprintf('Open port success: %d.\n\n', openPortResult);

%   Scan motor IDs
fprintf('Scan results: \n Port: %s\n Baud Rate: %d\n==================\n', PORT_NAME, PORT_BAUD);
for ii = 1:length(MOTOR_IDS)
  ping_result = dxlio.pingGetModelNum( MOTOR_IDS(ii) );
  if ( ~ping_result )
    fprintf('[not found] Motor ID: %d -> no response.\n', MOTOR_IDS(ii));  
  else
    fprintf('[FOUND] Motor ID: %d -> Model number: %d.\n', MOTOR_IDS(ii), ping_result);
  end
  pause(0.2);
end
%   Ping motor
fprintf('Pinging target motors ...\n');
for ii = 1:length(MOTOR_IDS)
  ping_result = dxlio.pingGetModelNum( MOTOR_IDS(ii) );
  if ( ~ping_result )
    fprintf('\nPing result -> no response!');
  else
    fprintf('Ping result -> Model number: %d, for Motor ID: %d.\n', ping_result, MOTOR_IDS(ii));
  end
end
fprintf('\n');
pause(2);

%   Clean-up
fprintf('Closing DXL port: %s.\n', port_name);
dxlio.closePort();
fprintf('Unloading DXL library.\n');
dxlio.unload_library();