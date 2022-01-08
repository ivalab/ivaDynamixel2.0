% 
% Test script: test_XM430W350_move_single.m
% 
% Description: 
%   Instantiate XM430_W350_IO motor IO class and command motor position.
% 

% [0] == Script parameter(s)
PORT_NAME = '/dev/ttyUSB0';
PORT_BAUD = 1000000;

MOTOR_IDS = [11, 12];


% [1] == Script setup
%   Update Matlab path
addpath('../');


% [2] == Instantiate & exercise base functionality
%   Setup
dxlio = XM430_W350_IO();
fprintf('\n');

fprintf('Loading DXL library.\n\n');
dxlio.load_library();

fprintf('Opening port: %s at baud: %d.... \n', PORT_NAME, PORT_BAUD);
openPortResult = dxlio.openPort( PORT_NAME, PORT_BAUD );
fprintf('Open port success: %d.\n\n', openPortResult);

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
pause(1);

%   Enable motor torque
torque_state = [1, 1];
for ii = 1:length(MOTOR_IDS)
  fprintf('Enabling torque: %d, for motor ID: %d.\n', torque_state(ii), MOTOR_IDS(ii));
end
fprintf('\n');
dxlio.set_torque_enable( MOTOR_IDS, torque_state );
pause(1);

%   Command motor position
goal_pos = [45, 90]*pi/180;  % rad
for ii = 1:length(MOTOR_IDS)
  fprintf('Commanding goal position: %d deg, for motor ID: %d.\n', goal_pos(ii)*180/pi, MOTOR_IDS(ii));
end
fprintf('\n');
dxlio.set_goal_position( MOTOR_IDS, goal_pos );
pause(3);

%   Clean-up
fprintf('Closing DXL port: %s.\n', PORT_NAME);
dxlio.closePort();
fprintf('Unloading DXL library.\n');
dxlio.unload_library();


