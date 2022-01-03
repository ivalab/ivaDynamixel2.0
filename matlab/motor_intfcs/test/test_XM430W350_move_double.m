% 
% Test script: test_XM430W350_move_single.m
% 
% Description: 
%   Instantiate XM430_W350_IO motor IO class and command motor position.
% 

% [0] == Script parameter(s)
MOTOR_IDS = [11, 12];


% [1] == Script setup
%   Update Matlab path
addpath('../');


% [2] == Instantiate & exercise base functionality
%   Setup
dxlio = XM430_W350_IO();

fprintf('Loading DXL library.\n\n');
dxlio.load_library();

port_name = '/dev/ttyUSB7';
port_baud = 1000000;
fprintf('Opening port: %s at baud: %d.... \n', port_name, port_baud);
openPortResult = dxlio.openPort( port_name, port_baud );
fprintf('Open port success: %d.\n\n', openPortResult);

%   Ping motor
for ii = 1:length(MOTOR_IDS)
  ping_result = dxlio.pingGetModelNum( MOTOR_IDS(ii) );
  if ( ~ping_result )
    fprintf('\nPing result -> no response!');
  else
    fprintf('Ping result -> Model number: %d, for Motor ID: %d.\n\n', ping_result, MOTOR_IDS(ii));
  end
end
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
fprintf('Closing DXL port: %s.\n', port_name);
dxlio.closePort();
fprintf('Unloading DXL library.\n');
dxlio.unload_library();


