% 
% Test script: test_XM430W350_move_single.m
% 
% Description: 
%   Instantiate XM430_W350_IO motor IO class and command motor position.
% 

% [0] == Script parameter(s)
PORT_NAME = '/dev/ttyUSB0';
PORT_BAUD = 1000000;

MOTOR_ID = 1;


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
fprintf('Pinging target motor ...\n');
ping_result = dxlio.pingGetModelNum( MOTOR_ID );
if ( ~ping_result )
%   fprintf('\nPing result -> no response!');
  fprintf('[not found] Motor ID: %d -> no response.\n', MOTOR_ID);
else
%   fprintf('Ping result -> Model number: %d, for Motor ID: %d.\n\n', ping_result, MOTOR_ID);
  fprintf('[FOUND] Motor ID: %d -> Model number: %d (%s).\n', MOTOR_ID, ping_result, DXL_IO_Impl.MODEL_NUM2NAME(ping_result));
end
pause(1);

%   Enable motor torque
torque_state = 1;
fprintf('[Motor ID: %d] Enabling torque: %d.\n\n', MOTOR_ID, torque_state);
dxlio.set_torque_enable( MOTOR_ID, torque_state );
pause(1);

%   Command motor position
goal_pos = (0)*pi/180;  % rad
fprintf('[Motor ID: %d] Commanding goal position: %d deg.\n\n', MOTOR_ID, goal_pos*180/pi);
dxlio.set_goal_position( MOTOR_ID, goal_pos );
pause(2);

%   Disable motor torque
torque_state = 0;
fprintf('[Motor ID: %d] Disabling torque: %d.\n\n', MOTOR_ID, torque_state);
dxlio.set_torque_enable( MOTOR_ID, torque_state );
pause(1);


%   Clean-up
fprintf('Closing DXL port: %s.\n', PORT_NAME);
dxlio.closePort();
fprintf('Unloading DXL library.\n');
dxlio.unload_library();


