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
addpath('../../');


% [2] == Instantiate & exercise base functionality
%   Setup
dxlio = dxl_intfcs.XM430_W350_IO();
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
%     fprintf('\nPing result -> no response!');
    fprintf('[not found] Motor ID: %d -> no response.\n', MOTOR_IDS(ii));
  else
%     fprintf('Ping result -> Model number: %d, for Motor ID: %d.\n', ping_result, MOTOR_IDS(ii));
    fprintf('[FOUND] Motor ID: %d -> Model number: %d (%s).\n', MOTOR_IDS(ii), ping_result, DXL_IO_Impl.MODEL_NUM2NAME(ping_result));
  end
end
fprintf('\n');
pause(1);

%   Enable motor torque
torque_state = [1, 1];
for ii = 1:length(MOTOR_IDS)
  fprintf('[Motor ID: %d] Enabling torque: %d.\n', MOTOR_IDS(ii), torque_state(ii));
end
fprintf('\n');
dxlio.set_torque_enable( MOTOR_IDS, torque_state );
pause(1);

%   Command motor position
goal_pos = [45, 90]*pi/180;  % rad
for ii = 1:length(MOTOR_IDS)
  fprintf('[Motor ID: %d] Commanding goal position: %d deg.\n', MOTOR_IDS(ii), goal_pos(ii)*180/pi);
end
fprintf('\n');
dxlio.set_goal_position( MOTOR_IDS, goal_pos );
pause(3);

%   Disable motor torque
torque_state = [0, 0];
for ii = 1:length(MOTOR_IDS)
  fprintf('[Motor ID: %d] Disabling torque: %d.\n', MOTOR_IDS(ii), torque_state(ii));
end
fprintf('\n');
dxlio.set_torque_enable( MOTOR_IDS, torque_state );
pause(1);


%   Clean-up
fprintf('Closing DXL port: %s.\n', PORT_NAME);
dxlio.closePort();
fprintf('Unloading DXL library.\n');
dxlio.unload_library();


