% 
% Test script: test_XM430W350_led_onoff.m
% 
% Description: 
%   Instantiate XM430_W350_IO motor IO class and update LED motor state.
% 

% [0] == Script parameter(s)
PORT_NAME = '/dev/ttyUSB0';
PORT_BAUD = 1000000;

MOTOR_IDS = 8;
% MOTOR_IDS = 2:12;


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

%   Enable torque, set motor position, read motor position & homing offset
%     Read present motor position
[ cur_motor_pos ] = dxlio.get_present_position( MOTOR_IDS );
for ii = 1:length(cur_motor_pos)
  fprintf('Retrieved motor position: %.4f rad (%.3f deg), motor ID: %d.\n', cur_motor_pos(ii), cur_motor_pos(ii)*180/pi, MOTOR_IDS(ii));
end
pause(1);

%   Clean-up
fprintf('Closing DXL port: %s.\n', PORT_NAME);
dxlio.closePort();
fprintf('Unloading DXL library.\n');
dxlio.unload_library();






