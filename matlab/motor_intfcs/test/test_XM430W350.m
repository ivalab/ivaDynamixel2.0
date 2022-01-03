% 
% Test script: test_DXLIO_XM430_W350_move_single.m
% 
% Description: 
%   Instantiate XM430_W350_IO motor IO class and exercise basic
%   functionality.
% 

% [0] == Script setup
%   Update Matlab path
addpath('../');


% [1] == Instantiate & exercise base functionality
%   Setup
dxlio = DXLIO_XM430_W350();

fprintf('Loading DXL library.\n\n');
dxlio.load_library();

port_name = '/dev/ttyUSB0';
port_baud = 1000000;
fprintf('Opening port: %s at baud: %d.... \n', port_name, port_baud);
openPortResult = dxlio.openPort( port_name, port_baud );
fprintf('Open port success: %d.\n\n', openPortResult);

pause(1);

%   Set LED state
motor_id = 12;
led_state = 0;
fprintf('Commanding LED state: %d for motor ID: %d.\n\n', led_state, motor_id);
tic;
dxlio.set_led( motor_id, led_state );

fprintf('Send time: %.4f sec.\n', toc);
pause(2);

%   Clean-up
fprintf('Closing DXL port: %s.\n', port_name);
dxlio.closePort();
fprintf('Unloading DXL library.\n');
dxlio.unload_library();
