% 
% Test script: test_XM430W350_led_onoff.m
% 
% Description: 
%   Instantiate XM430_W350_IO motor IO class and update LED motor state.
% 

% [0] == Script parameter(s)
MOTOR_ID = 12;


% [1] == Script setup
%   Update Matlab path
addpath('../');


% [2] == Instantiate & exercise base functionality
%   Setup
dxlio = XM430_W350_IO();

fprintf('Loading DXL library.\n\n');
dxlio.load_library();

port_name = '/dev/ttyUSB0';
port_baud = 1000000;
fprintf('Opening port: %s at baud: %d.... \n', port_name, port_baud);
openPortResult = dxlio.openPort( port_name, port_baud );
fprintf('Open port success: %d.\n\n', openPortResult);

%   Ping motor
ping_result = dxlio.pingGetModelNum( MOTOR_ID );
if ( ~ping_result )
  fprintf('\nPing result -> no response!');
else
  fprintf('Ping result -> Model number: %d, for Motor ID: %d.\n\n', ping_result, MOTOR_ID);
end
pause(1);

%   Set LED state
led_state = 0;
fprintf('Commanding LED state: %d for motor ID: %d.\n\n', led_state, MOTOR_ID);
dxlio.set_led( MOTOR_ID, led_state );
pause(2);

%   Clean-up
fprintf('Closing DXL port: %s.\n', port_name);
dxlio.closePort();
fprintf('Unloading DXL library.\n');
dxlio.unload_library();
