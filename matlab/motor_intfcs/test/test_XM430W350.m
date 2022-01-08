% 
% Test script: test_XM430W350_led_onoff.m
% 
% Description: 
%   Instantiate XM430_W350_IO motor IO class and update LED motor state.
% 

% [0] == Script parameter(s)
PORT_NAME = '/dev/ttyUSB0';
PORT_BAUD = 1000000;

MOTOR_ID = 12;


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
  fprintf('\nPing result -> no response!');
else
  fprintf('Ping result -> Model number: %d, for Motor ID: %d.\n\n', ping_result, MOTOR_ID);
end
pause(1);

%   Set LED state
led_state = 1;
fprintf('Commanding LED state: %d for motor ID: %d.\n\n', led_state, MOTOR_ID);
dxlio.set_led( MOTOR_ID, led_state );
pause(2);

%   Clean-up
fprintf('Closing DXL port: %s.\n', PORT_NAME);
dxlio.closePort();
fprintf('Unloading DXL library.\n');
dxlio.unload_library();
