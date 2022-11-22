% 
% Test script: test_scan_dxl_port.m
% 
% Description: 
%   Scan (ping) for all Dynamixel motors IDs (0 - 253).
% 

% [0] == Script parameter(s)
PORT_NAME = '/dev/ttyUSB0';
PORT_BAUD = 1000000;

% MOTOR_IDS = 0:253;
MOTOR_IDS = 1:12;

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
dxlio.scanForMotors( MOTOR_IDS )


%   Clean-up
fprintf('Closing DXL port: %s.\n', PORT_NAME);
dxlio.closePort();
fprintf('Unloading DXL library.\n');
dxlio.unload_library();
