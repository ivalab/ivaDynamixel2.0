% 
% Test script: test_DXLIO_XM430_W350_print_info.m
% 
% Description: 
%   Retrieve motor information.
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

%   Get motor info
print_info = true;
motor_status_str = dxlio.get_motor_hw_info( MOTOR_IDS, print_info );
disp(motor_status_str);
pause(3);

%   Clean-up
fprintf('Closing DXL port: %s.\n', PORT_NAME);
dxlio.closePort();
fprintf('Unloading DXL library.\n');
dxlio.unload_library();


