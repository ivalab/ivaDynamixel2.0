% 
% Test script: test_DXLIO_XM430_W350_print_info.m
% 
% Description: 
%   Retrieve motor information.
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

port_name = '/dev/ttyUSB0';
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

%   Get motor info
print_info = true;
motor_status_str = dxlio.get_motor_hw_info( MOTOR_IDS, print_info )
pause(3);

%   Clean-up
fprintf('Closing DXL port: %s.\n', port_name);
dxlio.closePort();
fprintf('Unloading DXL library.\n');
dxlio.unload_library();


