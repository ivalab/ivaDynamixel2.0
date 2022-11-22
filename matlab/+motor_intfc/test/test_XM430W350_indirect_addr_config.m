% 
% Test script: test_XM430W350_indirect_addr_config.m
% 
% Description: 
%   Instantiate XM430_W350_IO motor IO class and set up indirect addresses.
% 

% [0] == Script parameter(s)
PORT_NAME = '/dev/ttyUSB0';
PORT_BAUD = 1000000;

MOTOR_ID = 1;


% [1] == Script setup
%   Update Matlab path
addpath('../');


% [2] == Instantiate & exercise functionality
% Connect
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


% Motor configuration
%   Configure motors for 'Extended Position Control Mode'
oper_mode = 4*ones(size(MOTOR_ID));
fprintf('Setting operating mode: Extended Position Mode.\n');
dxlio.set_operating_mode( MOTOR_ID, oper_mode )
pause(1);

%   Print-out motor EEPROM control table state
print_info = true;
dxlio.get_motor_eeprom_state( MOTOR_ID, print_info );
pause(1);

%   Configure motors indirect registers (simultaneous position & velocity
%   commands)
dxlio.configure_control_table( MOTOR_ID );
pause(1);

%   Verify indirect registers configured
[ groupSyncReadData1 ] = dxlio.groupSyncReadAddr( MOTOR_ID, dxlio.ADDR_INDIRECT_POS_VEL_ACC, 2)
[ groupSyncReadData2 ] = dxlio.groupSyncReadAddr( MOTOR_ID, dxlio.ADDR_INDIRECT_POS_VEL_ACC+2, 2)
[ groupSyncReadData3 ] = dxlio.groupSyncReadAddr( MOTOR_ID, dxlio.ADDR_INDIRECT_POS_VEL_ACC+4, 2)
[ groupSyncReadData4 ] = dxlio.groupSyncReadAddr( MOTOR_ID, dxlio.ADDR_INDIRECT_POS_VEL_ACC+6, 2)

[ groupSyncReadData5 ] = dxlio.groupSyncReadAddr( MOTOR_ID, dxlio.ADDR_INDIRECT_POS_VEL_ACC+8, 2)
[ groupSyncReadData6 ] = dxlio.groupSyncReadAddr( MOTOR_ID, dxlio.ADDR_INDIRECT_POS_VEL_ACC+10, 2)
[ groupSyncReadData7 ] = dxlio.groupSyncReadAddr( MOTOR_ID, dxlio.ADDR_INDIRECT_POS_VEL_ACC+12, 2)
[ groupSyncReadData8 ] = dxlio.groupSyncReadAddr( MOTOR_ID, dxlio.ADDR_INDIRECT_POS_VEL_ACC+14, 2)
pause(1);

%   Clean-up
fprintf('Closing DXL port: %s.\n', PORT_NAME);
dxlio.closePort();
fprintf('Unloading DXL library.\n');
dxlio.unload_library();


