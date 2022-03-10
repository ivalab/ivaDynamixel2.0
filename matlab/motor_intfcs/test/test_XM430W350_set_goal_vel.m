% 
% Test script: test_XM430W350_set_goal_vel.m
% 
% Description: 
%   Instantiate XM430_W350_IO motor IO class and command motor position 
%   and velocity.
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
[ groupSyncReadData1 ] = dxlio.groupSyncReadAddr( MOTOR_ID, dxlio.ADDR_INDIRECT_POS_VEL, 2);
[ groupSyncReadData2 ] = dxlio.groupSyncReadAddr( MOTOR_ID, dxlio.ADDR_INDIRECT_POS_VEL+2, 2);
[ groupSyncReadData3 ] = dxlio.groupSyncReadAddr( MOTOR_ID, dxlio.ADDR_INDIRECT_POS_VEL+4, 2);
[ groupSyncReadData4 ] = dxlio.groupSyncReadAddr( MOTOR_ID, dxlio.ADDR_INDIRECT_POS_VEL+6, 2);

[ groupSyncReadData5 ] = dxlio.groupSyncReadAddr( MOTOR_ID, dxlio.ADDR_INDIRECT_POS_VEL+8, 2);
[ groupSyncReadData6 ] = dxlio.groupSyncReadAddr( MOTOR_ID, dxlio.ADDR_INDIRECT_POS_VEL+10, 2);
[ groupSyncReadData7 ] = dxlio.groupSyncReadAddr( MOTOR_ID, dxlio.ADDR_INDIRECT_POS_VEL+12, 2);
[ groupSyncReadData8 ] = dxlio.groupSyncReadAddr( MOTOR_ID, dxlio.ADDR_INDIRECT_POS_VEL+14, 2);
pause(1);

%   Query user to continue
input('Press <Enter> to begin gait execution ...');


% Motor position & velocity
%   Enable motor torque
torque_state = 1;
fprintf('Enabling torque: %d, for motor ID: %d.\n\n', torque_state, MOTOR_ID);
dxlio.set_torque_enable( MOTOR_ID, torque_state );
pause(1);

%   Command motor position
goal_pos = (90)*pi/180;  % rad
des_vel = (30)*pi/180;   % rad/s
fprintf('Commanding position & velocity: (%.2f deg, %.2f deg/s) for motor ID: %d.\n\n', goal_pos*180/pi, des_vel*180/pi, MOTOR_ID);
dxlio.set_goal_pos_vel_profile( MOTOR_ID, goal_pos, des_vel );
pause(3);

%   Disable motor torque
torque_state = 0;
fprintf('Disabling torque: %d, for motor ID: %d.\n\n', torque_state, MOTOR_ID);
dxlio.set_torque_enable( MOTOR_ID, torque_state );
pause(1);

%   Clean-up
fprintf('Closing DXL port: %s.\n', PORT_NAME);
dxlio.closePort();
fprintf('Unloading DXL library.\n');
dxlio.unload_library();


