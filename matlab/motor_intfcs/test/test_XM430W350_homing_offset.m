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

%   Enable torque, set motor position, read motor position & homing offset
%     Enable motor torque
torque_state = 1;
fprintf('Enabling torque: %d, for motor ID: %d.\n\n', torque_state, MOTOR_ID);
dxlio.set_torque_enable( MOTOR_ID, torque_state );
pause(1);

%     Command motor position
goal_pos = (0)*pi/180;  % rad
fprintf('Commanding goal position: %d deg, for motor ID: %d.\n\n', goal_pos*180/pi, MOTOR_ID);
dxlio.set_goal_position( MOTOR_ID, goal_pos );
pause(3);

%     Read present motor position
[ cur_motor_pos ] = dxlio.get_present_position( MOTOR_ID );
fprintf('Retrieved motor position: %.4f rad (%.3f deg), motor ID: %d.\n', cur_motor_pos, cur_motor_pos*180/pi, MOTOR_ID);
pause(1);

%     Read homing offset
[ cur_homing_offset ] = dxlio.get_homing_offset( MOTOR_ID );
fprintf('Retrieved homing offset: %.4f rad, for motor ID: %d.\n\n', cur_homing_offset, MOTOR_ID);

%   Disable torque (write enable to EEPROM), change homing offset, verify
%     Disable motor torque
torque_state = 0;
fprintf('Disabling torque: %d, for motor ID: %d.\n\n', torque_state, MOTOR_ID);
dxlio.set_torque_enable( MOTOR_ID, torque_state );
pause(1);

%     Set homing offset
homing_offset = 45*pi/180;  % rad
fprintf('Configuring homing offset: %.4f rad, for motor ID: %d.\n\n', homing_offset, MOTOR_ID);
dxlio.set_homing_offset( MOTOR_ID, homing_offset );
pause(1);

%     Read homing offset
[ cur_homing_offset ] = dxlio.get_homing_offset( MOTOR_ID );
fprintf('Retrieved homing offset: %.4f rad, for motor ID: %d.\n\n', cur_homing_offset, MOTOR_ID);

%     Read present motor position
[ cur_motor_pos ] = dxlio.get_present_position( MOTOR_ID );
fprintf('Retrieved motor position: %.4f rad (%.3f deg), motor ID: %d.\n', cur_motor_pos, cur_motor_pos*180/pi, MOTOR_ID);
pause(1);

%   Re-enable torque, set new motor position, read motor position 
%     Enable motor torque
torque_state = 1;
fprintf('Enabling torque: %d, for motor ID: %d.\n\n', torque_state, MOTOR_ID);
dxlio.set_torque_enable( MOTOR_ID, torque_state );
pause(1);

%     Command motor position
goal_pos = (0)*pi/180;  % rad
fprintf('Commanding goal position: %d deg, for motor ID: %d.\n\n', goal_pos*180/pi, MOTOR_ID);
dxlio.set_goal_position( MOTOR_ID, goal_pos );
pause(3);

%     Read present motor position
[ cur_motor_pos ] = dxlio.get_present_position( MOTOR_ID );
fprintf('Retrieved motor position: %.4f rad (%.3f deg), motor ID: %d.\n', cur_motor_pos, cur_motor_pos*180/pi, MOTOR_ID);
pause(1);

%   Disable torque, change homing position back to 0, verify
%     Disable motor torque
torque_state = 0;
fprintf('Disabling torque: %d, for motor ID: %d.\n\n', torque_state, MOTOR_ID);
dxlio.set_torque_enable( MOTOR_ID, torque_state );
pause(1);

%     Set homing offset
homing_offset = 0*pi/180;  % rad
fprintf('Configuring homing offset: %.4f rad, for motor ID: %d.\n\n', homing_offset, MOTOR_ID);
dxlio.set_homing_offset( MOTOR_ID, homing_offset );
pause(1);

%     Read present motor position
[ cur_motor_pos ] = dxlio.get_present_position( MOTOR_ID );
fprintf('Retrieved motor position: %.4f rad (%.3f deg), motor ID: %d.\n', cur_motor_pos, cur_motor_pos*180/pi, MOTOR_ID);
pause(1);

%     Read homing offset
[ cur_homing_offset ] = dxlio.get_homing_offset( MOTOR_ID );
fprintf('Retrieved homing offset: %.4f rad, for motor ID: %d.\n\n', cur_homing_offset, MOTOR_ID);
pause(1);


%   Clean-up
fprintf('Closing DXL port: %s.\n', PORT_NAME);
dxlio.closePort();
fprintf('Unloading DXL library.\n');
dxlio.unload_library();






