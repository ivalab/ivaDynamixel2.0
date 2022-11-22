% 
% Test script: configure_snakey_dxl_motor.m
% 
% Description: 
%   Write Snakey-specialized EEPROM configuration to XM430_W350_IO motor.
% 

% [0] == Script parameter(s)
PORT_NAME = '/dev/ttyUSB1';
PORT_BAUD = 1000000;

MOTOR_OP_MODE = 4;  % 3 = Default Position Control Mode; 4 = Extended Position Control Mode
MOTOR_HOMING_OFFSET = -180*pi/180;%-180*pi/180;  % rad

MOTOR_IDS = 1:8;
% MOTOR_IDS = 12;


% [1] == Script setup
%   Update Matlab path
addpath('../');


% [2] == Instantiate motor interface
%   Setup
dxlio = XM430_W350_IO();
fprintf('\n');

fprintf('Loading DXL library.\n\n');
dxlio.load_library();

fprintf('Opening port: %s at baud: %d.... \n', PORT_NAME, PORT_BAUD);
openPortResult = dxlio.openPort( PORT_NAME, PORT_BAUD );
fprintf('Open port success: %d.\n\n', openPortResult);

%   Ping motors
fprintf('Pinging target motors ...\n');
for ii = 1:length(MOTOR_IDS)
  ping_result = dxlio.pingGetModelNum( MOTOR_IDS(ii) );
  if ( ~ping_result )
    fprintf('[not found] Motor ID: %d -> no response.\n\n', MOTOR_IDS(ii));
  else
    fprintf('[FOUND] Motor ID: %d -> Model number: %d (%s).\n\n', MOTOR_IDS(ii), ping_result, DXL_IO_Impl.MODEL_NUM2NAME(ping_result));
  end
  pause(1);
end


% [3] == Set EEPROM motor properties 
for ii = 1:length(MOTOR_IDS)
  %     Disable motor torque
  torque_state = 0;
  fprintf('Disabling torque: %d, for motor ID: %d.\n\n', torque_state, MOTOR_IDS(ii));
  dxlio.set_torque_enable( MOTOR_IDS(ii), torque_state );
  pause(1);
  
  %   Set operating mode to: Extended Position Control Mode
  fprintf('[Motor ID: %d] Setting Operating Mode: %d.\n\n', MOTOR_IDS(ii), MOTOR_OP_MODE);
  dxlio.set_operating_mode(MOTOR_IDS(ii), MOTOR_OP_MODE);
  pause(1);
  
  %     Set homing offset
  fprintf('Configuring homing offset: %.4f rad, for motor ID: %d.\n\n', MOTOR_HOMING_OFFSET, MOTOR_IDS(ii));
  dxlio.set_homing_offset( MOTOR_IDS(ii), MOTOR_HOMING_OFFSET );
  pause(1);
  
  %   Print-out EEPROM motor configuration
  print_info = true;
  motor_status_str = dxlio.get_motor_eeprom_state( MOTOR_IDS(ii), print_info );
  pause(1);
end


% [4] == Set motor zero position
for ii = 1:length(MOTOR_IDS)
  %   Enable motor torque
  torque_state = 1;
  fprintf('[Motor ID: %d] Enabling torque: %d.\n\n', MOTOR_IDS(ii), torque_state);
  dxlio.set_torque_enable( MOTOR_IDS(ii), torque_state );
  pause(1);
  
  %   Command motor position: 0 deg
  goal_pos = (0)*pi/180;  % rad
  input(sprintf('Press <Enter> to travel to %.2f deg.\n', goal_pos*180/pi));
  fprintf('[Motor ID: %d] Commanding goal position: %.2f deg.\n\n', MOTOR_IDS(ii), goal_pos*180/pi);
  dxlio.set_goal_position( MOTOR_IDS(ii), goal_pos );
  pause(2);
  
  %     Read present motor position
  [ cur_motor_pos ] = dxlio.get_present_position( MOTOR_IDS(ii) );
  fprintf('Retrieved motor position: %.4f rad (%.3f deg), motor ID: %d.\n', cur_motor_pos, cur_motor_pos*180/pi, MOTOR_IDS(ii));
  pause(1);
  
%   %   Command motor position: 90 deg
%   goal_pos = (90)*pi/180;  % rad
%   input(sprintf('Press <Enter> to travel to %.2f deg.\n', goal_pos*180/pi));
%   fprintf('[Motor ID: %d] Commanding goal position: %.2f deg.\n\n', MOTOR_ID, goal_pos*180/pi);
%   dxlio.set_goal_position( MOTOR_ID, goal_pos );
%   pause(2);
%   
%   %     Read present motor position
%   [ cur_motor_pos ] = dxlio.get_present_position( MOTOR_ID );
%   fprintf('Retrieved motor position: %.4f rad (%.3f deg), motor ID: %d.\n', cur_motor_pos, cur_motor_pos*180/pi, MOTOR_ID);
%   pause(1);
%   
%   %   Command motor position: -90 deg
%   goal_pos = (-90)*pi/180;  % rad
%   input(sprintf('Press <Enter> to travel to %.2f deg.\n', goal_pos*180/pi));
%   fprintf('[Motor ID: %d] Commanding goal position: %.2f deg.\n\n', MOTOR_ID, goal_pos*180/pi);
%   dxlio.set_goal_position( MOTOR_ID, goal_pos );
%   pause(2);
%   
%   %     Read present motor position
%   [ cur_motor_pos ] = dxlio.get_present_position( MOTOR_ID );
%   fprintf('Retrieved motor position: %.4f rad (%.3f deg), motor ID: %d.\n', cur_motor_pos, cur_motor_pos*180/pi, MOTOR_ID);
%   pause(1);

  %   Disable motor torque
  torque_state = 0;
  fprintf('[Motor ID: %d] Disabling torque: %d.\n\n', MOTOR_IDS(ii), torque_state);
  dxlio.set_torque_enable( MOTOR_IDS(ii), torque_state );
  pause(1);
end


% [5] == Clean-up
%   Close port, unload library
fprintf('Closing DXL port: %s.\n', PORT_NAME);
dxlio.closePort();
fprintf('Unloading DXL library.\n');
dxlio.unload_library();


