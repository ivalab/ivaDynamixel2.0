% 
% Test script: test_XM430W350_set_goal_vel.m
% 
% Description: 
%   Instantiate XM430_W350_IO motor IO class and command motor position 
%   and velocity.
% 

% [0] == Script parameter(s)
PORT_NAME = '/dev/ttyUSB1';
PORT_BAUD = 1000000;

MOTOR_ID = 1;


% [1] == Script setup
%   Update Matlab path
addpath('../../');


% [2] == Instantiate & exercise functionality
% Connect
dxlio = dxl_intfcs.XM430_W350_IO();
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


% Command and read motor position
%   Query user to continue
input('Press <Enter> to begin trajectory execution ...');

%   Generate sinusoidal position and velocity trajectory
traj_dur = 10; traj_min_vel = (25)*pi/180;
traj_A = (90)*pi/180; traj_f = 0.2;

traj_time = linspace(0, traj_dur, ceil(traj_dur/0.1)+1);   % sec.
traj_dt = [ traj_time(2)-traj_time(1), diff(traj_time) ];

traj_pos = traj_A*sin(2*pi*(traj_f)*traj_time);                    % rad
traj_vel = max(abs([ pi/4, diff(traj_pos)./diff(traj_time) ]), traj_min_vel);  % positive 'velocity' only (rad/s)

%   Enable motor torque
torque_state = 1;
fprintf('Enabling torque: %d, for motor ID: %d.\n\n', torque_state, MOTOR_ID);
dxlio.set_torque_enable( MOTOR_ID, torque_state );
pause(1);

%   Command trajectory
goal_pos = traj_pos(1);  % rad
des_vel = traj_vel(1);   % rad/s
fprintf('Commanding initial position: (%.2f deg, %.2f deg/s) for motor ID: %d.\n', goal_pos*180/pi, des_vel*180/pi, MOTOR_ID);
dxlio.set_goal_pos_vel( MOTOR_ID, goal_pos, des_vel );
pause(2);

fprintf('Beginning trajectory ...\n');
pos_fb = Inf*ones(1, length(traj_pos));
for ii = 2:length(traj_pos)
  [ pres_pos ] = dxlio.get_present_position( MOTOR_ID );
  pos_fb(ii-1) = pres_pos;
  
  goal_pos = traj_pos(ii);  % rad
  des_vel = traj_vel(ii);   % rad/s
  dxlio.set_goal_pos_vel( MOTOR_ID, goal_pos, des_vel );

  pause(traj_dt(ii)*1.0);
end
[ pres_pos ] = dxlio.get_present_position( MOTOR_ID );
pos_fb(end) = pres_pos;
pause(1);
fprintf('Completed trajectory.\n\n');

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


% Plot results
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

figure;
hold on;
  traj_cmd_plot_hdl = plot(traj_time, traj_pos*180/pi, 'go-');
  traj_fb_plot_hdl = plot(traj_time, pos_fb*180/pi, 'r*-');
hold off;
legend([traj_cmd_plot_hdl, traj_fb_plot_hdl], {'Command', 'Measured'});
xlabel('Time (s)'); ylabel('Position (deg)');

rms_err = sqrt(mean((pos_fb-traj_pos).^2));
fprintf('RMS open-loop error: %.2f deg.\n\n', rms_err*180/pi);


