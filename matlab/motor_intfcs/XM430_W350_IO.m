%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author:   Alex Chang (modification)
%           Chi Nnoka (original)
% Date: 10/06/2019
% This is a modified version of the Dynamixel_IO class that Alex Chang created for the RX-28 motors.
% This Dynamixel XM430 class is intended for the Dynamixel XM430-W350R motors.
%  WARNING  :  Do not set the motors baud rate over 3 million. 
%              USB-to-serial chipsets we currently use are limited to 
%              3 million baud; setting a higher baud rate may result in an
%              unrecoverable Dynamixel motor.
%              To be safe, stay on 1 million baud.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef XM430_W350_IO < DXL_IO
  % This Class serves to organize the functions that command the new motors
  % Detailed explanation goes here

  properties (Access = protected)
    PROTOCOL_VERSION = 2.0;
    
    ANGLE_MIN = 0;          % min. achievable motor position (rad)
    ANGLE_MAX = 2*pi;       % max. achievable motor position (rad)
    ENC_BIT_LEN = 12;       % encoder count bit-length

    ENC_TO_RAD = 2*pi/(2^12-1);         % (rad/encoder cnts)
    ENC_HOME_POS = ceil((2^12-1)/2);   % Encoder count offset for zero rad. position
  end
  
  properties (Constant)
    % Dynamixel control table field addresses
    % ==== EEPROM Table ==== 
    ADDR_MODEL_NUMBER               = 0;
    ADDR_MODEL_INFORMATION          = 2;
    ADDR_FIRMWARE_VERSION           = 6;
    ADDR_ID                         = 7;
    ADDR_BAUD_RATE                  = 8;
    ADDR_RETURN_DELAY_TIME          = 9;
    ADDR_DRIVE_MODE                 = 10;
    ADDR_OPERATING_MODE             = 11;
    ADDR_SECONDARY_ID               = 12;
    ADDR_PROTOCOL_TYPE              = 13;
    ADDR_HOMING_OFFSET              = 20;
    ADDR_MOVING_THRESHOLD           = 24;
    ADDR_TEMPERATURE_LIMIT          = 31;
    ADDR_MAX_VOLTAGE_LIMIT          = 32;
    ADDR_MIN_VOLTAGE_LIMIT          = 34;
    ADDR_PWM_LIMIT                  = 36;
    ADDR_CURRENT_LIMIT              = 38;
    ADDR_VELOCITY_LIMIT             = 44;
    ADDR_MAX_POSITION_LIMIT         = 48;
    ADDR_MIN_POSITION_LIMIT         = 52;
    ADDR_STARTUP_CONFIGURATION      = 60;
    ADDR_SHUTDOWN                   = 63;
    % ==== RAM Table ==== 
    ADDR_TORQUE_ENABLE              = 64;
    ADDR_LED                        = 65;
    ADDR_STATUS_RETURN_LEVEL        = 68;
    ADDR_REGISTERED_INSTRUCTION     = 69;
    ADDR_HARDWARE_ERROR_STATUS      = 70;
    ADDR_VELOCITY_I_GAIN            = 76;
    ADDR_VELOCITY_P_GAIN            = 78;
    ADDR_POSITION_D_GAIN            = 80;
    ADDR_POSITION_I_GAIN            = 82;
    ADDR_POSITION_P_GAIN            = 84;
    ADDR_FEEDFORWARD_2ND_GAIN       = 88;
    ADDR_FEEDFORWARD_1ST_GAIN       = 90;
    ADDR_BUS_WATCHDOG               = 98;
    ADDR_GOAL_PWM                   = 100;
    ADDR_GOAL_CURRENT               = 102;
    ADDR_GOAL_VELOCITY              = 104;
    ADDR_PROFILE_ACCELERATION       = 108;
    ADDR_PROFILE_VELOCITY           = 112;
    ADDR_GOAL_POSITION              = 116;
    ADDR_REALTIME_TICK              = 120;
    ADDR_MOVING                     = 122;
    ADDR_MOVING_STATUS              = 123;
    ADDR_PRESENT_PWM                = 124;
    ADDR_PRESENT_CURRENT            = 126;
    ADDR_PRESENT_VELOCITY           = 128;
    ADDR_PRESENT_POSITION           = 132;
    ADDR_VELOCITY_TRAJECTORY        = 136;
    ADDR_POSITION_TRAJECTORY        = 140;
    ADDR_PRESENT_INPUT_VOLTAGE      = 144;
    ADDR_PRESENT_TEMPERATURE        = 146;
    ADDR_BACKUP_READY               = 147;
    
    ADDR_INDIRECT_ADDRESS_1          = 168;   % ADDR_INDIRECT_ADDRESS_1 ... ADDR_INDIRECT_ADDRESS_28 = 168:2:222
    ADDR_INDIRECT_DATA_1             = 224;   % ADDR_INDIRECT_DATA_1 ...   ADDR_INDIRECT_DATA_28 = 224:1:251

    ADDR_INDIRECT_ADDRESS_29         = 578;   % ADDR_INDIRECT_ADDRESS_29 ... ADDR_INDIRECT_ADDRESS_56 = 578:2:632
    ADDR_INDIRECT_DATA_29            = 634;   % ADDR_INDIRECT_DATA_29 ... ADDR_INDIRECT_DATA_56 = 634:1:661

    % Dynamixel control table field lengths
    % ==== EEPROM Table ==== 
    LEN_MODEL_NUMBER               = 2;
    LEN_MODEL_INFORMATION          = 4;
    LEN_FIRMWARE_VERSION           = 1;
    LEN_ID                         = 1;
    LEN_BAUD_RATE                  = 1;
    LEN_RETURN_DELAY_TIME          = 1;
    LEN_DRIVE_MODE                 = 1;
    LEN_OPERATING_MODE             = 1;
    LEN_SECONDARY_ID               = 1;
    LEN_PROTOCOL_TYPE              = 1;
    LEN_HOMING_OFFSET              = 4;
    LEN_MOVING_THRESHOLD           = 4;
    LEN_TEMPERATURE_LIMIT          = 1;
    LEN_MAX_VOLTAGE_LIMIT          = 2;
    LEN_MIN_VOLTAGE_LIMIT          = 2;
    LEN_PWM_LIMIT                  = 2;
    LEN_CURRENT_LIMIT              = 2;
    LEN_VELOCITY_LIMIT             = 4;
    LEN_MAX_POSITION_LIMIT         = 4;
    LEN_MIN_POSITION_LIMIT         = 4;
    LEN_STARTUP_CONFIGURATION      = 1;    
    LEN_SHUTDOWN                   = 1;
    % ==== RAM Table ==== 
    LEN_TORQUE_ENABLE              = 1;
    LEN_LED                        = 1;
    LEN_STATUS_RETURN_LEVEL        = 1;
    LEN_REGISTERED_INSTRUCTION     = 1;
    LEN_HARDWARE_ERROR_STATUS      = 1;
    LEN_VELOCITY_I_GAIN            = 2;
    LEN_VELOCITY_P_GAIN            = 2;
    LEN_POSITION_D_GAIN            = 2;
    LEN_POSITION_I_GAIN            = 2;
    LEN_POSITION_P_GAIN            = 2;
    LEN_FEEDFORWARD_2ND_GAIN       = 2;
    LEN_FEEDFORWARD_1ST_GAIN       = 2;
    LEN_BUS_WATCHDOG               = 1;
    LEN_GOAL_PWM                   = 2;
    LEN_GOAL_CURRENT               = 2;
    LEN_GOAL_VELOCITY              = 4;
    LEN_PROFILE_ACCELERATION       = 4;
    LEN_PROFILE_VELOCITY           = 4;
    LEN_GOAL_POSITION              = 4;
    LEN_REALTIME_TICK              = 2;
    LEN_MOVING                     = 1;
    LEN_MOVING_STATUS              = 1;
    LEN_PRESENT_PWM                = 2;
    LEN_PRESENT_CURRENT            = 2;
    LEN_PRESENT_VELOCITY           = 4;
    LEN_PRESENT_POSITION           = 4;
    LEN_VELOCITY_TRAJECTORY        = 4;
    LEN_POSITION_TRAJECTORY        = 4;
    LEN_PRESENT_INPUT_VOLTAGE      = 2;
    LEN_PRESENT_TEMPERATURE        = 1;
    LEN_BACKUP_READY               = 1;
  end
  
  methods  (Access = public)
    % Constructor & initialization
    function obj = XM430_W350_IO()
      
      obj@DXL_IO(); 
      
    end
    
    % TODO: motor calibration procedure (i.e. read motor bias)

    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Control Table EEPROM Area: Write/set methods
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Set motor ID
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_new_id:  vector of LED on/off values (1, 0)
    function set_id( obj, a_motor_ids, a_new_id )
      assert( (length(a_new_id) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_id()]: Incompatible input vector lengths!');
      
      obj.groupSyncWriteAddr( a_motor_ids, a_new_id, obj.ADDR_ID, obj.LEN_ID );
    end

    % Set motor baud rate
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_baud_rate:  vector of LED on/off values (1, 0)
    function set_baud_rate( obj, a_motor_ids, a_baud_rate )
      assert( (length(a_baud_rate) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_baud_rate()]: Incompatible input vector lengths!');
      
      obj.groupSyncWriteAddr( a_motor_ids, a_baud_rate, obj.ADDR_BAUD_RATE, obj.LEN_BAUD_RATE );
    end

    % Set motor return delay time
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_delay:  vector of delay times (0 - 508 usec)
    function set_return_delay_time( obj, a_motor_ids, a_delay )
      assert( (length(a_delay) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_return_delay_time()]: Incompatible input vector lengths!');
      
      delay_cnt = floor(a_delay*254/508);

      obj.groupSyncWriteAddr( a_motor_ids, delay_cnt, obj.ADDR_RETURN_DELAY_TIME, obj.LEN_RETURN_DELAY_TIME );
    end

    % Set motor drive mode flags
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_drive_mode_flag:  vector of drive mode flags (8-bit flag)
    function set_drive_mode( obj, a_motor_ids, a_drive_mode_flag )
      assert( (length(a_drive_mode_flag) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_drive_mode()]: Incompatible input vector lengths!');

      obj.groupSyncWriteAddr( a_motor_ids, a_drive_mode_flag, obj.ADDR_DRIVE_MODE, obj.LEN_DRIVE_MODE );
    end

    % Set motor operating mode value
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_oper_mode_value:  vector of operating mode values (0, 1, 3, 4, 5 or 16)
    function set_operating_mode( obj, a_motor_ids, a_oper_mode_value )
      assert( (length(a_oper_mode_value) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_operating_mode()]: Incompatible input vector lengths!');

      obj.groupSyncWriteAddr( a_motor_ids, a_oper_mode_value, obj.ADDR_OPERATING_MODE, obj.LEN_OPERATING_MODE );
    end

    % Set motor secondary ID
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_secondary_id:  vector of LED on/off values (1, 0)
    function set_secondary_id( obj, a_motor_ids, a_secondary_id )
      assert( (length(a_secondary_id) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_secondary_id()]: Incompatible input vector lengths!');
      
      obj.groupSyncWriteAddr( a_motor_ids, a_secondary_id, obj.ADDR_SECONDARY_ID, obj.LEN_SECONDARY_ID );
    end

    % Set motor communication protocol type
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_protocol_type:  vector of DXL communication protocol types (1 or 2)
    function set_comm_protocol( obj, a_motor_ids, a_protocol_type )
      assert( (length(a_protocol_type) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_comm_protocol()]: Incompatible input vector lengths!');
      
      obj.groupSyncWriteAddr( a_motor_ids, a_protocol_type, obj.ADDR_PROTOCOL_TYPE, obj.LEN_PROTOCOL_TYPE );
    end

    % Set motor homing offset
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_pos_offset:  vector of LED on/off values (1, 0)
    function set_homing_offset( obj, a_motor_ids, a_pos_offset )
      assert( (length(a_pos_offset) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_homing_offset()]: Incompatible input vector lengths!');
      
      obj.groupSyncWriteAddr( a_motor_ids, a_pos_offset, obj.ADDR_HOMING_OFFSET, obj.LEN_HOMING_OFFSET );
    end

    % Set motor moving threshold
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_speed_thresh:  vector of moving threshold speeds (0.0+ rad/sec)
    function set_moving_threshold( obj, a_motor_ids, a_speed_thresh )
      assert( (length(a_speed_thresh) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_moving_threshold()]: Incompatible input vector lengths!');
      
      speed_thresh_cnt = floor(a_speed_thresh/obj.ENC_TO_RAD);

      obj.groupSyncWriteAddr( a_motor_ids, speed_thresh_cnt, obj.ADDR_MOVING_THRESHOLD, obj.LEN_MOVING_THRESHOLD );
    end

    % Set motor temperature limit
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_temp_limit:  vector of temperature limits (0 - 100 deg C; increments of 1 deg)
    function set_temperature_limit( obj, a_motor_ids, a_temp_limit )
      assert( (length(a_temp_limit) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_temperature_limit()]: Incompatible input vector lengths!');
      
      obj.groupSyncWriteAddr( a_motor_ids, floor(a_temp_limit), obj.ADDR_TEMPERATURE_LIMIT, obj.LEN_TEMPERATURE_LIMIT );
    end

    % Set motor max. voltage limit
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_max_voltage:  vector of max. voltage limits (9.5 - 16.0 V)
    function set_max_voltage_limit( obj, a_motor_ids, a_max_voltage )
      assert( (length(a_max_voltage) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_max_voltage_limit()]: Incompatible input vector lengths!');
      
      max_voltage_cnt = floor(a_max_voltage*10);

      obj.groupSyncWriteAddr( a_motor_ids, max_voltage_cnt, obj.ADDR_MAX_VOLTAGE_LIMIT, obj.LEN_MAX_VOLTAGE_LIMIT );
    end

    % Set motor min. voltage limit
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_min_voltage:  vector of max. voltage limits (9.5 - 16.0 V)
    function set_min_voltage_limit( obj, a_motor_ids, a_min_voltage )
      assert( (length(a_min_voltage) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_min_voltage_limit()]: Incompatible input vector lengths!');
      
      min_voltage_cnt = floor(a_min_voltage*10);

      obj.groupSyncWriteAddr( a_motor_ids, min_voltage_cnt, obj.ADDR_MIN_VOLTAGE_LIMIT, obj.LEN_MIN_VOLTAGE_LIMIT );
    end

    % Set motor PWM duty cycle limit
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_duty_cycle_limit:  vector of PWM duty cycle limits (0 - 100%)
    function set_pwm_limit( obj, a_motor_ids, a_duty_cycle_limit )
      assert( (length(a_duty_cycle_limit) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_pwm_limit()]: Incompatible input vector lengths!');
      
      duty_cycle_limit_cnt = floor(a_duty_cycle_limit*855/100);

      obj.groupSyncWriteAddr( a_motor_ids, duty_cycle_limit_cnt, obj.ADDR_PWM_LIMIT, obj.LEN_PWM_LIMIT );
    end

    % Set motor current limit
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_current_limit:  vector of current limits (0 - 3209 mA)
    function set_current_limit( obj, a_motor_ids, a_current_limit )
      assert( (length(a_current_limit) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_current_limit()]: Incompatible input vector lengths!');
      
      current_limit_cnt = floor(a_current_limit/2.69);

      obj.groupSyncWriteAddr( a_motor_ids, current_limit_cnt, obj.ADDR_CURRENT_LIMIT, obj.LEN_CURRENT_LIMIT );
    end

    % Set motor velocity limit
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_vel_limit:  vector of velocity limit values (0 - 24 rad/sec)
    function set_velocity_limit( obj, a_motor_ids, a_vel_limit )
      assert( (length(a_vel_limit) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_velocity_limit()]: Incompatible input vector lengths!');
      
      vel_limit_cnt = floor(a_vel_limit/(0.229*2*pi/60));

      obj.groupSyncWriteAddr( a_motor_ids, vel_limit_cnt, obj.ADDR_VELOCITY_LIMIT, obj.LEN_VELOCITY_LIMIT );
    end

    % Set motor max. position limit
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_max_pos_limit:  vector of max. position limits (0 - 2*pi rad)
    function set_max_position_limit( obj, a_motor_ids, a_max_pos_limit )
      assert( (length(a_max_pos_limit) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_max_position_limit()]: Incompatible input vector lengths!');
      
      max_pos_limit_cnt = floor(a_max_pos_limit/obj.ENC_TO_RAD);

      obj.groupSyncWriteAddr( a_motor_ids, max_pos_limit_cnt, obj.ADDR_MAX_POSITION_LIMIT, obj.LEN_MAX_POSITION_LIMIT );
    end
    
    % Set motor min. position limit
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_min_pos_limit:  vector of min. position limits (0 - 2*pi rad)
    function set_min_position_limit( obj, a_motor_ids, a_min_pos_limit )
      assert( (length(a_min_pos_limit) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_min_position_limit()]: Incompatible input vector lengths!');
      
      min_pos_limit_cnt = floor(a_min_pos_limit/obj.ENC_TO_RAD);

      obj.groupSyncWriteAddr( a_motor_ids, min_pos_limit_cnt, obj.ADDR_MIN_POSITION_LIMIT, obj.LEN_MIN_POSITION_LIMIT );
    end

    % Set motor start-up configuration flags
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_startup_config_flags:  vector of start-up configuration flags (0 - 3; 8-bit flag)
    function set_startup_configuration( obj, a_motor_ids, a_startup_config_flags )
      assert( (length(a_startup_config_flags) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_startup_configuration()]: Incompatible input vector lengths!');
      
      obj.groupSyncWriteAddr( a_motor_ids, a_startup_config_flags, obj.ADDR_STARTUP_CONFIGURATION, obj.LEN_STARTUP_CONFIGURATION );
    end

    % Set motor shutdown condition flags
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_shutdown_cond_flags:  vector of shutdown configuration flags (8-bit flag)
    function set_shutdown_configuration( obj, a_motor_ids, a_shutdown_cond_flags )
      assert( (length(a_shutdown_cond_flags) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_shutdown_configuration()]: Incompatible input vector lengths!');
      
      obj.groupSyncWriteAddr( a_motor_ids, a_shutdown_cond_flags, obj.ADDR_SHUTDOWN, obj.LEN_SHUTDOWN );
    end


    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Control Table EEPROM Area: Read methods
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Read motor model number
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to read data from
    function [ result ] = get_model_number( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_MODEL_NUMBER, obj.LEN_MODEL_NUMBER);

      result = groupSyncReadData;
    end

    % Read motor model information
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to read data from
    function [ result ] = get_model_info( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_MODEL_INFORMATION, obj.LEN_MODEL_INFORMATION);

      result = groupSyncReadData;
    end

    % Read motor firmware version
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to read data from
    function [ result ] = get_firmware_ver( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_FIRMWARE_VERSION, obj.LEN_FIRMWARE_VERSION);

      result = groupSyncReadData;
    end

    % TODO: remaining read/get methods are non-urgent


    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Control Table RAM Area: Write/set methods
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Set motor torque enable
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to configure
    %   a_torque_enable:  vector of torque enable values (1, 0)
    function set_torque_enable( obj, a_motor_ids, a_torque_enable )
      assert( (length(a_torque_enable) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_torque_enable()]: Incompatible input vector lengths!');
      
      obj.groupSyncWriteAddr( a_motor_ids, a_torque_enable, obj.ADDR_TORQUE_ENABLE, obj.LEN_TORQUE_ENABLE );
    end

    % Set motor LED state
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_led_state:  vector of LED on/off values (1, 0)
    function set_led( obj, a_motor_ids, a_led_state )
      assert( (length(a_led_state) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_led()]: Incompatible input vector lengths!');
      
      obj.groupSyncWriteAddr( a_motor_ids, a_led_state, obj.ADDR_LED, obj.LEN_LED );
    end

    % Set motor status return level
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_status_return_level:  vector of LED on/off values (1, 0)
    function set_status_return_level( obj, a_motor_ids, a_status_return_level )
      assert( (length(a_status_return_level) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_status_return_level()]: Incompatible input vector lengths!');
      
      obj.groupSyncWriteAddr( a_motor_ids, a_status_return_level, obj.ADDR_STATUS_RETURN_LEVEL, obj.LEN_STATUS_RETURN_LEVEL );
    end

    % Set motor velocity I-gain
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_I_gain:  vector of LED on/off values (1, 0)
    function set_velocity_I_gain( obj, a_motor_ids, a_I_gain)
      assert( (length(a_I_gain) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_velocity_I_gain()]: Incompatible input vector lengths!');
      
      obj.groupSyncWriteAddr( a_motor_ids, a_I_gain, obj.ADDR_VELOCITY_I_GAIN, obj.LEN_VELOCITY_I_GAIN );
    end

    % Set motor velocity P-gain
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_P_gain:  vector of LED on/off values (1, 0)
    function set_velocity_P_gain( obj, a_motor_ids, a_P_gain)
      assert( (length(a_P_gain) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_velocity_P_gain()]: Incompatible input vector lengths!');
      
      obj.groupSyncWriteAddr( a_motor_ids, a_P_gain, obj.ADDR_VELOCITY_P_GAIN, obj.LEN_VELOCITY_P_GAIN );
    end

    % Set motor position D-gain
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_D_gain:  vector of LED on/off values (1, 0)
    function set_position_D_gain( obj, a_motor_ids, a_D_gain)
      assert( (length(a_D_gain) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_position_D_gain()]: Incompatible input vector lengths!');
      
      obj.groupSyncWriteAddr( a_motor_ids, a_D_gain, obj.ADDR_POSITION_D_GAIN, obj.LEN_POSITION_D_GAIN );
    end

    % Set motor position I-gain
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_I_gain:  vector of LED on/off values (1, 0)
    function set_position_I_gain( obj, a_motor_ids, a_I_gain)
      assert( (length(a_I_gain) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_position_I_gain()]: Incompatible input vector lengths!');
      
      obj.groupSyncWriteAddr( a_motor_ids, a_I_gain, obj.ADDR_POSITION_I_GAIN, obj.LEN_POSITION_I_GAIN );
    end

    % Set motor position P-gain
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_P_gain:  vector of LED on/off values (1, 0)
    function set_position_P_gain( obj, a_motor_ids, a_P_gain)
      assert( (length(a_P_gain) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_position_P_gain()]: Incompatible input vector lengths!');
      
      obj.groupSyncWriteAddr( a_motor_ids, a_P_gain, obj.ADDR_POSITION_P_GAIN, obj.LEN_POSITION_P_GAIN );
    end

    % Set motor feedforward 2nd gain
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_ff_2nd_gain:  vector of LED on/off values (1, 0)
    function set_feedforward_2nd_gain( obj, a_motor_ids, a_ff_2nd_gain)
      assert( (length(a_ff_2nd_gain) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_feedforward_2nd_gain()]: Incompatible input vector lengths!');
      
      obj.groupSyncWriteAddr( a_motor_ids, a_ff_2nd_gain, obj.ADDR_FEEDFORWARD_2ND_GAIN, obj.LEN_FEEDFORWARD_2ND_GAIN );
    end

    % Set motor feedforward 1st gain
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_ff_1st_gain:  vector of LED on/off values (1, 0)
    function set_feedforward_1st_gain( obj, a_motor_ids, a_ff_1st_gain)
      assert( (length(a_ff_1st_gain) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_feedforward_1st_gain()]: Incompatible input vector lengths!');
      
      obj.groupSyncWriteAddr( a_motor_ids, a_ff_1st_gain, obj.ADDR_FEEDFORWARD_1ST_GAIN, obj.LEN_FEEDFORWARD_1ST_GAIN );
    end

    % Set motor bus watchdog value
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_watchdog_inteval:  vector of LED on/off values (1, 0)
    function set_bus_watchdog( obj, a_motor_ids, a_watchdog_inteval)
      assert( (length(a_watchdog_inteval) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_bus_watchdog()]: Incompatible input vector lengths!');
      
      obj.groupSyncWriteAddr( a_motor_ids, a_watchdog_inteval, obj.ADDR_BUS_WATCHDOG, obj.LEN_BUS_WATCHDOG );
    end

    % Set motor goal PWM
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_pwm_duty_cycle:  vector of PWM duty cycle values (0 - 100%)
    function set_goal_pwm( obj, a_motor_ids, a_pwm_duty_cycle)
      assert( (length(a_pwm_duty_cycle) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_goal_pwm()]: Incompatible input vector lengths!');
      
      pwm__duty_cycle_cnt = floor(a_pwm_duty_cycle*885/100);
      
      obj.groupSyncWriteAddr( a_motor_ids, pwm__duty_cycle_cnt, obj.ADDR_GOAL_PWM, obj.LEN_GOAL_PWM );
    end

    % Set motor goal current
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_goal_current:  vector of LED on/off values (1, 0)
    function set_goal_current( obj, a_motor_ids, a_goal_current)
      assert( (length(a_goal_current) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_goal_current()]: Incompatible input vector lengths!');
      
      obj.groupSyncWriteAddr( a_motor_ids, a_goal_current, obj.ADDR_GOAL_CURRENT, obj.LEN_GOAL_CURRENT );
    end

    % Set motor goal velocity
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_goal_velocity:  vector of velocity values (rad/sec)
    function set_goal_velocity( obj, a_motor_ids, a_goal_velocity)
      assert( (length(a_goal_velocity) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_goal_velocity()]: Incompatible input vector lengths!');
      
      goal_velocity_cnt = floor(a_goal_velocity/(0.229*2*pi/60));

      obj.groupSyncWriteAddr( a_motor_ids, goal_velocity_cnt, obj.ADDR_GOAL_VELOCITY, obj.LEN_GOAL_VELOCITY );
    end

    % Set motor acceleration profile
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_accel_profile:  vector of LED on/off values (1, 0)
    function set_acceleration_profile( obj, a_motor_ids, a_accel_profile)
      assert( (length(a_accel_profile) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_acceleration_profile()]: Incompatible input vector lengths!');
      
      obj.groupSyncWriteAddr( a_motor_ids, a_accel_profile, obj.ADDR_PROFILE_ACCELERATION, obj.LEN_PROFILE_ACCELERATION );
    end

    % Set motor velocity profile
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_vel_profile:  vector of LED on/off values (1, 0)
    function set_velocity_profile( obj, a_motor_ids, a_vel_profile)
      assert( (length(a_vel_profile) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_velocity_profile()]: Incompatible input vector lengths!');
      
      obj.groupSyncWriteAddr( a_motor_ids, a_vel_profile, obj.ADDR_PROFILE_VELOCITY, obj.LEN_PROFILE_VELOCITY );
    end

    % Command motor goal positions
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_pos:  vector of position values (0 - 4095)
    function set_goal_position( obj, a_motor_ids, a_pos )
      assert( (length(a_pos) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_goal_pos()]: Incompatible input vector lengths!');

      pos_cnt = floor(a_pos/obj.ENC_TO_RAD);

      obj.groupSyncWriteAddr( a_motor_ids, pos_cnt, obj.ADDR_GOAL_POSITION, obj.LEN_GOAL_POSITION );
    end

    % TODO: method to set goal position & goal velocity simultaneously


    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Control Table RAM Area: Read methods
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Read/check whether instruction registered by REG_WRITE
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:           boolean vector (0 or 1)    
    function [ result ] = is_instruction_registered( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_REGISTERED_INSTRUCTION, obj.LEN_REGISTERED_INSTRUCTION);

      result = groupSyncReadData;
    end

    % Read motor hardware error status (8-bit flag)
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:           vector of 8-bit flags
    function [ result ] = get_hw_error_status( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_HARDWARE_ERROR_STATUS, obj.LEN_HARDWARE_ERROR_STATUS);

      result = groupSyncReadData;
    end

    % Read motor real-time tick
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:           vector of reatime tick time (0 - 32767 ms)
    function [ result ] = get_realtime_tick( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_REALTIME_TICK, obj.LEN_REALTIME_TICK);

      result = groupSyncReadData;
    end

    % Check whether motor horn is moving
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:           boolean vector (0 or 1; 1 = movement detected or movement profile in progress)
    function [ result ] = is_moving( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_MOVING, obj.LEN_MOVING);

      result = groupSyncReadData;
    end

    % Read motor moving status (8-bit flag)
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:           vector of 8-bit flags
    function [ result ] = get_moving_status( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_MOVING_STATUS, obj.LEN_MOVING_STATUS);

      result = groupSyncReadData;
    end
    
    % Read motor present PWM duty cycle
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:           vector of PWM duty cycle values (0 - 100%)
    function [ result ] = get_present_pwm( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_PRESENT_PWM, obj.LEN_PRESENT_PWM);

      result = groupSyncReadData*100/885;
    end
    
    % Read motor present current
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:           vector of current values (mA)
    function [ result ] = get_present_current( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_PRESENT_CURRENT, obj.LEN_PRESENT_CURRENT);

      result = groupSyncReadData*2.69;
    end

    % Read motor present velocity
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:           vector of velocity values (rad/sec)
    function [ result ] = get_present_velocity( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_PRESENT_VELOCITY, obj.LEN_PRESENT_VELOCITY);

      result = groupSyncReadData*0.229*2*pi/60;
    end

    % Read motor present position
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:           vector of position values (rad)
    function [ result ] = get_present_position( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_PRESENT_POSITION, obj.LEN_PRESENT_POSITION);

      result = groupSyncReadData*obj.ENC_TO_RAD;
    end

    % Read motor velocity trajectory
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:           vector of velocity trajectory values (rad/sec)
    function [ result ] = get_velocity_trajectory( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_VELOCITY_TRAJECTORY, obj.LEN_VELOCITY_TRAJECTORY);

      result = groupSyncReadData*0.229*2*pi/60;
    end

    % Read motor position trajectory
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:           vector of position trajectory values (rad)
    function [ result ] = get_position_trajectory( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_POSITION_TRAJECTORY, obj.LEN_POSITION_TRAJECTORY);

      result = groupSyncReadData*obj.ENC_TO_RAD;
    end

    % Read motor present input voltage
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:           vector of voltage measurements (V)
    function [ result ] = get_present_input_voltage( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_PRESENT_INPUT_VOLTAGE, obj.LEN_PRESENT_INPUT_VOLTAGE);

      result = groupSyncReadData/10;
    end

    % Read motor present temperature
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:           vector of temperature measurements (deg C)
    function [ result ] = get_present_temperature( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_PRESENT_TEMPERATURE, obj.LEN_PRESENT_TEMPERATURE);

      result = groupSyncReadData;
    end

    % Check whether back-up of motor control table exists
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:           boolean vector (0 or 1)
    function [ result ] = is_backup_ready( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_BACKUP_READY, obj.LEN_BACKUP_READY);

      result = groupSyncReadData;
    end

    % TODO: print motor info wrapper method -> generate nice info printout/string
    % TODO: print motor error status -> generate nice info printout/string

    % TODO: read/get methods for RW control table entries; not urgent


  end
end
