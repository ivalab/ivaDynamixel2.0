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

classdef XM430_W350_IO < dxl_intfcs.DXL_IO
  % This Class serves to organize the functions that command the new motors
  % Detailed explanation goes here

  properties (Access = protected)
    % Motor instance configuration management
    MOTOR_OP_MODE;          % motor ID map: operating (control) modes
    MOTOR_DRIVE_MODE;       % motor ID map: drive modes
    MOTOR_MIN_ANGLE;        % motor ID map: min. achievable motor position (rad)
    MOTOR_MAX_ANGLE;        % motor ID map: max. achievable motor position (rad)
    MOTOR_HOME_ANGLE;       % motor ID map: encoder count offset for zero rad. position

    MOTOR_INDIR_REGS_CONFIGURED;      % motor ID map: boolean flags track whether indirect addresses configured

    PROTOCOL_VERSION = 2.0;
  end
    
  properties (Constant)
    % (Immutable) Motor model-specific properties
    ANGLE_MIN = -180*pi/180;        % min. achievable motor position (rad)
    ANGLE_MAX = 180*pi/180;         % max. achievable motor position (rad)
    ENC_BIT_LEN = 12;               % encoder count bit length

    ENC_TO_RAD = 2*pi/(2^12);     % (rad/encoder cnts)
    ENC_HOME_POS = 0*180/pi;        % Encoder count offset for zero rad. position 
                                    %   (Note: motor-dependent applicability)
    
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
    
    % Control table indirect addresses and data (i.e. 'custom' fields)
    %   Simultaneous comm. of Goal Position and Velocity Profile
    ADDR_INDIRECT_POS_VEL_ACC           = 168;    %    Indirect register location (address mapping)
    ADDR_INDIRECT_DATA_POS_VEL_ACC      = 224;    %    Indirect data register
    LEN_INDIRECT_POS_VEL                = 8;      %    Byte length: 8 (LEN_GOAL_POSITION + LEN_PROFILE_VELOCITY)
    LEN_INDIRECT_POS_VEL_ACC            = 12;     %    Byte length: 12 (LEN_GOAL_POSITION + LEN_PROFILE_VELOCITY + LEN_PROFILE_ACCELERATION)

    % XM430-W350 error code flags/masks
    ERRBIT_VOLTAGE      = 1;
    ERRBIT_OVERHEAT     = 4;
    ERRBIT_ENCODER      = 8;
    ERRBIT_SHOCK        = 16;
    ERRBIT_OVERLOAD     = 32;

    % XM430-W350 operating (control) modes
    OPMODE_CURRENT_CNTRL = 0;
    OPMODE_VELOCITY_CNTRL = 1;
    OPMODE_POSITION_CNTRL = 3;
    OPMODE_EXT_POS_CNTRL = 4;
    OPMODE_CURRENT_POS_CNTRL = 5;
    OPMODE_PWM_CNTRL = 16;
  end
  
  methods  (Access = public)
    % Constructor & initialization
    function obj = XM430_W350_IO()
      
      obj@dxl_intfcs.DXL_IO(); 
      
      % Motor instance configuration management (default initialization),
      % for possible motor IDs 0 - 253
      obj.MOTOR_OP_MODE = uint8(3*ones(1, 254));    % motor operating mode (uint8)
      obj.MOTOR_DRIVE_MODE = uint8(0*ones(1, 254)); % motor drive mode (uint8)
      obj.MOTOR_MIN_ANGLE = 0*ones(1, 254);         % min. achievable motor position (rad)
      obj.MOTOR_MAX_ANGLE = 360*ones(1, 254);       % max. achievable motor position (rad)
      obj.MOTOR_HOME_ANGLE = 0*ones(1, 254);        % angle offset for zero/home position (rad) [not to be confused with motor homing offset]

      obj.MOTOR_INDIR_REGS_CONFIGURED = logical(false*ones(1, 254));     % boolean flag tracks whether indirect register configured for each motor
    end


    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Control table management
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Configure control table indirect addresses with pre-determined
    % arrangement (XM430-W350-specific configuration)
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    % 
    % TODO: Call superclass method for more general indirect reg. config.
    function configure_control_table( obj, a_motor_ids )
      % Consecutive arrangement of:
      %   Goal Position (4 registers), Profile Velocity (4 registers), Profile Acceleration (4 registers) 
      % Mapped to indirect addresses:
      %   ADDR_INDIRECT_POS_VEL_ACC -> ADDR_INDIRECT_POS_VEL_ACC+23
      obj.groupSyncWriteAddr( a_motor_ids, obj.ADDR_GOAL_POSITION, obj.ADDR_INDIRECT_POS_VEL_ACC, 2 );              % Goal Position mapping
      obj.groupSyncWriteAddr( a_motor_ids, obj.ADDR_GOAL_POSITION+1, obj.ADDR_INDIRECT_POS_VEL_ACC+2, 2 );
      obj.groupSyncWriteAddr( a_motor_ids, obj.ADDR_GOAL_POSITION+2, obj.ADDR_INDIRECT_POS_VEL_ACC+4, 2 );
      obj.groupSyncWriteAddr( a_motor_ids, obj.ADDR_GOAL_POSITION+3, obj.ADDR_INDIRECT_POS_VEL_ACC+6, 2 );

      obj.groupSyncWriteAddr( a_motor_ids, obj.ADDR_PROFILE_VELOCITY, obj.ADDR_INDIRECT_POS_VEL_ACC+8, 2 );         % Profile Velocity mapping
      obj.groupSyncWriteAddr( a_motor_ids, obj.ADDR_PROFILE_VELOCITY+1, obj.ADDR_INDIRECT_POS_VEL_ACC+10, 2 );
      obj.groupSyncWriteAddr( a_motor_ids, obj.ADDR_PROFILE_VELOCITY+2, obj.ADDR_INDIRECT_POS_VEL_ACC+12, 2 );
      obj.groupSyncWriteAddr( a_motor_ids, obj.ADDR_PROFILE_VELOCITY+3, obj.ADDR_INDIRECT_POS_VEL_ACC+14, 2 );

      obj.groupSyncWriteAddr( a_motor_ids, obj.ADDR_PROFILE_ACCELERATION, obj.ADDR_INDIRECT_POS_VEL_ACC+16, 2 );    % Profile Acceleration mapping
      obj.groupSyncWriteAddr( a_motor_ids, obj.ADDR_PROFILE_ACCELERATION+1, obj.ADDR_INDIRECT_POS_VEL_ACC+18, 2 );
      obj.groupSyncWriteAddr( a_motor_ids, obj.ADDR_PROFILE_ACCELERATION+2, obj.ADDR_INDIRECT_POS_VEL_ACC+20, 2 );
      obj.groupSyncWriteAddr( a_motor_ids, obj.ADDR_PROFILE_ACCELERATION+3, obj.ADDR_INDIRECT_POS_VEL_ACC+22, 2 );

      % Toggle indicator flag(s)
      if ( sum(a_motor_ids == obj.DXL_BROADCAST_ID) )   % indirect addresses configured for all motors (broadcast ID)
        obj.set_properties( obj.DXL_MIN_ID:obj.DXL_MAX_ID, 'IndirectRegsConfigured', true);
      else
        obj.set_properties( a_motor_ids, 'IndirectRegsConfigured', true);
      end
    end


    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Multi-motor management (configure/enforce motor-specific properties)
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % TODO: validity check on other argument types when parsing
    % TODO: use to set most EEPROM properties (keep instance and motor
    % property values sync'd), will have to read first after init(?)
    function set_properties( obj, a_motor_ids, varargin)
      % Parse input parameters
      parser = inputParser;
      addParameter(parser, 'OpMode', NaN, @obj.opmode_is_valid);         % operating (control) mode
      addParameter(parser, 'DriveMode', NaN, @obj.drivemode_is_valid);   % drive mode
      addParameter(parser, 'MinAngle', NaN);                    % min. angle (SW-enforced; extended position control mode) 
      addParameter(parser, 'MaxAngle', NaN);                    % max. angle (SW-enforced; extended position control mode)
      addParameter(parser, 'HomeAngle', NaN);                   % zero/home angle (SW-implemented; NOT YET IMPLEMENTED HERE)
      addParameter(parser, 'IndirectRegsConfigured', NaN);      % zero/home angle (SW-implemented; NOT YET IMPLEMENTED HERE)
      parse(parser, varargin{:});
      input_params = parser.Results;    % struct format
      
      inds = a_motor_ids+1;
      
      if ( ~isnan(input_params.OpMode) )
        obj.MOTOR_OP_MODE(inds) = input_params.OpMode;
      end
      if ( ~isnan(input_params.DriveMode) )
        obj.MOTOR_DRIVE_MODE(inds) = input_params.DriveMode;
      end
      if ( ~isnan(input_params.MinAngle) )
        obj.MOTOR_MIN_ANGLE(inds) = input_params.MinAngle;
      end
      if ( ~isnan(input_params.MaxAngle) )
        obj.MOTOR_MAX_ANGLE(inds) = input_params.MaxAngle;
      end
      if ( ~isnan(input_params.HomeAngle) )
        obj.MOTOR_HOME_ANGLE(inds) = input_params.HomeAngle;
      end
      if ( ~isnan(input_params.IndirectRegsConfigured) )
        obj.MOTOR_INDIR_REGS_CONFIGURED(inds) = input_params.IndirectRegsConfigured;
      end
    end

    function [ result ] = get_property( obj, a_motor_ids, prop_name )
      assert( sum(obj.valid_motor_id(a_motor_ids)) == length(a_motor_ids), '[DXLIO_XM430_W350::get_property()]: Invalid motors ID(s)!')

      inds = a_motor_ids+1;

      if ( strcmp(prop_name, 'OpMode') )
        result = obj.MOTOR_OP_MODE(inds);
      elseif ( strcmp(prop_name, 'MinAngle') )
        result = obj.MOTOR_MIN_ANGLE(inds);
      elseif ( strcmp(prop_name, 'MaxAngle') )
        result = obj.MOTOR_MAX_ANGLE(inds);
      elseif( strcmp(prop_name, 'HomeAngle') )
        result = obj.MOTOR_HOME_ANGLE(inds);
      elseif( strcmp(prop_name, 'IndirectRegsConfigured') )
        result = obj.MOTOR_INDIR_REGS_CONFIGURED(inds);
      end
    end
    

    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Motor reports
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Read and report motor HW information
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to read data from
    %
    % Output(s):
    %   result:           string summary of motor HW information  
    %
    %   TODO: handle comm. failures (for 1+ motor ids)
    function [ motor_status_str ] = get_motor_hw_info( obj, a_motor_ids, a_print_info )
      if ( nargin < 3 )
        a_print_info = false;
      end
      
      [ result_model_nums ] = obj.get_model_number( a_motor_ids );
%       [ result_model_infos ] = obj.get_model_info( a_motor_ids );     % product lines don't currently implement
      [ result_firmware_vers ] = obj.get_firmware_ver( a_motor_ids );
      
      motor_status_str = cell(size(a_motor_ids));
      for ii = 1:length(a_motor_ids)
        motor_status_str{ii} = sprintf('Motor ID: %d \n\t Model Number: %d (%s)\n\t Firmware Version: %d\n', ...
                                    a_motor_ids(ii), result_model_nums(ii), ...
                                    obj.MODEL_NUM2NAME(result_model_nums(ii)), result_firmware_vers(ii));
      end
      
      if ( a_print_info )
        fprintf('MOTOR HW SUMMARY:\n====================\n\n');
        for ii = 1:length(motor_status_str)
          fprintf('%s\n', motor_status_str{ii});
        end
        fprintf('====================\n\n');
      end
    end
    
    % Read and report motor EEPROM control table fields
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:           string summary of motor EEPROM control table state
    %
    %   TODO: handle comm. failures (for 1+ motor ids); uint8 vals in
    %   parantheses next to bit print out
    function [ motor_status_str ] = get_motor_eeprom_state( obj, a_motor_ids, a_print_info )
      if ( nargin < 3 )
        a_print_info = false;
      end
      
      [ result_baudrates ] = obj.get_baud_rate( a_motor_ids );
      [ result_delaytimes ] = obj.get_return_delay_time( a_motor_ids );
      [ result_drivemode ] = obj.get_drive_mode( a_motor_ids );
      [ result_opmode ] = obj.get_operating_mode( a_motor_ids );
      [ result_secid ] = obj.get_secondary_id( a_motor_ids );
      [ result_commtype ] = obj.get_comm_protocol( a_motor_ids );
      [ result_homingoffset ] = obj.get_homing_offset( a_motor_ids );
      [ result_movethresh ] = obj.get_moving_threshold( a_motor_ids );
      [ result_templim ] = obj.get_temperature_limit( a_motor_ids );
      [ result_maxvoltlim ] = obj.get_max_voltage_limit( a_motor_ids );
      [ result_minvoltlim ] = obj.get_min_voltage_limit( a_motor_ids );
      [ result_pwmlim ] = obj.get_pwm_limit( a_motor_ids );
      [ result_currentlim ] = obj.get_current_limit( a_motor_ids );
      [ result_vellim ] = obj.get_velocity_limit( a_motor_ids );
      [ result_maxposlim ] = obj.get_max_position_limit( a_motor_ids );
      [ result_minposlim ] = obj.get_min_position_limit( a_motor_ids );
      [ result_startup_configs ] = obj.get_startup_configuration( a_motor_ids );
      [ result_shutdownconfigs ] = obj.get_shutdown_configuration( a_motor_ids );
      
      motor_status_str = cell(size(a_motor_ids));
      for ii = 1:length(a_motor_ids)
        motor_status_str{ii} = sprintf('Motor ID: %d \n', a_motor_ids(ii));
        
        motor_status_str{ii} = sprintf('%s\t Baud Rate: %d bps\n', motor_status_str{ii}, result_baudrates(ii));
        motor_status_str{ii} = sprintf('%s\t Return Delay Time: %.2f us\n', motor_status_str{ii}, result_delaytimes(ii));
        motor_status_str{ii} = sprintf('%s\t Drive Mode: %s [8-bit flag]\n', motor_status_str{ii}, dec2bin(result_drivemode(ii), 8));
        motor_status_str{ii} = sprintf('%s\t Operating Mode: %d\n', motor_status_str{ii}, result_opmode(ii));
        motor_status_str{ii} = sprintf('%s\t Secondary ID: %d\n', motor_status_str{ii}, result_secid(ii));
        motor_status_str{ii} = sprintf('%s\t DXL Comm. Protocol: %.1f\n', motor_status_str{ii}, result_commtype(ii));
        motor_status_str{ii} = sprintf('%s\t Homing Offset: %.4f rad (%.3f deg)\n', motor_status_str{ii}, result_homingoffset(ii), result_homingoffset(ii)*180/pi);
        motor_status_str{ii} = sprintf('%s\t Moving Threshold: %.4f rad (%.3f deg)\n', motor_status_str{ii}, result_movethresh(ii), result_movethresh(ii)*180/pi);
        motor_status_str{ii} = sprintf('%s\t Temperature Limit: %.2f deg C\n', motor_status_str{ii}, result_templim(ii));
        motor_status_str{ii} = sprintf('%s\t Max. Voltage Limit: %.2f V\n', motor_status_str{ii}, result_maxvoltlim(ii));
        motor_status_str{ii} = sprintf('%s\t Min. Voltage Limit: %.2f V\n', motor_status_str{ii}, result_minvoltlim(ii));
        motor_status_str{ii} = sprintf('%s\t PWM Duty Cycle Limit: %.2f%%\n', motor_status_str{ii}, result_pwmlim(ii));
        motor_status_str{ii} = sprintf('%s\t Current Limit: %.2f mA\n', motor_status_str{ii}, result_currentlim(ii));
        motor_status_str{ii} = sprintf('%s\t Velocity Limit: %.4f rad/sec (%.3f deg/sec)\n', motor_status_str{ii}, result_vellim(ii), result_vellim(ii)*180/pi);
        motor_status_str{ii} = sprintf('%s\t Max. Position Limit: %.4f rad (%.3f deg)\n', motor_status_str{ii}, result_maxposlim(ii), result_maxposlim(ii)*180/pi);
        motor_status_str{ii} = sprintf('%s\t Min. Position Limit: %.4f rad (%.3f deg)\n', motor_status_str{ii}, result_minposlim(ii), result_minposlim(ii)*180/pi);
        motor_status_str{ii} = sprintf('%s\t Start-up Configuration: %s [8-bit flag]\n', motor_status_str{ii}, dec2bin(result_startup_configs(ii), 8));
        motor_status_str{ii} = sprintf('%s\t Shutdown Configuration: %s [8-bit flag]\n', motor_status_str{ii}, dec2bin(result_shutdownconfigs(ii), 8));
      end
      
      if ( a_print_info )
        fprintf('MOTOR EEPROM STATE:\n====================\n\n');
        for ii = 1:length(motor_status_str)
          fprintf('%s\n', motor_status_str{ii});
        end
        fprintf('====================\n\n');
      end
    end
    
    % Read and report motor error state
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to read data from
    %
    % Output(s):
    %   result:           string summary of motor error states
    %
    %   TODO: handle comm. failures (for 1+ motor ids)
    function [ motor_err_states_str ] = get_motor_error_state( obj, a_motor_ids, a_print_info )
      if ( nargin < 3 )
        a_print_info = false;
      end
      
      [ result_err_states ] = obj.get_hw_error_status( a_motor_ids );
      
      [ motor_err_states_str ] = obj.error_state_to_string( result_err_states );
      
      if ( a_print_info )
        fprintf('MOTOR ERROR STATES:\n====================\n\n');
        for ii = 1:length(motor_err_states_str)
          fprintf('Motor ID: %d \n', a_motor_ids(ii));
          fprintf('%s\n', motor_err_states_str{ii});
        end
        fprintf('====================\n\n');
      end
    end
    
    
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
    %
    % TODO: use ENC2BAUD map
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
    %   a_drive_mode_flags:  vector of drive mode flags (8-bit flag)
    function set_drive_mode( obj, a_motor_ids, a_drive_mode_flags )
      assert( (length(a_drive_mode_flags) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_drive_mode()]: Incompatible input vector lengths!');

      obj.groupSyncWriteAddr( a_motor_ids, a_drive_mode_flags, obj.ADDR_DRIVE_MODE, obj.LEN_DRIVE_MODE );

      % Update motor properties (TODO: put check for broadcast ID inside set_properties)
      if ( sum(a_motor_ids == obj.DXL_BROADCAST_ID) )   % Drive Mode set for all motors (broadcast ID)
        obj.set_properties( obj.DXL_MIN_ID:obj.DXL_MAX_ID, 'DriveMode', a_drive_mode_flags);
      else
        obj.set_properties( a_motor_ids, 'DriveMode', a_drive_mode_flags);
      end
    end

    % Set motor operating mode value
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_oper_mode_value:  vector of operating mode values 
    %                       ( 0  - Current Control Mode
    %                         1  - Velocity Control Mode
    %                         3  - Position Control Mode
    %                         4  - Extended Position Control Mode
    %                         5  - Current-based Position Control Mode
    %                         16 - PWM Control Mode )
    function set_operating_mode( obj, a_motor_ids, a_oper_mode_value )
      assert( (length(a_oper_mode_value) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_operating_mode()]: Incompatible input vector lengths!');

      obj.groupSyncWriteAddr( a_motor_ids, a_oper_mode_value, obj.ADDR_OPERATING_MODE, obj.LEN_OPERATING_MODE );

      % Update motor properties
      if ( sum(a_motor_ids == obj.DXL_BROADCAST_ID) )   % Operating Mode set for all motors (broadcast ID)
        obj.set_properties( obj.DXL_MIN_ID:obj.DXL_MAX_ID, 'OpMode', a_oper_mode_value);
      else
        obj.set_properties( a_motor_ids, 'OpMode', a_oper_mode_value);
      end
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
    %   a_pos_offset:  vector of position offset values (0 - 2*pi rad)
    function set_homing_offset( obj, a_motor_ids, a_pos_offset )
      assert( (length(a_pos_offset) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_homing_offset()]: Incompatible input vector lengths!');
      
      pos_offset_cnt_signed = floor(a_pos_offset/obj.ENC_TO_RAD);
      pos_offset_cnt = typecast(int32(pos_offset_cnt_signed), 'uint32');  % convert double -> 32-bit signed int -> 32-bit unsigned int

      obj.groupSyncWriteAddr( a_motor_ids, pos_offset_cnt, obj.ADDR_HOMING_OFFSET, obj.LEN_HOMING_OFFSET );
    end

    % Set motor moving threshold
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_speed_thresh:  vector of moving threshold speeds (0 - 24.5324 rad/sec)
    function set_moving_threshold( obj, a_motor_ids, a_speed_thresh )
      assert( (length(a_speed_thresh) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_moving_threshold()]: Incompatible input vector lengths!');
      
      speed_thresh_cnt = floor(a_speed_thresh/(0.229*2*pi/60));

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
    % 
    % Note(s): Only applicable when motor in Position Control Mode
    function set_max_position_limit( obj, a_motor_ids, a_max_pos_limit )
      assert( (length(a_max_pos_limit) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_max_position_limit()]: Incompatible input vector lengths!');
      
      max_pos_limit_cnt = floor(a_max_pos_limit/obj.ENC_TO_RAD); % TODO: typecast to uint32?

      obj.groupSyncWriteAddr( a_motor_ids, max_pos_limit_cnt, obj.ADDR_MAX_POSITION_LIMIT, obj.LEN_MAX_POSITION_LIMIT );
    end
    
    % Set motor min. position limit
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_min_pos_limit:  vector of min. position limits (0 - 2*pi rad)
    % 
    % Note(s): Only applicable when motor in Position Control Mode
    function set_min_position_limit( obj, a_motor_ids, a_min_pos_limit )
      assert( (length(a_min_pos_limit) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_min_position_limit()]: Incompatible input vector lengths!');
      
      min_pos_limit_cnt = floor(a_min_pos_limit/obj.ENC_TO_RAD); % TODO: typecast to uint32?

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
    % 
    % Output(s):
    %   result:           vector of motor model numbers (decode /w DXL_IO.MODEL_NUM2NAME map)  
    function [ result ] = get_model_number( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_MODEL_NUMBER, obj.LEN_MODEL_NUMBER);

      result = groupSyncReadData;
    end

    % Read motor model information
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:           vector of model info values (not used)  
    function [ result ] = get_model_info( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_MODEL_INFORMATION, obj.LEN_MODEL_INFORMATION);

      result = groupSyncReadData;
    end

    % Read motor firmware version
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:           vector of firmware versions
    function [ result ] = get_firmware_ver( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_FIRMWARE_VERSION, obj.LEN_FIRMWARE_VERSION);

      result = groupSyncReadData;
    end
    
    % Read motor ID
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:       vector of motor IDs
    function [ result ] = get_id( obj, a_motor_ids )     
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_ID, obj.LEN_ID );

      result = groupSyncReadData;
    end

    % Read motor baud rate
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:       vector of baud rate integer values (decode /w DXL_IO.BAUDRATE_ENC2RATE map) 
    function [ result ] = get_baud_rate( obj, a_motor_ids )      
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_BAUD_RATE, obj.LEN_BAUD_RATE );

      result = -1*ones(size(groupSyncReadData));
      for ii = 1:length(groupSyncReadData)
        result(ii) = obj.BAUDRATE_ENC2RATE(groupSyncReadData(ii));
      end      
    end

    % Read motor return delay time
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:       vector of delay times (0 - 508 usec)
    function [ result ] = get_return_delay_time( obj, a_motor_ids )      
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_RETURN_DELAY_TIME, obj.LEN_RETURN_DELAY_TIME );

      result = groupSyncReadData*508/254;
    end

    % Read motor drive mode flags
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:       vector of drive mode flags (8-bit flag)
    function [ result ] = get_drive_mode( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_DRIVE_MODE, obj.LEN_DRIVE_MODE );

      result = groupSyncReadData;
    end

    % Read motor operating mode value
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:       vector of operating mode values (0, 1, 3, 4, 5 or 16)
    function [ result ] = get_operating_mode( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_OPERATING_MODE, obj.LEN_OPERATING_MODE );

      result = groupSyncReadData;
    end

    % Read motor secondary ID
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:       vector of motor IDs
    function [ result ] = get_secondary_id( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_SECONDARY_ID, obj.LEN_SECONDARY_ID );

      result = groupSyncReadData;
    end

    % Read motor communication protocol type
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:       vector of DXL communication protocol types (1 or 2)
    function [ result ] = get_comm_protocol( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_PROTOCOL_TYPE, obj.LEN_PROTOCOL_TYPE );

      result = groupSyncReadData;
    end

    % Read motor homing offset
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:       vector of position offset values (0 - 2*pi rad)
    function [ result ] = get_homing_offset( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_HOMING_OFFSET, obj.LEN_HOMING_OFFSET );

      groupSyncReadData_signed = double(typecast(uint32(groupSyncReadData), 'int32'));  % convert 32-bit unsigned int -> 32-bit signed int -> double

      result = groupSyncReadData_signed*obj.ENC_TO_RAD;
    end

    % Read motor moving threshold
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:       vector of moving threshold speeds (0 - 24.5324 rad/sec)
    function [ result ] = get_moving_threshold( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_MOVING_THRESHOLD, obj.LEN_MOVING_THRESHOLD );

      result = groupSyncReadData*(0.229*2*pi/60);
    end

    % Read motor temperature limit
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:       vector of temperature limits (0 - 100 deg C; increments of 1 deg)
    function [ result ] = get_temperature_limit( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_TEMPERATURE_LIMIT, obj.LEN_TEMPERATURE_LIMIT );

      result = groupSyncReadData;
    end

    % Read motor max. voltage limit
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:       vector of max. voltage limits (9.5 - 16.0 V)
    function [ result ] = get_max_voltage_limit( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_MAX_VOLTAGE_LIMIT, obj.LEN_MAX_VOLTAGE_LIMIT );

      result = groupSyncReadData/10;
    end

    % Read motor min. voltage limit
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:       vector of max. voltage limits (9.5 - 16.0 V)
    function [ result ] = get_min_voltage_limit( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_MIN_VOLTAGE_LIMIT, obj.LEN_MIN_VOLTAGE_LIMIT );

      result = groupSyncReadData/10;
    end

    % Read motor PWM duty cycle limit
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:       vector of PWM duty cycle limits (0 - 100%)
    function [ result ] = get_pwm_limit( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_PWM_LIMIT, obj.LEN_PWM_LIMIT );

      result = groupSyncReadData*100/855;
    end

    % Read motor current limit
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:       vector of current limits (0 - 3209 mA)
    function [ result ] = get_current_limit( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_CURRENT_LIMIT, obj.LEN_CURRENT_LIMIT );

      result = groupSyncReadData*2.69;
    end

    % Read motor velocity limit
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:       vector of velocity limit values (0 - 24 rad/sec)
    function [ result ] = get_velocity_limit( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_VELOCITY_LIMIT, obj.LEN_VELOCITY_LIMIT );

      result = groupSyncReadData*(0.229*2*pi/60);
    end

    % Read motor max. position limit
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:       vector of max. position limits (0 - 2*pi rad)
    %
    % Note(s): Only applicable when motor in Position Control Mode
    function [ result ] = get_max_position_limit( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_MAX_POSITION_LIMIT, obj.LEN_MAX_POSITION_LIMIT );

      result = groupSyncReadData*obj.ENC_TO_RAD;
    end

    % Read motor min. position limit
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:       vector of min. position limits (0 - 2*pi rad)
    %
    % Note(s): Only applicable when motor in Position Control Mode
    function [ result ] = get_min_position_limit( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_MIN_POSITION_LIMIT, obj.LEN_MIN_POSITION_LIMIT );

      result = groupSyncReadData*obj.ENC_TO_RAD;
    end

    % Read motor start-up configuration flags
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:       vector of start-up configuration flags (0 - 3; 8-bit flag)
    function [ result ] = get_startup_configuration( obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_STARTUP_CONFIGURATION, obj.LEN_STARTUP_CONFIGURATION );

      result = groupSyncReadData;
    end

    % Read motor shutdown condition flags
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to read data from
    % 
    % Output(s):
    %   result:       vector of shutdown configuration flags (8-bit flag)
    function [ result ] = get_shutdown_configuration( obj, a_motor_ids, a_print_info )
      if ( nargin < 3 )
        a_print_info = false;
      end

      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_SHUTDOWN, obj.LEN_SHUTDOWN );

      result = groupSyncReadData;

      if ( a_print_info )
        [ motor_err_states_str ] = obj.error_state_to_string( groupSyncReadData );
        
        if ( a_print_info )
          fprintf('MOTOR SHUTDOWN STATES:\n====================\n\n');
          for ii = 1:length(motor_err_states_str)
            fprintf('Motor ID: %d \n', a_motor_ids(ii));
            fprintf('%s\n', motor_err_states_str{ii});
          end
          fprintf('====================\n\n');
        end
      end
    end


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
    %   a_accel_profile:  vector of acceleration values (>= 0, rad/sec^2)
    %
    % TODO: currently presumes velocity-based profile (Drive Mode)
    function set_acceleration_profile( obj, a_motor_ids, a_accel_profile)
      assert( (length(a_accel_profile) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_acceleration_profile()]: Incompatible input acceleration lengths!');
      
      accel_profile_cnt = floor(a_accel_profile/(214.577*2*pi/3600)); 

      obj.groupSyncWriteAddr( a_motor_ids, accel_profile_cnt, obj.ADDR_PROFILE_ACCELERATION, obj.LEN_PROFILE_ACCELERATION );
    end

    % Set motor velocity profile
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_vel_profile:  vector of (positive) velocity values (>= 0, rad/sec)
    %
    % TODO: currently presumes velocity-based profile (Drive Mode)
    function set_velocity_profile( obj, a_motor_ids, a_vel_profile)
      assert( (length(a_vel_profile) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_velocity_profile()]: Incompatible input vector lengths!');
      
      vel_profile_cnt = floor(a_vel_profile/(0.229*2*pi/60));

      obj.groupSyncWriteAddr( a_motor_ids, vel_profile_cnt, obj.ADDR_PROFILE_VELOCITY, obj.LEN_PROFILE_VELOCITY );
    end

    % Command motor goal positions
    %
    % Input(s):
    %   a_motor_ids:  vector of motor IDs to configure
    %   a_pos:  vector of position values 
    %           (Position Control Mode: 0 - 2*pi rad)
    %           (Extended Position Control Mode: -256*2*pi - +256*2*pi rad)
    %           (Current-based Position Control Mode: -256*2*pi - +256*2*pi rad)
    function set_goal_position( obj, a_motor_ids, a_pos )
      assert( (length(a_pos) == length(a_motor_ids) ), ...
              '[DXLIO_XM430_W350::set_goal_pos()]: Incompatible input vector lengths!');

      pos_cnt_signed = round(a_pos/obj.ENC_TO_RAD);
      pos_cnt = typecast(int32(pos_cnt_signed), 'uint32');  % convert double -> 32-bit signed int -> 32-bit unsigned int

      obj.groupSyncWriteAddr( a_motor_ids, pos_cnt, obj.ADDR_GOAL_POSITION, obj.LEN_GOAL_POSITION );
    end


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
    function [ result ] = get_present_position(  obj, a_motor_ids )
      [ groupSyncReadData ] = obj.groupSyncReadAddr( a_motor_ids, obj.ADDR_PRESENT_POSITION, obj.LEN_PRESENT_POSITION);

      groupSyncReadData_signed = double(typecast(uint32(groupSyncReadData), 'int32'));  % convert 32-bit unsigned int -> 32-bit signed int -> double
      
      result = groupSyncReadData_signed*obj.ENC_TO_RAD;
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

      groupSyncReadData_signed = double(typecast(uint32(groupSyncReadData), 'int32'));  % convert 32-bit unsigned int -> 32-bit signed int -> double

      result = groupSyncReadData_signed*obj.ENC_TO_RAD;
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


    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Custom motor commands
    %   (Based on custom control table indirect register configuration)
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%     % Simultaneously set Goal Position and Velocity Profile
%     %
%     % Input(s):
%     %   a_motor_ids:      vector of motor IDs to configure
%     %   a_pos:  vector of position values 
%     %           (Position Control Mode: 0 - 2*pi rad)
%     %           (Extended Position Control Mode: -256*2*pi - +256*2*pi rad)
%     %           (Current-based Position Control Mode: -256*2*pi - +256*2*pi rad)
%     %   a_vel_profile:  vector of velocity values (rad/sec)
%     function set_goal_pos_vel( obj, a_motor_ids, a_pos, a_vel_profile )
%       assert( sum(obj.get_property( a_motor_ids, 'IndirectRegsConfigured' )) == length(a_motor_ids), ...
%               '[DXLIO_XM430_W350::set_goal_pos_vel()]: Indirect registers not configured appropriately for one or more motors. Use configure_control_table() first.');
%       assert( (length(a_pos) == length(a_motor_ids) && length(a_vel_profile) == length(a_motor_ids)), ...
%               '[DXLIO_XM430_W350::set_goal_pos_vel()]: Incompatible input vector lengths!');
% 
%       group_id = obj.groupSyncWrite( obj.ADDR_INDIRECT_DATA_POS_VEL_ACC, obj.LEN_INDIRECT_POS_VEL );
% 
%       % Add motor IDs to write group
%       for ii = 1:length(a_motor_ids)
%         % Add goal position to group write config.
%         pos_cnt_signed = floor(a_pos(ii)/obj.ENC_TO_RAD);
%         pos_cnt = typecast(int32(pos_cnt_signed), 'uint32');  % convert double -> 32-bit signed int -> 32-bit unsigned int
%         if ( ~obj.groupSyncWriteAddParam( group_id, a_motor_ids(ii), pos_cnt, int32(obj.LEN_GOAL_POSITION) ) )
%           error('[DXL_IO::set_goal_pos_vel()] Motor goal position failed to be added to write group (%d) ...', a_motor_ids(ii));
%         end
% 
%         % Add velocity profile to group write config.
%         vel_profile_cnt = floor(a_vel_profile(ii)/(0.229*2*pi/60));
%         if ( ~obj.groupSyncWriteAddParam( group_id, a_motor_ids(ii), vel_profile_cnt, int32(obj.LEN_PROFILE_VELOCITY) ) )
%           error('[DXL_IO::set_goal_pos_vel()] Motor velocity profile failed to be added to write group (%d) ...', a_motor_ids(ii));
%         end
%       end
%       
%       obj.groupSyncWriteTxPacket( group_id );
%         
%       % TODO: check TxRxResult (optional)?
%       dxl_comm_result = obj.getLastTxRxResult();
%       if dxl_comm_result ~= obj.COMM_TXSUCCESS
%           warning('Comm. error: %s\n', obj.getTxRxResult(dxl_comm_result));
%       end
%       
%       obj.groupSyncWriteClearParam( group_id );        % clear write group data (permits group ID re-use)
%     end

%     % Simultaneously set Goal Position, Velocity Profile and Acceleration
%     % Profile
%     %
%     % Input(s):
%     %   a_motor_ids:      vector of motor IDs to configure
%     %   a_pos:  vector of position values 
%     %           (Position Control Mode: 0 - 2*pi rad)
%     %           (Extended Position Control Mode: -256*2*pi - +256*2*pi rad)
%     %           (Current-based Position Control Mode: -256*2*pi - +256*2*pi rad)
%     %   a_vel_profile:  vector of velocity values (rad/sec)
%     %   a_accel_profile:  vector of acceleration values (>= 0, rad/sec^2)
%     function set_goal_pos_vel_accel( obj, a_motor_ids, a_pos, a_vel_profile, a_accel_profile )
%       assert( sum(obj.get_property( a_motor_ids, 'IndirectRegsConfigured' )) == length(a_motor_ids), ...
%               '[DXLIO_XM430_W350::set_goal_pos_vel_accel()]: Indirect registers not configured appropriately for one or more motors. Use configure_control_table() first.');
%       assert( (length(a_pos) == length(a_motor_ids) && length(a_vel_profile) == length(a_motor_ids) && length(a_accel_profile) == length(a_motor_ids)), ...
%               '[DXLIO_XM430_W350::set_goal_pos_vel_accel()]: Incompatible input vector lengths!');
% 
%       group_id = obj.groupSyncWrite( obj.ADDR_INDIRECT_DATA_POS_VEL_ACC, obj.LEN_INDIRECT_POS_VEL_ACC );
% 
%       % Add motor IDs to write group
%       for ii = 1:length(a_motor_ids)
%         % Add goal position to group write config.
%         pos_cnt_signed = floor(a_pos(ii)/obj.ENC_TO_RAD);
%         pos_cnt = typecast(int32(pos_cnt_signed), 'uint32');  % convert double -> 32-bit signed int -> 32-bit unsigned int
%         if ( ~obj.groupSyncWriteAddParam( group_id, a_motor_ids(ii), pos_cnt, int32(obj.LEN_GOAL_POSITION) ) )
%           error('[DXL_IO::set_goal_pos_vel_accel()] Motor goal position failed to be added to write group (%d) ...', a_motor_ids(ii));
%         end
% 
%         % Add velocity profile to group write config.
%         vel_profile_cnt = floor(a_vel_profile(ii)/(0.229*2*pi/60));
%         if ( ~obj.groupSyncWriteAddParam( group_id, a_motor_ids(ii), vel_profile_cnt, int32(obj.LEN_PROFILE_VELOCITY) ) )
%           error('[DXL_IO::set_goal_pos_vel_accel()] Motor velocity profile failed to be added to write group (%d) ...', a_motor_ids(ii));
%         end
% 
%         % Add acceleration profile to group write config.
%         accel_profile_cnt = floor(a_accel_profile/(214.577*2*pi/3600)); 
%         if ( ~obj.groupSyncWriteAddParam( group_id, a_motor_ids(ii), accel_profile_cnt, int32(obj.LEN_PROFILE_ACCELERATION) ) )
%           error('[DXL_IO::set_goal_pos_vel_accel()] Motor acceleration profile failed to be added to write group (%d) ...', a_motor_ids(ii));
%         end
%       end
%       
%       obj.groupSyncWriteTxPacket( group_id );
%         
%       % TODO: check TxRxResult (optional)?
%       dxl_comm_result = obj.getLastTxRxResult();
%       if dxl_comm_result ~= obj.COMM_TXSUCCESS
%           warning('Comm. error: %s\n', obj.getTxRxResult(dxl_comm_result));
%       end
%       
%       obj.groupSyncWriteClearParam( group_id );        % clear write group data (permits group ID re-use)
%     end

    % Simultaneously set Goal Position and Velocity Profile
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to configure
    %   a_pos:  vector of position values 
    %           (Position Control Mode: 0 - 2*pi rad)
    %           (Extended Position Control Mode: -256*2*pi - +256*2*pi rad)
    %           (Current-based Position Control Mode: -256*2*pi - +256*2*pi rad)
    %   a_vel_profile:  vector of velocity values (rad/sec)
    function set_goal_pos_vel( obj, a_motor_ids, a_pos, a_vel_profile )
      assert( (length(a_pos) == length(a_motor_ids) && length(a_vel_profile) == length(a_motor_ids)), ...
              '[DXLIO_XM430_W350::set_goal_pos_vel()]: Incompatible input vector lengths!');

      group_id = obj.groupSyncWrite( obj.ADDR_PROFILE_VELOCITY, obj.LEN_PROFILE_VELOCITY + obj.LEN_GOAL_POSITION);

      % Add motor IDs to write group
      for ii = 1:length(a_motor_ids)
        % Add velocity profile to group write config.
        vel_profile_cnt = floor(a_vel_profile(ii)/(0.229*2*pi/60));
        if ( ~obj.groupSyncWriteAddParam( group_id, a_motor_ids(ii), vel_profile_cnt, obj.LEN_PROFILE_VELOCITY ) )
          error('[DXL_IO::set_goal_pos_vel()] Motor velocity profile failed to be added to write group (%d) ...', a_motor_ids(ii));
        end

        % Add goal position to group write config.
        pos_cnt_signed = floor(a_pos(ii)/obj.ENC_TO_RAD);
        pos_cnt = typecast(int32(pos_cnt_signed), 'uint32');  % convert double -> 32-bit signed int -> 32-bit unsigned int
        if ( ~obj.groupSyncWriteAddParam( group_id, a_motor_ids(ii), pos_cnt, obj.LEN_GOAL_POSITION ) )
          error('[DXL_IO::set_goal_pos_vel()] Motor goal position failed to be added to write group (%d) ...', a_motor_ids(ii));
        end
      end
      
      obj.groupSyncWriteTxPacket( group_id );
        
      % TODO: check TxRxResult (optional)?
      dxl_comm_result = obj.getLastTxRxResult();
      if dxl_comm_result ~= obj.COMM_TXSUCCESS
          warning('Comm. error: %s\n', obj.getTxRxResult(dxl_comm_result));
      end
      
      obj.groupSyncWriteClearParam( group_id );        % clear write group data (permits group ID re-use)
    end

    % Simultaneously set Goal Position, Velocity Profile and Acceleration
    % Profile
    %
    % Input(s):
    %   a_motor_ids:      vector of motor IDs to configure
    %   a_pos:  vector of position values 
    %           (Position Control Mode: 0 - 2*pi rad)
    %           (Extended Position Control Mode: -256*2*pi - +256*2*pi rad)
    %           (Current-based Position Control Mode: -256*2*pi - +256*2*pi rad)
    %   a_vel_profile:  vector of velocity values (rad/sec)
    %   a_accel_profile:  vector of acceleration values (>= 0, rad/sec^2)
    function set_goal_pos_vel_accel( obj, a_motor_ids, a_pos, a_vel_profile, a_accel_profile )
      assert( (length(a_pos) == length(a_motor_ids) && length(a_vel_profile) == length(a_motor_ids) && length(a_accel_profile) == length(a_motor_ids)), ...
              '[DXLIO_XM430_W350::set_goal_pos_vel_accel()]: Incompatible input vector lengths!');

      group_id = obj.groupSyncWrite( obj.ADDR_PROFILE_ACCELERATION, ...
                                      obj.LEN_PROFILE_ACCELERATION + obj.LEN_PROFILE_VELOCITY + obj.LEN_GOAL_POSITION );

      % Add motor IDs to write group
      for ii = 1:length(a_motor_ids)
        % Add acceleration profile to group write config.
        accel_profile_cnt = floor(a_accel_profile/(214.577*2*pi/3600)); 
        if ( ~obj.groupSyncWriteAddParam( group_id, a_motor_ids(ii), accel_profile_cnt, obj.LEN_PROFILE_ACCELERATION ) )
          error('[DXL_IO::set_goal_pos_vel_accel()] Motor acceleration profile failed to be added to write group (%d) ...', a_motor_ids(ii));
        end

        % Add velocity profile to group write config.
        vel_profile_cnt = floor(a_vel_profile(ii)/(0.229*2*pi/60));
        if ( ~obj.groupSyncWriteAddParam( group_id, a_motor_ids(ii), vel_profile_cnt, obj.LEN_PROFILE_VELOCITY ) )
          error('[DXL_IO::set_goal_pos_vel_accel()] Motor velocity profile failed to be added to write group (%d) ...', a_motor_ids(ii));
        end

        % Add goal position to group write config.
        pos_cnt_signed = floor(a_pos(ii)/obj.ENC_TO_RAD);
        pos_cnt = typecast(int32(pos_cnt_signed), 'uint32');  % convert double -> 32-bit signed int -> 32-bit unsigned int
        if ( ~obj.groupSyncWriteAddParam( group_id, a_motor_ids(ii), pos_cnt, obj.LEN_GOAL_POSITION ) )
          error('[DXL_IO::set_goal_pos_vel_accel()] Motor goal position failed to be added to write group (%d) ...', a_motor_ids(ii));
        end
      end
      
      obj.groupSyncWriteTxPacket( group_id );
        
      % TODO: check TxRxResult (optional)?
      dxl_comm_result = obj.getLastTxRxResult();
      if dxl_comm_result ~= obj.COMM_TXSUCCESS
          warning('Comm. error: %s\n', obj.getTxRxResult(dxl_comm_result));
      end
      
      obj.groupSyncWriteClearParam( group_id );        % clear write group data (permits group ID re-use)
    end


    % TODO: print motor error status -> generate nice info printout/string
    % TODO: read/get methods for RW control table entries; not urgent

    % TODO: unit conversion utilities (e.g. pos, vel, acc) to/from encoder counts

    % INDIR ADDR config. goes in base class

    % constants for flag value, e.g. EXTENDED_POSITION_CONTROL_MODE

  end

  methods (Access = protected)
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Helpers
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function [ err_states_str ] = error_state_to_string( obj, a_err_state )
      err_states_str = cell(length(a_err_state), 1);
      for ii = 1:length(a_err_state)
        err_flags = a_err_state(ii);

        % Parse into boolean (string) outcomes
        if ( bitand(uint8(err_flags), uint8(obj.ERRBIT_VOLTAGE)) ) err_voltage = 'Y'; else err_voltage = 'N'; end
        if ( bitand(uint8(err_flags), uint8(obj.ERRBIT_OVERHEAT)) ) err_overheat = 'Y'; else err_overheat = 'N'; end
        if ( bitand(uint8(err_flags), uint8(obj.ERRBIT_ENCODER)) ) err_encoder = 'Y'; else err_encoder = 'N'; end
        if ( bitand(uint8(err_flags), uint8(obj.ERRBIT_SHOCK)) ) err_shock = 'Y'; else err_shock = 'N'; end
        if ( bitand(uint8(err_flags), uint8(obj.ERRBIT_OVERLOAD)) ) err_overload = 'Y'; else err_overload = 'N'; end
        
        % Format error reports
        err_states_str{ii} = sprintf('%s\t [%s]\tInput Voltage Error\n', err_states_str{ii}, err_voltage);
        err_states_str{ii} = sprintf('%s\t [%s]\tOverheating Error\n', err_states_str{ii}, err_overheat);
        err_states_str{ii} = sprintf('%s\t [%s]\tMotor Encoder Error\n', err_states_str{ii}, err_encoder);
        err_states_str{ii} = sprintf('%s\t [%s]\tElectrical Shock Error\n', err_states_str{ii}, err_shock);
        err_states_str{ii} = sprintf('%s\t [%s]\tOverload Error\n', err_states_str{ii}, err_overload);
      end      
    end

    function [ result ] = opmode_to_string( obj, a_opmode )
      if ( a_opmode == obj.OPMODE_CURRENT_CNTRL )
        result = 'OP_MODE_CURRENT_CNTRL';
      elseif ( a_opmode == obj.OPMODE_VELOCITY_CNTRL )
        result = 'OPMODE_VELOCITY_CNTRL';
      elseif ( a_opmode == obj.OPMODE_POSITION_CNTRL )
        result = 'OPMODE_POSITION_CNTRL';
      elseif ( a_opmode == obj.OPMODE_EXT_POS_CNTRL )
        result = 'OPMODE_EXT_POS_CNTRL';
      elseif ( a_opmode == obj.OPMODE_CURRENT_POS_CNTRL )
        result = 'OPMODE_CURRENT_POS_CNTRL';
      elseif ( a_opmode == obj.OPMODE_PWM_CNTRL )
        result = 'OPMODE_PWM_CNTRL';
      else
        error('[XM430_W350_IO::opmode_to_string()] Invalid operating mode specified: %d.', a_opmode);
      end
    end

    function [ result ] = opmode_is_valid( obj, a_opmode )
      result = ( a_opmode == obj.OPMODE_CURRENT_CNTRL || ...
                  a_opmode == obj.OPMODE_VELOCITY_CNTRL || ...
                  a_opmode == obj.OPMODE_POSITION_CNTRL ||...
                  a_opmode == obj.OPMODE_EXT_POS_CNTRL || ...
                  a_opmode == obj.OPMODE_CURRENT_POS_CNTRL || ...
                  a_opmode == obj.OPMODE_PWM_CNTRL );
    end

    function [ result ] = drivemode_is_valid( obj, a_drivemode )
      result = ( bitand(a_drivemode, 242) == 0 );   % Bits 1, 4, 5, 6, 7 must be 0
    end

  end
end
