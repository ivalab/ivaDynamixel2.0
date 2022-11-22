%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Chi Nnoka
% Revised: A. Chang (12/2021)
% Date: 07/09/2019
% This is a modified version of the DXL_IO class that Alex Chang created for the RX-28 motors.
% 
%  WARNING  :  Do not set the motors baud rate over 3 million. 
%              USB-to-serial chipsets we currently use are limited to 
%              3 million baud; setting a higher baud rate may result in an
%              unrecoverable Dynamixel motor.
%              To be safe, stay on 1 million baud.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef DXL_IO < handle
  % This Class serves to organize the functions that command the new motors
  % Detailed explanation goes here
  
  properties (Access = protected)
    os = -1;                % 0 = linux; 1 = windows; 2 = mac os
    lib_name;               % DXL library name
    lib_file;               % Binary shared library filename
    
    port_hdl = '';
    port_dev_name = '';
    port_baud_rate = 1;
    port_open = false;
  end
  
  properties  (Abstract, Access = protected)
    PROTOCOL_VERSION;
  end

  properties (Constant)
    % Dynamixel packet instructions
    INSTR_PING          = 1;
    INSTR_READ          = 2;
    INSTR_WRITE         = 3;
    INSTR_REG_WRITE     = 4;
    INSTR_ACTION        = 5;
    INSTR_RESET         = 6;
    INSTR_SYNC_READ	    = 82;
    INSTR_SYNC_WRITE	  = 83;
    
    % Dynamixel error code masks
    %   NOTE: 8-bit flag definition differs for each Dynamixel motor family
    
    % Dynamixel broadcast motor ID
    DXL_BROADCAST_ID    = 254;
    DXL_MIN_ID          = 0;
    DXL_MAX_ID          = 253;
    
    % Dynamixel (locally-determined) communication status on last tx/rx
    COMM_TXSUCCESS  = 0;
    COMM_RXSUCCESS  = 1;
    COMM_TXFAIL     = 2;
    COMM_RXFAIL     = 3;
    COMM_TXERROR    = 4;
    COMM_RXWAITING  = 5;
    COMM_RXTIMEOUT  = 6;
    COMM_RXCORRUPT  = 7;
        
    MODEL_NUM2NAME = containers.Map( [12, 300, 18, 10, 24, 28, 64, 107, 104, 29, ...
                                      30, 310, 311, 320, 321, 350, 1060, 1030, 1020, 1130, ...
                                      1120, 1050, 1040, 1010, 1000, 35072, 37928, 37896, 38176, 38152, ...
                                      43288, 46096, 46352, 51200, 53768, 54024, 2000, 2010, 2020], ...
                                      {'AX_12A', 'AX_12W', 'AX_18A', 'RX_10', 'RX_24F', 'RX_28', 'RX_64', 'EX_106', 'MX_12W', 'MX_28', ...
                                       'MX_28_2', 'MX_64', 'MX_64_2', 'MX_106', 'MX_106_2', 'XL_320', 'XL430_W250', 'XM430_W210', 'XM430_W350', 'XM540_W150', ...
                                       'XM540_W270', 'XH430_V210', 'XH430_V350', 'XH430_W210', 'XH430_W350', 'PRO_L42_10_S300_R', 'PRO_L54_30_S400_R', 'PRO_L54_30_S500_R', 'PRO_L54_50_S290_R', 'PRO_L54_50_S500_R', ...
                                       'PRO_M42_10_S260_R', 'PRO_M54_40_S250_R', 'PRO_M54_60_S250_R', 'PRO_H42_20_S300_R', 'PRO_H54_100_S500_R', 'PRO_H54_200_S500_R', 'PRO_PLUS_H42P_020_S300_R', 'PRO_PLUS_H54P_100_S500_R', 'PRO_PLUS_H54P_200_S500_R'});
    BAUDRATE_ENC2RATE = containers.Map( [0, 1, 2, 3, 4, 5, 6, 7], ...
                                        [9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000, 4500000]);
  end
  
  properties  (Abstract, Constant)
    ANGLE_MIN;          % min. achievable motor position (rad)
    ANGLE_MAX;          % max. achievable motor position (rad)
    ENC_BIT_LEN;        % encoder count bit length

    ENC_TO_RAD;         % (rad/encoder cnts)
    ENC_HOME_POS;       % Encoder count offset for zero rad. position 
                        %   (Note: motor-dependent applicability)
  end
  
  methods  (Access = public)
    % Constructor
    function obj = DXL_IO()
      % Determine OS-appropriate Dynamixel (SDK) library name
      if strcmp(computer, 'PCWIN')          % Windows OS
        obj.os = 1;
        obj.lib_name = 'dxl_x86_c';
        obj.lib_file = 'dxl_x86_c.dll';
        fprintf('[DXL_IO] Windows (32-bit) OS detected.\n');
      elseif strcmp(computer, 'PCWIN64')    % Windows OS
        obj.os = 1;
        obj.lib_name = 'dxl_x64_c';
        obj.lib_file = 'dxl_x64_c.dll';
        fprintf('[DXL_IO] Windows (64-bit) OS detected.\n');
      elseif strcmp(computer, 'GLNX86')     % Unix OS
        obj.os = 0;
        obj.lib_name = 'libdxl_x86_c';
        obj.lib_file = 'libdxl_x86_c.so';
        fprintf('[DXL_IO] Unix (32-bit) OS detected.\n');
      elseif strcmp(computer, 'GLNXA64')    % Unix OS
        obj.os = 0;
        obj.lib_name = 'libdxl_x64_c';
        obj.lib_file = 'libdxl_x64_c.so';
        fprintf('[DXL_IO] Unix (64-bit) OS detected.\n');
      elseif strcmp(computer, 'MACI64')     % Mac OS
        obj.os = 2;
        obj.lib_name = 'libdxl_mac_c';
        obj.lib_file = 'libdxl_mac_c.dylib';
        fprintf('[DXL_IO] Mac (64-bit) OS detected.\n');
      else
        error('[DXL_IO] Unsupported operating system detected!\n');
      end

      % Update Matlab path with supporting DXL scripts/libraries/headers
      dxlio_path = which('dxl_intfcs.DXL_IO');
      if ( obj.os == 0 || obj.os == 2 )      % Unix derivative or Mac OS
        [pathstr, ~, ~] = fileparts(dxlio_path);
        addpath(genpath([pathstr '/../m_basic_function']));   % Matlab wrappers
        addpath([pathstr '/../../c/include/dynamixel_sdk']);  % C headers
        addpath([pathstr '/../../c/build/linux64']);          % Compiled library
      elseif ( obj.os == 1 )  % Windows         % TODO: Check windows addpaths are valid
        base_dir_end_ind = strfind(dxlio_path, 'matlab')-1;
        addpath(genpath([dxlio_path(1:base_dir_end_ind+6) '\m_basic_function'])); % Matlab wrappers
        addpath([dxlio_path(1:base_dir_end_ind) 'c\include\dynamixel_sdk']);      % C headers
        addpath([dxlio_path(1:base_dir_end_ind) 'c\build\win64']);                % Compiled library
      end
    end
    

    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Matlab compiled-library management
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function load_library( obj )
      if ( ~libisloaded(obj.lib_name) )
        [notfound, warnings] = loadlibrary(obj.lib_name, 'dynamixel_sdk.h', ...
                                            'addheader', 'port_handler.h', ...
                                            'addheader', 'packet_handler.h', ...
                                            'addheader', 'group_sync_write.h', ...
                                            'addheader', 'group_sync_read.h', ...
                                            'addheader', 'group_bulk_write.h', ...
                                            'addheader', 'group_bulk_read.h');
      else
        warning('[DXL_IO] Library is already loaded: %s.', obj.lib_name);
      end
    end
    
    %   Unload dynamixel library
    function unload_library( obj )
      unloadlibrary(obj.lib_name);
    end
    

    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Port configuration
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %   Serial connection via FTDI adapter
    %   Windows:= 'COM<##>'
    %   LINUX = '/dev/ttyUSB<##>'
    %
    %     Set port = <##>
    %     Set baud according to below table:
    %       Value:        Goal BPS:
    %       7             4.5M
    %       6             4.0M
    %       5             3.0M
    %       4             2.0M
    %       3             1.0M
    %       2             115200.0
    %       1              57600.0 (FACTORY DEFAULT)
    %       0               9600.0    
    function [VALUE] = openPort( obj, a_port_dev, a_baud_rate )
      obj.is_port_closed( 'openPort()' );  % assert a port is not already open
      
      obj.port_hdl = obj.portHandler(a_port_dev);  % handle to port device
      
      obj.packetHandler();          % initialze port-associated data structs
      
      VALUE = openPort( obj.port_hdl );  % open port
      obj.port_open = logical(VALUE);
      obj.port_dev_name = a_port_dev;
      
      % Set baud rate if specified
      if ( nargin > 2 )
        setBaudResult = obj.setBaudRate( a_baud_rate );
      end      
    end
    
    function closePort( obj )
      obj.is_port_open( 'closePort()' );  % assert a port is currently open
      
      closePort(obj.port_hdl);
      obj.port_open = false;
    end

    function [VALUE] = portHandler(obj, a_port_dev )    % TODO: protected
      VALUE = portHandler( a_port_dev );
    end

    function [result] = setBaudRate( obj, a_baud_rate )
      obj.is_port_open( 'setBaudRate()' );  % assert a port is currently open
      
      result = setBaudRate( obj.port_hdl, a_baud_rate );
      if ( result )
        obj.port_baud_rate = a_baud_rate;
      end
    end
    
    function [VALUE] = getBaudRate( obj )
      obj.is_port_open( 'getBaudRate()' );  % assert a port is currently open
      
      VALUE = getBaudRate( obj.port_hdl );
    end

    function [] = setPortName( obj, a_port_name )
      obj.is_port_open( 'setPortName()' );  % assert a port is currently open
      
      setPortName( obj.port_hdl, a_port_name );
    end
    
    function [STRING] = getPortName( obj )
      obj.is_port_open( 'getPortName()' );  % assert a port is currently open
      
      STRING = getPortName( obj.port_hdl );
    end
    
    function [] = setPacketTimeout( obj, a_packet_length )
      obj.is_port_open( 'setPacketTimeout()' );  % assert a port is currently open
      
      setPacketTimeout( obj.port_hdl, a_packet_length );
    end
    
    function [] = setPacketTimeoutMSec( obj, a_msec )
      obj.is_port_open( 'setPacketTimeoutMSec()' );  % assert a port is currently open
      
      setPacketTimeoutMSec( obj.port_hdl, a_msec );
    end
    
    function [VALUE] = isPacketTimeout( obj )
      obj.is_port_open( 'isPacketTimeout()' );  % assert a port is currently open
      
      VALUE = isPacketTimeout( obj.port_hdl );
    end

    %   Initialize underlying port-associated structs (required after call to obj.portHandler())
    function packetHandler( obj )       % TODO: Protected
      packetHandler();
    end
 

    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Multi-motor management utilities
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function [] = scanForMotors( obj, a_id_list )
      obj.is_port_open( 'scanForMotors()' );  % assert a port is currently open

      %   Scan motor IDs
      fprintf('Scan results: \n Port: %s\n Baud Rate: %d bps\n==================\n', obj.port_dev_name, obj.port_baud_rate);
      for ii = 1:length(a_id_list)
        ping_result = obj.pingGetModelNum( a_id_list(ii) );
        if ( ~ping_result )
          fprintf('[not found] Motor ID: %d -> no response.\n', a_id_list(ii));  
        else
          fprintf('[FOUND] Motor ID: %d -> Model number: %d (%s).\n', a_id_list(ii), ping_result, obj.MODEL_NUM2NAME(ping_result));
        end
        pause(0.2);
      end
      fprintf('\n');
    end


    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Dynamixel high-level motor interface (e.g. packet construction, Tx/Rx)
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % ==== Basic/common motor capabilities ====
    function [] = ping( obj, a_id )
      obj.is_port_open( 'ping()' );  % assert a port is currently open
      
      ping( obj.port_hdl, obj.PROTOCOL_VERSION, a_id );
    end
    
    function [VALUE] = pingGetModelNum( obj, a_id )
      obj.is_port_open( 'pingGetModelNum()' );  % assert a port is currently open
      
      VALUE = pingGetModelNum( obj.port_hdl, obj.PROTOCOL_VERSION, a_id );
    end
    
    function [] = broadcastPing( obj )
      obj.is_port_open( 'broadcastPing()' );  % assert a port is currently open
      
      broadcastPing( obj.port_hdl, obj.PROTOCOL_VERSION );
    end
    
    function [VALUE] = getBroadcastPingResult( obj, a_id )
      obj.is_port_open( 'getBroadcastPingResult()' );  % assert a port is currently open
      
      VALUE = getBroadcastPingResult( obj.port_hdl, obj.PROTOCOL_VERSION, a_id );
    end
    
    % Reset the present position control table value to absolute value 
    % within one rotation
    %   Applied products: MX with DYNAMIXEL Protocol 2.0 (Firmware v42 or above), 
    %                     DYNAMIXEL-X series (Firmware v42 or above)
    %
    % Input(s):
    %   a_id:             motor ID
    function [] = clearMultiTurn( obj, a_id )
      obj.is_port_open( 'clearMultiTurn()' );  % assert a port is currently open
      
      clearMultiTurn( obj.port_hdl, obj.PROTOCOL_VERSION, a_id );
    end
    
    % Reset motor control table to default factory settings
    %
    % Input(s):
    %   a_id:             motor ID
    %   a_option:         (0xFF) Reset all, 
    %                     (0x01) Reset all except ID, 
    %                     (0x02) Reset all except ID and Baudrate
    function [] = factoryReset( obj, a_id, a_option )
      obj.is_port_open( 'factoryReset()' );  % assert a port is currently open
      
      factoryReset( obj.port_hdl, obj.PROTOCOL_VERSION, a_id, a_option );
    end

    % Reboot motor
    %
    % Input(s):
    %   a_id:             motor ID
    function [] = reboot( obj, a_id )
      obj.is_port_open( 'reboot()' );  % assert a port is currently open
      
      reboot( obj.port_hdl, obj.PROTOCOL_VERSION, a_id );
    end
    
    % ==== Packet Tx/Rx status ==== 
    %   Last Tx/Rx comm. result (i.e. COMM_*)
    % TODO: assert port open
    function [VALUE] = getLastTxRxResult( obj )
      obj.is_port_open( 'getLastTxRxResult()' );  % assert a port is currently open
      
      VALUE = getLastTxRxResult( obj.port_hdl, obj.PROTOCOL_VERSION );
    end
    
    %   Convert comm. result to string message
    function [STRING] = getTxRxResult( obj, a_result )
      STRING = getTxRxResult( obj.PROTOCOL_VERSION, a_result );
    end

    %   Last Rx packet error (i.e. ERRRBIT_*)
    function [VALUE] = getLastRxPacketError( obj )
      obj.is_port_open( 'getLastRxPacketError()' );  % assert a port is currently open
      
      VALUE = getLastRxPacketError( obj.port_hdl, obj.PROTOCOL_VERSION );
    end

    %   Convert error result to string message
    function [STRING] = getRxPacketError( obj, a_error )
      STRING = getRxPacketError( obj.PROTOCOL_VERSION, a_error );
    end
    
    % ==== Read motor data ==== 
    %   READ 1 byte
    function [VALUE] = read1ByteTxRx( obj, a_id, a_address )
      obj.is_port_open( 'read1ByteTxRx()' );  % assert a port is currently open
      
      VALUE = read1ByteTxRx( obj.port_hdl, obj.PROTOCOL_VERSION, a_id, a_address );
    end

    function [] = read1ByteTx( obj, a_id, a_address )
      obj.is_port_open( 'read1ByteTx()' );  % assert a port is currently open
      
      read1ByteTx( obj.port_hdl, obj.PROTOCOL_VERSION, a_id, a_address );
    end
    
    function [VALUE] = read1ByteRx( obj )
      obj.is_port_open( 'read1ByteRx()' );  % assert a port is currently open
      
      VALUE = read1ByteRx( obj.port_hdl, obj.PROTOCOL_VERSION );
    end

    %   READ 2 bytes
    function [VALUE] = read2ByteTxRx( obj, a_id, a_address )
      obj.is_port_open( 'read2ByteTxRx()' );  % assert a port is currently open
      
      VALUE = read2ByteTxRx( obj.port_hdl, obj.PROTOCOL_VERSION, a_id, a_address );
    end
    
    function [] = read2ByteTx( obj, a_id, a_address )
      obj.is_port_open( 'read2ByteTx()' );  % assert a port is currently open
      
      read2ByteTx( obj.port_hdl, obj.PROTOCOL_VERSION, a_id, a_address );
    end
        
    function [VALUE] = read2ByteRx( obj )
      obj.is_port_open( 'read2ByteRx()' );  % assert a port is currently open
      
      VALUE = read2ByteRx( obj.port_hdl, obj.PROTOCOL_VERSION );
    end

    %   READ 4 byte
    function [VALUE] = read4ByteTxRx( obj, a_id, a_address )
      obj.is_port_open( 'read4ByteTxRx()' );  % assert a port is currently open
      
      VALUE = read4ByteTxRx( obj.port_hdl, obj.PROTOCOL_VERSION, a_id, a_address );
    end
    
    function [] = read4ByteTx( obj, a_id, a_address )
      obj.is_port_open( 'read4ByteTx()' );  % assert a port is currently open
      
      read4ByteTx( obj.port_hdl, obj.PROTOCOL_VERSION, a_id, a_address );
    end
    
    function [VALUE] = read4ByteRx( obj )
      obj.is_port_open( 'read4ByteRx()' );  % assert a port is currently open
      
      VALUE = read4ByteRx( obj.port_hdl, obj.PROTOCOL_VERSION );
    end
    
    % ==== Write motor data ==== 
    %   WRITE 1 byte
    function [] = write1ByteTxRx( obj, a_id, a_address, a_data )
      obj.is_port_open( 'write1ByteTxRx()' );  % assert a port is currently open
      
      write1ByteTxRx( obj.port_hdl, obj.PROTOCOL_VERSION, a_id, a_address, a_data );
    end
    
    function [] = write1ByteTxOnly( obj, a_id, a_address, a_data )
      obj.is_port_open( 'write1ByteTxOnly()' );  % assert a port is currently open
      
      write1ByteTxOnly( obj.port_hdl, obj.PROTOCOL_VERSION, a_id, a_address, a_data );
    end
   
    %   WRITE 2 bytes
    function [] = write2ByteTxRx( obj, a_id, a_address, a_data )
      obj.is_port_open( 'write2ByteTxRx()' );  % assert a port is currently open
      
      write2ByteTxRx( obj.port_hdl, obj.PROTOCOL_VERSION, a_id, a_address, a_data );
    end
    
    function [] = write2ByteTxOnly( obj, a_id, a_address, a_data )
      obj.is_port_open( 'write2ByteTxOnly()' );  % assert a port is currently open
      
      write2ByteTxOnly( obj.port_hdl, obj.PROTOCOL_VERSION, a_id, a_address, a_data );
    end
    
    %   WRITE 4 bytes
    function [] = write4ByteTxRx( obj, a_id, a_address, a_data )
      obj.is_port_open( 'write4ByteTxRx()' );  % assert a port is currently open
      
      write4ByteTxRx( obj.port_hdl, obj.PROTOCOL_VERSION, a_id, a_address, a_data );
    end
    
    function [] = write4ByteTxOnly( obj, a_id, a_address, a_data )
      obj.is_port_open( 'write4ByteTxOnly()' );  % assert a port is currently open
      
      write4ByteTxOnly( obj.port_hdl, obj.PROTOCOL_VERSION, a_id, a_address, a_data );
    end
    
    % ==== REG_WRITE ==== 
    %   (synchronized execution by multiple motors upon send/receipt 
    %     of ACTION instruction)
    function [] = regWriteTxOnly( obj, a_id, a_address, a_length )
      obj.is_port_open( 'regWriteTxOnly()' );  % assert a port is currently open
      
      regWriteTxOnly( obj.port_hdl, obj.PROTOCOL_VERSION, a_id, a_address, a_length );
    end
    
    function [] = regWriteTxRx( obj, a_id, a_address, a_length )
      obj.is_port_open( 'regWriteTxRx()' );  % assert a port is currently open
      
      regWriteTxRx( obj.port_hdl, obj.PROTOCOL_VERSION, a_id, a_address, a_length );
    end
    
    % ==== Group Sync read ====
    % NOTE(s): Once created, each group ID cannot be removed and is 
    % associated with a specified 'start' destination register address and 
    % data length (i.e. # bytes to read/write)

    % NOTE(s): Currently limited to reading one register address at a time
    % [TODO: extend to read multiple consecutive register addresses (i.e. 
    %   allow a_data_length to be array of sequenced byte lengths; one 
    %   length for each address)]
    function [ groupSyncReadData ] = groupSyncReadAddr( obj, a_motor_ids, a_start_addresss, a_data_length )
      group_id = obj.groupSyncRead( a_start_addresss, a_data_length );
      
      % Add motor IDs to read group
      for ii = 1:length(a_motor_ids)        
        if ( ~obj.groupSyncReadAddParam( group_id, a_motor_ids(ii) ) )
          error('[ERROR] DXL_IO::groupSyncReadAddr(): motor ID failed to be added to read group (%d) ...', a_motor_ids(ii));
        end
      end
      
      obj.groupSyncReadTxRxPacket( group_id );
      
      % Retrieve read group data
      groupSyncReadData = zeros(size(a_motor_ids));
      for ii = 1:length(a_motor_ids)        
        groupSyncReadData(ii) = obj.groupSyncReadGetData( group_id, a_motor_ids(ii), a_start_addresss, a_data_length );
      end
      
      obj.groupSyncReadClearParam( group_id );        % clear read group data (permits group ID re-use)
    end
    
    %   Create read group and initialize read group-associated structs
    function [group_id] = groupSyncRead( obj, a_start_addresss, a_data_length )
      obj.is_port_open( 'groupSyncRead()' );  % assert a port is currently open
      
      group_id = groupSyncRead( obj.port_hdl, obj.PROTOCOL_VERSION, a_start_addresss, a_data_length );
    end
    
    %   Add motor ID to read group
    function [VALUE] = groupSyncReadAddParam( obj, a_group_id, a_id )
      VALUE = groupSyncReadAddParam( a_group_id, a_id );
    end
    
    %   Remove motor ID from read group
    function [] = groupSyncReadRemoveParam( obj, a_group_id, a_id )
      groupSyncReadRemoveParam( a_group_id, a_id );
    end
    
    %   Clear all motor IDs associated with read group
    function [] = groupSyncReadClearParam( obj, a_group_id )
      groupSyncReadClearParam( a_group_id );
    end
    
    %   Group Sync read Tx/Rx
    function [] = groupSyncReadTxRxPacket( obj, a_group_id )
      groupSyncReadTxRxPacket( a_group_id );
    end
    
    function [] = groupSyncReadTxPacket( obj, a_group_id )
      groupSyncReadTxPacket( a_group_id );
    end
    
    function [] = groupSyncReadRxPacket( obj, a_group_id )
      groupSyncReadRxPacket( a_group_id );
    end
    
    %   Retrieve Group Sync read received data
    function [VALUE] = groupSyncReadIsAvailable( obj, a_group_id, a_id, a_address, a_data_length )
      VALUE = groupSyncReadIsAvailable( a_group_id, a_id, a_address, a_data_length );
    end
    
    function [VALUE] = groupSyncReadGetData( obj, a_group_id, a_id, a_address, a_data_length )
      VALUE = groupSyncReadGetData( a_group_id, a_id, a_address, a_data_length );
    end

    % ==== Group Sync write ====
    % NOTE(s): Currently limited to writing one register address at a time
    % [TODO: extend to write multiple consecutive register addresses (i.e. 
    %   allow a_data_length to be array of sequenced byte lengths; one 
    %   length for each address)]
    function [] = groupSyncWriteAddr( obj, a_motor_ids, a_data, a_start_addresss, a_data_length )
      group_id = obj.groupSyncWrite( a_start_addresss, a_data_length );

      % Add motor IDs to write group
      for ii = 1:length(a_motor_ids)
        if ( ~obj.groupSyncWriteAddParam( group_id, a_motor_ids(ii), a_data(ii), a_data_length ) )
          error('[DXL_IO::groupSyncWriteAddr()] Motor data failed to be added to write group (%d) ...', a_motor_ids(ii));
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
    
    %   Create write group and initialize write group-associated structs
    function [group_id] = groupSyncWrite( obj, a_start_addresss, a_data_length )
      obj.is_port_open( 'groupSyncWrite()' );  % assert a port is currently open
      
      group_id = groupSyncWrite( obj.port_hdl, obj.PROTOCOL_VERSION, a_start_addresss, a_data_length );
    end
    
    %   Add motor ID,  data to write group
    function [VALUE] = groupSyncWriteAddParam(obj, a_group_id, a_id, a_data, a_data_length )
      VALUE = groupSyncWriteAddParam( a_group_id, a_id, a_data, a_data_length );
    end
    
    %   Repace data beginning at a_data_pos byte num. for motor ID, with a_data (of length, a_data_length bytes)
    function [VALUE] = groupSyncWriteChangeParam( obj, a_group_id, a_id, a_data, a_data_length, a_data_pos )
      VALUE = groupSyncWriteChangeParam( a_group_id, a_id, a_data, a_data_length, a_data_pos );
    end

    %   Remove motor ID and data from write group
    function [] = groupSyncWriteRemoveParam( obj, a_group_id, a_id )
      groupSyncWriteRemoveParam( a_group_id, a_id );
    end
    
    %   Clear all motor IDs and data associated with write group
    function [] = groupSyncWriteClearParam( obj, a_group_id )
      groupSyncWriteClearParam( a_group_id );
    end

    %   Group Sync write Tx
    function [] = groupSyncWriteTxPacket( obj, a_group_id )
      groupSyncWriteTxPacket( a_group_id );
    end    
    
    % ==== Group Bulk read ====
    % TODO: Add wrapper functions for group_bulk_write and group_bulk__read
    % Create read group and initialize read group-associated structs
    function [group_id] = groupBulkRead( obj )
      obj.is_port_open( 'groupBulkRead()' );  % assert a port is currently open
      
      group_id = groupBulkRead( obj.port_hdl, obj.PROTOCOL_VERSION );
    end
    
    %   Add motor ID, address and num. bytes to read group
    function [VALUE] = groupBulkReadAddParam( obj, a_group_id, a_id, a_start_addresss, a_data_length )
      VALUE = groupBulkReadAddParam( a_group_id, a_id, a_start_addresss, a_data_length );
    end
    
    %   Remove motor ID from read group
    function [] = groupBulkReadRemoveParam( obj, a_group_id, a_id )
      groupBulkReadRemoveParam( a_group_id, a_id );
    end
    
    %   Clear all motor IDs associated with read group
    function [] = groupBulkReadClearParam( obj, a_group_id )
      groupBulkReadClearParam( a_group_id );
    end
    
    %   Group Bulk read Tx/Rx
    function [] = groupBulkReadTxRxPacket( obj, a_group_id )
      groupBulkReadTxRxPacket( a_group_id );
    end
    
    function [] = groupBulkReadTxPacket( obj, a_group_id )
      groupBulkReadTxPacket( a_group_id );
    end
    
    function [] = groupBulkReadRxPacket( obj, a_group_id )
      groupBulkReadRxPacket( a_group_id );
    end
    
    %   Retrieve Group Bulk read received data
    function [VALUE] = groupBulkReadIsAvailable( obj, a_group_id, a_id, a_address, a_data_length)
      VALUE = groupBulkReadIsAvailable( a_group_id, a_id, a_address, a_data_length );
    end
    
    function [VALUE] = groupBulkReadGetData( obj, a_group_id, a_id, a_address, a_data_length)
      VALUE = groupBulkReadGetData( a_group_id, a_id, a_address, a_data_length );
    end

    % ==== Group Bulk write ====
    %   Create write group and initialize write group-associated structs
    function [group_id] = groupBulkWrite( obj )
      obj.is_port_open( 'groupBulkWrite()' );  % assert a port is currently open
      
      group_id = groupBulkWrite( obj.port_hdl, obj.PROTOCOL_VERSION );
    end
    
    %   Add motor ID, register address and data to write group
    function [VALUE] = groupBulkWriteAddParam( obj, a_group_id, a_id, a_start_addresss, a_data_length, a_data, a_input_length )
      VALUE = groupBulkWriteAddParam( a_group_id, a_id, a_start_addresss, a_data_length, a_data, a_input_length );
    end
    
    %   Repace data beginning at a_data_pos byte num. for motor ID, for a_data_length bytes, with a_data
    function [VALUE] = groupBulkWriteChangeParam( obj, a_group_id, a_id, a_start_addresss, a_data_length, a_data, a_input_length, a_data_pos )
      VALUE = groupBulkWriteChangeParam( a_group_id, a_id, a_start_addresss, a_data_length, a_data, a_input_length, a_data_pos );
    end

    %   Remove motor ID and data from write group
    function [] = groupBulkWriteRemoveParam( obj, a_group_id, a_id )
      groupBulkWriteRemoveParam( a_group_id, a_id );
    end
    
    %   Clear all motor IDs and data associated with write group
    function [] = groupBulkWriteClearParam( obj, a_group_id )
      groupBulkWriteClearParam( a_group_id );
    end
    
    %   Group Bulk write Tx
    function [] = groupBulkWriteTxPacket( obj, a_group_id )
      groupBulkWriteTxPacket( a_group_id );
    end


    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Dynamixel mid-level motor interface 
    %   (READ/WRITE, e.g. used by writeXByteTx)
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    function [] = writeTxRx( obj, a_id, a_address, a_length )
      obj.is_port_open( 'writeTxRx()' );  % assert a port is currently open
      
      writeTxRx( obj.port_hdl, obj.PROTOCOL_VERSION, a_id, a_address, a_length );
    end
    
    function [] = writeTxOnly( obj, a_id, a_address, a_length )
      obj.is_port_open( 'writeTxOnly()' );  % assert a port is currently open
      
      writeTxOnly( obj.port_hdl, obj.PROTOCOL_VERSION, a_id, a_address, a_length );
    end
    
    function [] = readTxRx( obj, a_id, a_address, a_length )
      obj.is_port_open( 'readTxRx()' );  % assert a port is currently open
      
      readTxRx( obj.port_hdl, obj.PROTOCOL_VERSION, a_id, a_address, a_length );
    end
    
    function [] = readTx( obj, a_id, a_address, a_length )
      obj.is_port_open( 'readTx()' );  % assert a port is currently open
      
      readTx( obj.port_hdl, obj.PROTOCOL_VERSION, a_id, a_address, a_length );
    end

    function [] = readRx( obj, a_length )
      obj.is_port_open( 'readRx()' );  % assert a port is currently open
      
      readRx( obj.port_hdl, obj.PROTOCOL_VERSION, a_length );
    end
    
    % ==== Bulk read/write ==== 
    function [] = bulkReadTx(obj, a_param_length )
      obj.is_port_open( 'bulkReadTx()' );  % assert a port is currently open
      
      bulkReadTx( obj.port_hdl, obj.PROTOCOL_VERSION, a_param_length );
    end
    
    function [] = bulkWriteTxOnly( obj, a_param_length )
      obj.is_port_open( 'bulkWriteTxOnly()' );  % assert a port is currently open
      
      bulkWriteTxOnly( obj.port_hdl, obj.PROTOCOL_VERSION, a_param_length );
    end
    
    % ==== Sync read/write ==== 
    %   Read/Write address(es) acrosss multiple motors 
    function [] = syncReadTx( obj, a_start_address, a_data_length, a_param_length )
      syncReadTx( obj.PROTOCOL_VERSION, a_start_address, a_data_length, a_param_length );
    end
    
    function [] = syncWriteTxOnly( obj, a_start_address, a_data_length, a_param_length )
      obj.is_port_open( 'syncWriteTxOnly()' );  % assert a port is currently open
      
      syncWriteTxOnly( obj.port_hdl, obj.PROTOCOL_VERSION, a_start_address, a_data_length, a_param_length );
    end
    
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Dynamixel low-level motor interface 
    %   (Packet send/receive, e.g. used by mid-level Tx/Rx methods)
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function [] = txrxPacket( obj )
      obj.is_port_open( 'txrxPacket()' );  % assert a port is currently open
      
      txRxPacket( obj.port_hdl, obj.PROTOCOL_VERSION );
    end
    
    function [] = txPacket( obj )
      obj.is_port_open( 'txPacket()' );  % assert a port is currently open
      
      txPacket( obj.port_hdl, obj.PROTOCOL_VERSION );
    end

    function [] = rxPacket( obj )
      obj.is_port_open( 'rxPacket()' );  % assert a port is currently open
      
      rxPacket( obj.port_hdl, obj.PROTOCOL_VERSION );
    end
    
    %   Return current Rx data bytes (re-seuqenced) as single (multi-byte) value
    function [VALUE] = getDataRead(obj, a_data_length, a_data_pos )
      obj.is_port_open( 'getDataRead()' );  % assert a port is currently open
      
      VALUE = getDataRead( obj.port_hdl, obj.PROTOCOL_VERSION, a_data_length, a_data_pos );
    end

    %   Set current Tx data, specifying byte length (i.e. a_data_length)
    function [] = setDataWrite(obj, a_data_length, a_data_pos, a_data)
      obj.is_port_open( 'setDataWrite()' );  % assert a port is currently open
      
      setDataWrite( obj.port_hdl, obj.PROTOCOL_VERSION, a_data_length, a_data_pos, a_data );
    end


    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Generic utilities    
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %   Convert position (encoder count to rad)
    function [result] = pos_enc_to_rad( obj, a_enc_pos )    % TODO: change implementation to use ref rad & enc cnt? implement specialized version in subclass? [Note: (4095-2048)*180/2048]
      result = a_enc_pos*obj.ENC_TO_RAD;
    end

    %   Convert position (rad to encoder count)
    function [result] = pos_rad_to_enc( obj, a_rad_pos )
      result = floor(a_rad_pos/obj.ENC_TO_RAD);
    end

    %   Convert speed (encoder counts/sec to rad/s)
    function [result] = speed_enc_to_rad( obj, a_enc_speed )
      result = a_enc_speed*obj.ENC_TO_RAD;
    end

    %   Convert speed (rad/s to encoder counts/sec)
    function [result] = speed_rad_to_enc( obj, a_rad_speed )    
      result = a_rad_speed/obj.ENC_TO_RAD;
    end
  end
  
  methods (Access=protected)
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Helpers
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function is_port_open( obj, src_func_str )
      assert( obj.port_open, '[DXL_IO::%s] No port currently open.', src_func_str);
    end

    function is_port_closed( obj, src_func_str )
      assert( ~obj.port_open, '[DXL_IO::%s] A port is already open.', src_func_str);
    end

    function [result] = valid_motor_id( obj, a_motor_ids )
      result = (a_motor_ids >= obj.DXL_MIN_ID & a_motor_ids <= obj.DXL_MAX_ID);
    end
  end

  methods (Abstract)

  end
end

