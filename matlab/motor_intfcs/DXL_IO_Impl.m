%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Alex Chang
% Date: 12/2021
% DXL_IO_Impl implements the (abstract) DXL_IO interface class. It 
% facilitates access to basic functionality, that is common across several 
% Dynamixel motor families.
%
%  WARNING  :  Do not set the motors baud rate over 3 million. 
%              USB-to-serial chipsets we currently use are limited to 
%              3 million baud; setting a higher baud rate may result in an
%              unrecoverable Dynamixel motor.
%              To be safe, stay on 1 million baud.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef DXL_IO_Impl < DXL_IO
  % This Class serves to organize the functions that command the new motors
  % Detailed explanation goes here

  properties (Access = protected)
    % Most properties N/A in implementated DXL_IO base class
    PROTOCOL_VERSION = 2.0;   % default DXL comm. protocol
    
    ANGLE_MIN = -1;
    ANGLE_MAX = -1;
    ENC_BIT_LEN = -1;

    ENC_TO_RAD = -1;
    ENC_HOME_POS = -1;
  end
  
  properties (Constant)

  end
  
  methods  (Access = public)
    % Constructor & initialization
    function obj = DXL_IO_Impl( a_protocol )      
      obj@DXL_IO(); 
      
      if ( nargin > 0 )   % Set DXL comm. protocol based on user input; otherwise leave at default value
        if ( a_protocol == 1 || a_protocol == 2 )
          obj.set_dxl_protocol( a_protocol );
        else
          warning('[DXL_IO_Impl::DXL_IO_Impl()] Invalid protocol specified: %0.1f. Defaulting to DXL protocol 2.0.', a_protocol);
          obj.set_dxl_protocol( 2.0 );
        end
      end
    end

    % Set DXL comm. protocol based on user input
    function set_dxl_protocol( obj, a_protocol )
      assert(a_protocol == 1 || a_protocol == 2, ...
              '[DXL_IO_Impl::set_dxl_protocol()] Invalid protocol specified: %0.1f.', a_protocol);

      obj.PROTOCOL_VERSION = a_protocol;
    end
  end
end
