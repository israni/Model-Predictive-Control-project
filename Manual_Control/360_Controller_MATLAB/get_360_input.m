%%
function [d_f, F_x, Esc] = get_360_input()
 	% Get State of Controller
    persistent initialized controllerLibrary myController;
    
    if(isempty(initialized))
        controllerLibrary = NET.addAssembly([pwd '\Manual_Control\360_Controller_MATLAB\SharpDX.XInput.dll']);
        myController = SharpDX.XInput.Controller(SharpDX.XInput.UserIndex.One);
        initialized = 1;
    end
    
  
    
    State = myController.GetState();

    % Get Throttle / Braking
    leftTrigger = double(State.Gamepad.LeftTrigger);
    rightTrigger = double(State.Gamepad.RightTrigger);
    if(leftTrigger > 0) 
        F_x = leftTrigger / 255 * -4000; % Braking (-10,000 max)
    else                
        F_x = rightTrigger / 255 * 3000;  % Not Braking (6,000 max)
    end
    
    % Get Steering Input
    leftX = double(State.Gamepad.LeftThumbX) / 32768;
    d_f = leftX * -0.3; % +/- 0.5 max
    
    % Get Exit Command
    rightY = double(State.Gamepad.RightThumbY) / 32767;
    if(rightY < -0.99)
        Esc = 1;
    else
        Esc = 0;
    end
end