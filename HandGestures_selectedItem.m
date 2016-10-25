function varargout = HandGestures_selectedItem(varargin)

% global variables for storing states
global default_switch;
global default_temperature;
global default_mode;
global default_wind;

global switch_status;
global temperature_status;
global menu_item;
global mode_option_item;
global wind_option_item;

% set the default status, when start display them
default_switch = 0;
default_temperature = 20;
default_mode = 0; % mode = {'Cool', 'Fan', 'Dry', 'Power Saver'};
default_wind = 0; %  wind = {'Automatic', 'High', 'Medium', 'Low'};

% record current status, so that change when get an order
switch_status = default_switch;
temperature_status = default_temperature;
mode_option_item = default_mode;
wind_option_item = default_wind;
menu_item = 0;

order_seq = {0, 1, 2, 3, 4, 0, 0, 3, 4, 1, 2, 3, 3, 4, 0};

length = size(order_seq);
i=0;

current_item = 2;

while(i < length(2))
    mattest = cell2mat(order_seq);
    order = mattest(i+1);

    if (order == 1) % Up, -1 
        current_item = mod(current_item -1 + 4, 4);
        if (current_item == 1)
            current_item = 4;
        end
    elseif(order == 2) % Down
        current_item = mod(current_item + 1, 4);
        if (current_item == 1)
            current_item = 2;
        end
    elseif(order == 0) % On
        if(switch_status == 1) % if on and turn off, then set to 1
            current_item = 1;
        else % if Off and trun on, then set to 2
            current_item = 2;
        end
    end

out = set_status(order);
disp1 = out(1);
drawnow update



% head caption
uicontrol('Style','text',...
'String', 'Status:',...
'Position',[50 265 200 50],...
'HorizontalAlignment', 'left');

uicontrol('Style','Listbox',...
'String', disp1{1},...
'Position',[50 100 200 200],...
'Value', current_item);

uicontrol('Style','text',...
'String', 'User''s Current Gesture Is: ',...
'Position',[300 265 200 50],...
'HorizontalAlignment', 'left');

uicontrol('Style','Listbox',...
'String', out(2),...
'Position',[300 100 200 200]);

pause(1);
    
    i = i + 1;
end




end


function varoutput = set_status(order)

% global variables for storing states
global default_switch;
global default_temperature;
global default_mode;
global default_wind;

global switch_status;
global temperature_status;
global menu_item;
global mode_option_item;
global wind_option_item;

mode = {'Mode: Cool', 'Mode: Fan', 'Mode: Dry', 'Mode: Power Saver'};
wind = {'Air Flow speed: Automatic', 'Air Flow speed: High', 'Air Flow speed: Medium', 'Air Flow speed: Low'};
operation = {'On/Off', 'Up', 'Down', 'Left', 'Right'};

% get the start/stop commander
if (order == 0)
    % current is off, then start the display
    if (switch_status == 0)
        switch_status = 1;
        menu_item = 0;  % set current menu to be 'temperature'
        % now set all the display%%%%%
    % current is on, then stop the display
    else
        switch_status = 0;
        % record all of our display into default
        default_temperature = temperature_status;
        default_mode = mode_option_item;
        default_wind = wind_option_item;
    end
elseif (order == 1) % Up
   check_switch_status();    % check if it's on, otherwise don't do
   menu_item = mod(menu_item - 1 + 3, 3);
elseif (order == 2) % Down
    check_switch_status();
    menu_item = mod(menu_item+1, 3);
elseif (order == 3) % Left
    check_switch_status();
    if (menu_item == 0)
        temperature_status = temperature_status - 1;
    elseif (menu_item == 1)
        mode_option_item = mod(mode_option_item + 3, 4);
    elseif (menu_item == 2)
        wind_option_item = mod(wind_option_item + 3, 4);
    end
elseif (order == 4) % right
    check_switch_status();
    if (menu_item == 0)
        temperature_status = temperature_status + 1;
    elseif (menu_item == 1)
        mode_option_item = mod(mode_option_item + 1, 4);
    elseif (menu_item == 2)
        wind_option_item = mod(wind_option_item + 1, 4);
    end
end


operation_str = sprintf('Operation is %s', cell2mat(operation(order+1)));

if (switch_status == 0)
    varoutput_str = sprintf('switch = %d',switch_status);
    if (switch_status == 0)
        status_str1 = 'AC: Off';
    else
        status_str1 = 'AC: On';
    end
    %varoutput = {switch_status};
    varoutput = {status_str1, ' ', ' ',' '};
else
    varoutput_str = sprintf('switch = %d, temperature = %d, mode = %s, wind = %s',switch_status, temperature_status, cell2mat(mode(mode_option_item+1)), cell2mat(wind(wind_option_item+1)));
    if (switch_status == 0)
        status_str1 = 'AC: Off';
    else
        status_str1 = 'AC: On';
    end
    %varoutput = {switch_status, temperature_status, cell2mat(mode(mode_option_item+1)), cell2mat(wind(wind_option_item+1))};
    temperature_str1 = sprintf('Temperature: %d', temperature_status);
    varoutput = {status_str1, temperature_str1, cell2mat(mode(mode_option_item+1)), cell2mat(wind(wind_option_item+1))};
end
disp(operation_str);
disp(varoutput_str); 

varoutput = {varoutput, cell2mat(operation(order+1))};
end

function var = check_switch_status()
global switch_status;

if (switch_status == 0)
    disp('Warning: it is closed, please open first and then switch menus');
    %exit();
end
end

% This function mocks the output of other modules
% 0 - double push
% 1 - Up
% 2 - Down
% 3 - Left
% 4 - Right
function varout = mockOutput()
    varout = 0;
end