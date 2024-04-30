function varargout = Guide_Project(varargin)
    % GUIDE_PROJECT MATLAB code for Guide_Project.fig
    %      GUIDE_PROJECT, by itself, creates a new GUIDE_PROJECT or raises the existing
    %      singleton*.
    %
    %      H = GUIDE_PROJECT returns the handle to a new GUIDE_PROJECT or the handle to
    %      the existing singleton*.
    %
    %      GUIDE_PROJECT('CALLBACK',hObject,eventData,handles,...) calls the local
    %      function named CALLBACK in GUIDE_PROJECT.M with the given input arguments.
    %
    %      GUIDE_PROJECT('Property','Value',...) creates a new GUIDE_PROJECT or raises the
    %      existing singleton*.  Starting from the left, property value pairs are
    %      applied to the GUI before Guide_Project_OpeningFcn gets called.  An
    %      unrecognized property name or invalid value makes property application
    %      stop.  All inputs are passed to Guide_Project_OpeningFcn via varargin.
    %
    %      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
    %      instance to run (singleton)".
    %
    % See also: GUIDE, GUIDATA, GUIHANDLES
    
    % Edit the above text to modify the response to help Guide_Project
    
    % Last Modified by GUIDE v2.5 29-Apr-2024 16:37:40
    
    % Begin initialization code - DO NOT EDIT
    gui_Singleton = 1;
    gui_State = struct('gui_Name',       mfilename, ...
                       'gui_Singleton',  gui_Singleton, ...
                       'gui_OpeningFcn', @Guide_Project_OpeningFcn, ...
                       'gui_OutputFcn',  @Guide_Project_OutputFcn, ...
                       'gui_LayoutFcn',  [] , ...
                       'gui_Callback',   []);
    if nargin && ischar(varargin{1})
        gui_State.gui_Callback = str2func(varargin{1});
    end
    
    if nargout
        [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
    else
        gui_mainfcn(gui_State, varargin{:});
    end
    % End initialization code - DO NOT EDIT
end    

% --- Executes just before Guide_Project is made visible.
function Guide_Project_OpeningFcn(hObject, eventdata, handles, varargin)
    % This function has no output args, see OutputFcn.
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to Guide_Project (see VARARGIN)
    clc;
    
    global prev_pos;
    % pos = prev_position([0 0 0]',"save");
    prev_pos = [0.4115 0.1501 0.4331]';
    robot = make_robot();
    axes(handles.axes1);
    robot.plot(zeros(1,6));
    
    
    % Choose default command line output for Guide_Project
    handles.output = hObject;
    
    % Update handles structure
    guidata(hObject, handles);
    
    % UIWAIT makes Guide_Project wait for user response (see UIRESUME)
    % uiwait(handles.figure1);
end 

% --- Outputs from this function are returned to the command line.
function varargout = Guide_Project_OutputFcn(hObject, eventdata, handles) 
    % varargout  cell array for returning output args (see VARARGOUT);
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    
    % Get default command line output from handles structure
    varargout{1} = handles.output;
end 

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
    % hObject    handle to pushbutton1 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    X = str2double(get(handles.editX,"String"));
    Y = str2double(get(handles.editY,"String"));
    Z = str2double(get(handles.editZ,"String"));
    h = warndlg('warning','title');
    % printcallbacks(X,Y,Z);
    % init_point = prev_position([0 0 0]', "get");
    clc;
    cla(handles.axes2);
    cla(handles.axes3);
    cla(handles.axes4);
    cla(handles.axes5);
    global prev_pos;
    init_point = prev_pos;
    path = [init_point [X Y Z]'];
    [t_acc, jointpos, jointvel, jointacc, tau_acc] = point2point(path);

    robot = make_robot();
    
    l = size(t_acc,2);
    for ii = 1:100:l
        axes(handles.axes1);
        robot.plot(jointpos(:,ii)');
    
        axes(handles.axes2);
        hold on;
        plot(t_acc(1:100:ii), jointpos(1,1:100:ii), 'Linewidth', 2, 'Color','r');
        plot(t_acc(1:100:ii), jointpos(2,1:100:ii), 'Linewidth', 2, 'Color','g');
        plot(t_acc(1:100:ii), jointpos(3,1:100:ii), 'Linewidth', 2, 'Color','b');
        % hold off;
        title('Position Profiles');
        xlabel('Time [s]'), ylabel('Torque [Nm]');
        xlim([t_acc(1), t_acc(end)]);
        ylims = [min(jointpos(1,:)), max(jointpos(1,:)),min(jointpos(2,:)), max(jointpos(2,:)),min(jointpos(3,:)), max(jointpos(3,:))];
        ylim([min(ylims), max(ylims)]);
        legend({'Joint 1', 'Joint 2', 'Joint 3'});
        hold off;
    
        axes(handles.axes3);
        hold on;
        plot(t_acc(1:100:ii), jointvel(1,1:100:ii), 'Linewidth', 2, 'Color','r');
        plot(t_acc(1:100:ii), jointvel(2,1:100:ii), 'Linewidth', 2, 'Color','g');
        plot(t_acc(1:100:ii), jointvel(3,1:100:ii), 'Linewidth', 2, 'Color','b');
        % hold off;
        title('Velocity Profiles');
        xlabel('Time [s]'), ylabel('Torque [Nm]');
        xlim([t_acc(1), t_acc(end)]);
        ylims = [min(jointvel(1,:)), max(jointvel(1,:)),min(jointvel(2,:)), max(jointvel(2,:)),min(jointvel(3,:)), max(jointvel(3,:))];
        ylim([min(ylims), max(ylims)]);
        legend({'Joint 1', 'Joint 2', 'Joint 3'});
        hold off;
    
        axes(handles.axes5);
        hold on;
        plot(t_acc(1:200:ii), tau_acc(1,1:200:ii), 'Linewidth', 2, 'Color','r');
        plot(t_acc(1:200:ii), tau_acc(2,1:200:ii), 'Linewidth', 2, 'Color','g');
        plot(t_acc(1:200:ii), tau_acc(3,1:200:ii), 'Linewidth', 2, 'Color','b');
        % hold off;
        title('Torque Profiles');
        xlabel('Time [s]'), ylabel('Torque [Nm]');
        xlim([t_acc(1), t_acc(end)]);
        ylims = [min(tau_acc(1,:)), max(tau_acc(1,:)),min(tau_acc(2,:)), max(tau_acc(2,:)),min(tau_acc(3,:)), max(tau_acc(3,:))];
        ylim([min(ylims), max(ylims)]);
        legend({'Joint 1', 'Joint 2', 'Joint 3'});
        hold off;
    
        axes(handles.axes4);
        hold on;
        plot(t_acc(1:200:ii), jointacc(1,1:200:ii), 'Linewidth', 2, 'Color','r');
        plot(t_acc(1:200:ii), jointacc(2,1:200:ii), 'Linewidth', 2, 'Color','g');
        plot(t_acc(1:200:ii), jointacc(3,1:200:ii), 'Linewidth', 2, 'Color','b');
        % hold off;
        title('Acceleration Profiles');
        xlabel('Time [s]'), ylabel('Torque [Nm]');
        xlim([t_acc(1), t_acc(end)]);
        ylims = [min(jointacc(1,:)), max(jointacc(1,:)),min(jointacc(2,:)), max(jointacc(2,:)),min(jointacc(3,:)), max(jointacc(3,:))];
        ylim([min(ylims), max(ylims)]);
        legend({'Joint 1', 'Joint 2', 'Joint 3'});
        hold off;
    
        drawnow; % This command forces MATLAB to draw the plots immediately
        pause(0.01);
    end
    
    % robot.plot(jointacc(:,1:100:end)');
    prev_pos = [X Y Z]';
    % pos = prev_position(prev_pos,"save")
end

function editX_Callback(hObject, eventdata, handles)
% hObject    handle to editX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editX as text
%        str2double(get(hObject,'String')) returns contents of editX as a double
end

% --- Executes during object creation, after setting all properties.
function editX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function editY_Callback(hObject, eventdata, handles)
% hObject    handle to editY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editY as text
%        str2double(get(hObject,'String')) returns contents of editY as a double
end

% --- Executes during object creation, after setting all properties.
function editY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function editZ_Callback(hObject, eventdata, handles)
    % hObject    handle to editZ (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    
    % Hints: get(hObject,'String') returns contents of editZ as text
    %        str2double(get(hObject,'String')) returns contents of editZ as a double
end

% --- Executes during object creation, after setting all properties.
function editZ_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to editZ (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called
    
    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end


function edit4_Callback(hObject, eventdata, handles)
    % hObject    handle to edit4 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    
    % Hints: get(hObject,'String') returns contents of edit4 as text
    %        str2double(get(hObject,'String')) returns contents of edit4 as a double
end

% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to edit4 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called
    
    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end 


function edit5_Callback(hObject, eventdata, handles)
    % hObject    handle to edit5 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    
    % Hints: get(hObject,'String') returns contents of edit5 as text
    %        str2double(get(hObject,'String')) returns contents of edit5 as a double
end

% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to edit5 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called
    
    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end


function edit6_Callback(hObject, eventdata, handles)
    % hObject    handle to edit6 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    
    % Hints: get(hObject,'String') returns contents of edit6 as text
    %        str2double(get(hObject,'String')) returns contents of edit6 as a double
end    

% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to edit6 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called
    
    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end


function edit7_Callback(hObject, eventdata, handles)
    % hObject    handle to edit7 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    
    % Hints: get(hObject,'String') returns contents of edit7 as text
    %        str2double(get(hObject,'String')) returns contents of edit7 as a double
end

% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to edit7 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called
    
    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end


function edit8_Callback(hObject, eventdata, handles)
    % hObject    handle to edit8 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    
    % Hints: get(hObject,'String') returns contents of edit8 as text
    %        str2double(get(hObject,'String')) returns contents of edit8 as a double
end

% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to edit8 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called
    
    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end 


function edit9_Callback(hObject, eventdata, handles)
    % hObject    handle to edit9 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    
    % Hints: get(hObject,'String') returns contents of edit9 as text
    %        str2double(get(hObject,'String')) returns contents of edit9 as a double
end

% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to edit9 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called
    
    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
