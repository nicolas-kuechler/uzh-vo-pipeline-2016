function [handle] = debugGui(window, window_params, l_handle)
    
    %Function Workspace Globals
    continue_flag = false;
    frame_index = window_params.window_index; %set index to newest frame
    window_pos = window_params.window_size;
    

    if isempty(fieldnames(l_handle))
        gui = figure('Name','Simulation Control', 'NumberTitle','off', ...
           'Visible','off','Position',[360,500,900,600]);

        prev_btn = uicontrol('Position',[20 20 200 40],'String','Previous Frame',...
        'Callback',@prev);

        next_btn = uicontrol('Position',[220 20 200 40],'String','Next Frame',...
        'Callback',@next);
    
        continue_btn = uicontrol('Position',[460 20 200 40],'String','Calculate Step',...
              'Callback',@continue_cbck);

        handle = struct('figure', gui, ...
                        'continue_btn', continue_btn, ...
                        'next_btn', next_btn, ...
                        'prev_btn', prev_btn);
                    
    else %just update
        figure(l_handle.figure);
        handle = l_handle;
        %update callback because of global values in callback (otherwise
        %they would refer to the old instance
        set(findobj(handle.continue_btn), 'Callback' , @continue_cbck);
        set(findobj(handle.next_btn), 'Callback' , @next);
        set(findobj(handle.prev_btn), 'Callback' , @prev);
    end
    
    while ~continue_flag;
        frame = window{1,frame_index};
        %TODO: replace old plot instead of overdrawing
        hold on;
        imshow(frame.image);
        plot(frame.debug_data.curr_matched_kp(1,:), frame.debug_data.curr_matched_kp(2,:), 'rx', 'Linewidth', 2);
        hold off;
        
        uiwait(gcf);      %wait for next button
    end
    

%% Callbacks etc
    function prev(source,eventdata)
         if window_pos > 1 
            window_pos = window_pos-1;
            if frame_index == 1
                frame_index = window_params.window_max_size;
            else
               frame_index = frame_index-1;
            end

         end
         frame_index
         uiresume(gcbf);
    end

    function next(source,eventdata)
        if window_pos < window_params.window_max_size && window_pos < window_params.window_size
            window_pos = window_pos+1;
            frame_index = mod(frame_index, ...
                window_params.window_max_size)+1;
        end
        frame_index
        uiresume(gcbf);
    end

    function continue_cbck(source,eventdata)
        continue_flag = true;
        uiresume(gcbf);
    end

end