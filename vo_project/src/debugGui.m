function ui
sld= uicontrol('Style', 'slider',...
    'Min',1,'Max',50,'Value',41,...
    'Position', [400 20 120 20],...
    'Callback', @surfzlim); 
    
    function surfzlim(source,event)
        val = 51 - source.Value;
        % For R2014a and earlier:
        % val = 51 - get(source,'Value');

        zlim(ax,[-val val]);
    end
end