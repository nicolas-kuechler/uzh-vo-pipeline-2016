function plotTrajectory(location, orientation, pt_cloud, plot_last)
 
if size(orientation,2)-1 < plot_last
    plot_last = size(orientation,2)-1;
end
 
i = size(orientation,2);
    figure(23456788);
    subplot(1,2,1)
    cameraSize = 0.1;
    plotCamera('Location',location(:,end),'Orientation',reshape(orientation(:,end),[3 3]),'Size',...
        cameraSize,'Color','r','Label',num2str(i),'Opacity',.5);
    hold on
    pcshow(pt_cloud','VerticalAxis','y','VerticalAxisDir',...
        'down','MarkerSize',45);    
    plot3(location(1,end-plot_last:end),location(2,end-plot_last:end), location(3,end-plot_last:end))
    grid on
    view([0,-1,0])
    lims = [location(:,end)' - 5; location(:,end)' + 5];
    axis(lims(:));
    hold off
    
    subplot(1,2,2);
    plot(location(1,:), location(3,:));
    view([0,-1,0]);
    lims = [-10, 10,0,50];
    axis(lims);
end