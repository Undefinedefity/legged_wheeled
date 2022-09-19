state = load("turning.txt");
gif_name = "turning.gif";
t =  0: 0.002 : (length(state)-1) * 0.002;
% plot(t,state(:,1),'-r','LineWidth',1);
% hold on;
% plot(t,state(:,2),'b','LineWidth',1);
% hold on;
% plot(t,state(:,3),'-r','LineWidth',1);
% hold on;
%% 
left_front_abad = 1;
left_front_hip = 2;
left_front_knee = 3;
left_front_wheel = 4;
left_back_abad = 5;
left_back_hip = 6;
left_back_knee = 7;
left_back_wheel = 8;

left_front_abad_torque = state(:,left_front_abad);
left_front_hip_torque = state(:,left_front_hip);
left_front_knee_torque = state(:,left_front_knee);
left_front_wheel_torque = state(:,left_front_wheel);
left_back_abad_torque = state(:,left_back_abad);
left_back_hip_torque = state(:,left_back_hip);
left_back_knee_torque = state(:,left_back_knee);
left_back_wheel_torque = state(:,left_back_wheel);
%%
left_front_abad_plot = left_front_abad_torque(:);
left_front_hip_plot = left_front_hip_torque(:);
left_front_knee_plot = left_front_knee_torque(:);
left_front_wheel_plot = left_front_wheel_torque(:);
left_back_abad_plot = left_back_abad_torque(:);
left_back_hip_plot = left_back_hip_torque(:);
left_back_knee_plot = left_back_knee_torque(:);
left_back_wheel_plot = left_back_wheel_torque(:);

% determine whether slow the video
slow_ratio = 1;
% time between each frame(s)
sample_time = 0.04; % 0.04s --> 50 fps
%% plot
%--- plot the first frame ---%

% plot abad
plot(t(1),left_front_abad_plot(1),'-r','LineWidth',2);
hold on;
plot(t(1),left_back_abad_plot(1),'-g','LineWidth',2);
hold on;

% % plot data1
% plot(t(1),left_front_hip_plot(1),'-r','LineWidth',2);
% hold on;
% 
% % % plot data2
% plot(t(1),left_front_knee_plot(1),'g','LineWidth',2);
% hold on;
% 
% % % plot data3
% plot(t(1),left_front_wheel_plot(1),'-b','LineWidth',2);
% hold on;

% % % plot data4
% plot(t(1),left_back_hip_plot(1),'-r','LineWidth',2);
% hold on;
% 
% % % plot data5
% plot(t(1),left_back_knee_plot(1),'b','LineWidth',2);
% hold on;
% 
% % % plot data6
% plot(t(1),left_back_wheel_plot(1),'-k','LineWidth',2);
% hold on;
%legend('ee_x', 'ee_y', 'ee_z');

grid on;
xlabel('Time(s)');
ylabel('joint output torque(Nm)');
xlim([0 t(end)]);
ylim([-20 20]);
% set(gca,'XTick',[-0.6:0.4:0.8]);
% set(gca,'YTick',[0:4:t_end]);
set(gca,'FontSize',20,'fontname','times');

%-- set the figure position, length and width --%
set(gcf,'color','white','Position',[400,320,1500,900]); %% position [x, y, length, width]
mark= 1;
%-- record as gif --%
for k = 1:(t(end)/sample_time)

    plot(t(1:(sample_time/0.002)*k),left_front_abad_plot(1:(sample_time/0.002)*k),'-r','LineWidth',2);
    hold on;

    plot(t(1:(sample_time/0.002)*k),left_back_abad_plot(1:(sample_time/0.002)*k),'-g','LineWidth',2);
    hold on;

%     plot(t(1:(sample_time/0.002)*k),left_front_hip_plot(1:(sample_time/0.002)*k),'-r','LineWidth',2);
%     hold on;
% 
%     plot(t(1:(sample_time/0.002)*k),left_front_knee_plot(1:(sample_time/0.002)*k),'-g','LineWidth',2);
%     hold on;
% 
%     plot(t(1:(sample_time/0.002)*k),left_front_wheel_plot(1:(sample_time/0.002)*k),'-b','LineWidth',2);
%     hold on;

%     plot(t(1:(sample_time/0.002)*k),left_back_wheel_plot(1:(sample_time/0.002)*k),'-.m','LineWidth',2);
%     hold on;
% 
%     plot(t(1:(sample_time/0.002)*k),left_back_wheel_plot(1:(sample_time/0.002)*k),'-.y','LineWidth',2);
%     hold on;
% 
%     plot(t(1:(sample_time/0.002)*k),left_back_wheel_plot(1:(sample_time/0.002)*k),'-.k','LineWidth',2);
%     hold on;

    legend('leftFrontAbAd','leftBackAbAd');
    %-- record the frame and save --%
    F = getframe(gcf);
    im = frame2im(F);
    [I,map] = rgb2ind(im,256);
    if mark == 1
        imwrite(I, map, gif_name, 'GIF', 'Loopcount', inf, 'DelayTime', sample_time);
        mark = mark + 1;
    else
        imwrite(I, map, gif_name, 'WriteMode', 'append', 'DelayTime', sample_time);
    end
end