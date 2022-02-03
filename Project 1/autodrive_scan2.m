%
% use keyboard to drive a wheeled mobile robot
% 
% to run: just type in 
% 
% >> autodrive_scan
% 
% use the arrow key in the figure window to drive the robot 
% (up=forward, down=backware, left=turn left, right=turn right)
% press q to quit
%
% this program calls 
%
% roomspec: set up the room and all collision objects in colobj structure
% roomshow: show all the collision objects
% robotspec: set up the robot collision body
% robotshow: show robot in the same plot
% robotfcn: function that gets called when a key is pressed in the figure
% output: generates all the sensor reading based on the current robot pose
% pose_est: estimating the pose of the robot
% colcheck: collision check between robot and all objects in room
%

% input u is assume available


 ez=[0;0;1];

% UWB (local GPS)
zW=colobj.obj{1}.Z;
lW=colobj.obj{1}.X;
zW=0;
pL(:,1)=[0;0;zW];pL(:,2)=[lW;0;zW];
% **** bearing sensors ****
pB(:,1)=colobj.obj{14}.Pose(1:3,4);
pB(:,2)=colobj.obj{10}.Pose(1:3,4);
N_scan=8;
N_range=size(pL,2);N_bearing=size(pB,2);N_odo=2;
%
ns=N_range+N_bearing+N_odo+N_scan; % total # of sensors
%wcov=[0.01;0.01];vcov=.15*ones(ns,1); % noise covariance ORIGINAL
wcov=[0.01;0.01];vcov=.001*ones(ns,1); % noise covariance  NOISE Turned OFF
% steering command and sampling period
v=.1;w=.1;ts=1;
% initial sensor reading
y(:,1)=output(q(:,1),pL,pB,N_scan,[0;0],vcov,robot,colobj);
% initial state estimate
qhat(:,1)=pose_est(y(:,1),pL,pB,N_scan,wcov,vcov);

% initialization for the EKF
%qhat_EKF(:,1)=qhat(:,1);P{1}=.1*eye(3,3);S{1}=.1*eye(3,3);

N_step=size(u,2);
%N_step=50;
% scan output range
% # of scan lines in lidar
ns=(N_range+N_odo+N_bearing+1:N_range+N_odo+N_bearing+N_scan);
% figure #
fignum=10;
%create occupancy map
map = binaryOccupancyMap(11,11,10);
% show one scan in robot frame
figure(fignum);
% no initial knowledge
qhat(:,1)=[0;0;0];
% first scan
[xloctemp,yloctemp]=scanloc(y(ns,1),zeros(3,1));
xloc_k(:,1)=xloctemp+qhat(1,1);
yloc_k(:,1)=yloctemp+qhat(2,1);
plot(xloctemp,yloctemp,'x');
hold on;
view(-90,90);axis([-1 11 -1 11 0 4])
axis([-1 11 -3 9 0 4])
% up to max = 3 segments
numseg=2;
% minimum segment length
minL=3;
% alignment threshold 
threshold=.2;
% extract lines
%[Pts,SegLength,Idx,unitvec,mbline,xline,yline]=...
%    linesegment([xloctemp yloctemp],threshold,numseg,minL,30);
%
% initial sensor reading
y(:,1)=output(q(:,1),pL,pB,N_scan,[0;0],vcov,robot,colobj);
% initial state estimate
qhat1(:,1)=pose_est(y(:,1),pL,pB,N_scan,wcov,vcov);
% initialization for the EKF
qhat_EKF(:,1)=qhat1(:,1);P{1}=.1*eye(3,3);S{1}=.1*eye(3,3);

N_step=size(u,2);
for k=1:N_step
    % generate output
    y(:,k+1)=output(q(:,k+1),pL,pB,N_scan,utrue(:,k),vcov,robot,colobj);
    % kinematics 
    [qhat(:,k+1),uu]=wmr(qhat(:,k),utrue(:,k),ts,zeros(size(wcov)));
    % one scan in robot frame
    [xloctemp,yloctemp]=scanloc(y(ns,k),zeros(3,1));
    % estimate robot state (wrt initial robot pose) using the robot
    % kinematics 
    % EKF estimate
    % [qhat_EKF(:,k+1),P{k+1},S{k+1}]=... 
    %    pose_est_kalman2(qhat_EKF(:,k),u(:,k),y(:,k),ts,...
    %    pL,pB,N_scan,wcov,vcov,P{k},S{k},robot,colobj);
    % switch to the estimate frame
    scanloc_k=(rot2(ez,qhat(3,k+1))*[xloctemp yloctemp]'+qhat(1:2,k+1))';
    %%uncomment for mapping ^^
    %scanloc_k=(rot2(ez,qhat_EKF(3,k+1))*[xloctemp yloctemp]'+qhat_EKF(1:2,k+1))'; %Uncomment for SLAM
    xloc_k(:,k+1)=scanloc_k(:,1);
    yloc_k(:,k+1)=scanloc_k(:,2);
    scanloc_k(:,:)=scanloc_k+[0.5*ones(8,1) 2.5*ones(8,1)]; %uncomment for
    %mapping
    %scanloc_k(:,:)=scanloc_k+[0.5*ones(8,1) 0.5*ones(8,1)]; %uncomment for SLAM
    setOccupancy(map,[scanloc_k(:,:)],1);
    figure(fignum+1)
    show(map)
    view(-90,90);axis([-1 11 -1 11 0 4]);
end


t=[0:N_step];
figure(2);h=plot(t,qhat(:,1:N_step+1),'-',...
    t,qhat_EKF(:,1:N_step+1),':');
set(h(1),'linewidth',2);set(h(2),'linewidth',2);set(h(3),'linewidth',2);
set(h(4),'linewidth',1);set(h(5),'linewidth',1);set(h(6),'linewidth',1);
legend('x','y','\theta',...
    'x_{EKF}','y_{EKF}','\theta_{EKF}');


cloud=[reshape(xloc_k,(N_step+1)*N_scan,1),reshape(yloc_k,(N_step+1)*N_scan,1)];

%figure(1)
%plot(cloud(:,1),cloud(:,2),'o')
%view(-90,90);axis([-1 11 -3 9 0 4]);grid;
%hold off;
%
%cloud=[cloud zeros((N_step+1)*N_scan,1)];
%
%ptCloud=pointCloud(cloud);
%[labels,numClusters] = pcsegdist(ptCloud,0.5);
%figure(2)
%pcshow(ptCloud.Location,labels)
%colormap(hsv(numClusters))
%title('Point Cloud Clusters')
