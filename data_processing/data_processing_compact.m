%% ---- DATA PROCESSING----%%

%This scripts process the basg with multiple recordings, filtering the
%messages through the boolean task variable, representing whenever the task
%is "on" or "off". 

clear all
clc
%%
%Select the bags folder
source_path = uigetdir([]);
manager_bags = dir([source_path, '/manager/*.bag']);
optitrack_bags = dir([source_path, '/optitrack/*.bag']);
optitrack_raw_bags = dir([source_path, '/optitrack_raw/*.bag']);
RS_bags = dir([source_path, '/RS/*.bag']);
gripper_bags = dir([source_path, '/gripper/*.bag']);
femto_bags = dir([source_path, '/femto/*.bag']);

manager_folder = manager_bags.folder;
manager_name = manager_bags.name;
manager_full_name = [manager_folder, '/', manager_name];

optitrack_folder = optitrack_bags.folder;
optitrack_name = optitrack_bags.name;
optitrack_full_name = [optitrack_folder, '/', optitrack_name];

optitrack_raw_folder = optitrack_raw_bags.folder;
optitrack_raw_name = optitrack_raw_bags.name;
optitrack_raw_full_name = [optitrack_raw_folder, '/', optitrack_raw_name];

gripper_folder = gripper_bags.folder;
gripper_name = gripper_bags.name;
gripper_full_name = [gripper_folder, '/', gripper_name];

femto_folder = femto_bags.folder;
femto_name = femto_bags.name;
femto_full_name = [femto_folder, '/', femto_name];

RS_folder = RS_bags.folder;
RS_name = RS_bags.name;
RS_full_name = [RS_folder, '/', RS_name];

manager = rosbag(manager_full_name);
gripper = rosbag(gripper_full_name);

femto = rosbag(femto_full_name);
RS = rosbag(RS_full_name);
OPT = rosbag(optitrack_full_name);
OPT_raw = rosbag(optitrack_raw_full_name);

%Read glasses information file
glasses_dir = dir([source_path,'/gaze_data', '/*.csv']);
glasses_path = glasses_dir.folder + "/" + glasses_dir.name;

%Extract task boolean variable and realtive timestamps
task_msg = select(manager, 'Time', [manager.StartTime manager.EndTime], 'Topic', '/task');
task_bool = readMessages(task_msg);
time_task = task_msg.MessageList.Time;

%Filter task_on instants and indeces
task_value = false(length(task_bool),1);

for ii = 1: length(task_bool)
    task_value(ii,1) = task_bool{ii}.Data;
end

time_on = time_task(task_value,1);

% to fix cut demo at the beginning. Uncomment if not needed
idx_tot = (task_value==1);
idx_modified = [];

for ii = 1: length(idx_tot)
    if (idx_tot(ii) == 1) && (idx_tot(ii-1)== 0)
        idx_modified(ii-10: ii) = 1;
    else
        idx_modified = [idx_modified; idx_tot(ii)];
    end
end

new_idx_on = logical(idx_modified);
time_on = time_task(new_idx_on, 1);


%Extract sensor messages

%Extract hand pose
hand_msg = select(OPT, 'Topic', '/hand_new_pose');
%hand_raw_msg = select(OPT_raw, 'Topic', '/optitrack/hand');
time_hand = hand_msg.MessageList.Time;

%Extract head pose
head_msg = select(OPT, 'Topic', '/glasses_new_pose');
%head_rarw_msg = select(OPT_raw, 'Topic', '/optitrack/tobii_glasses');
time_head = head_msg.MessageList.Time;

%Franka raw pose 
franka_msg = select(OPT_raw, 'Topic', '/optitrack/franka');
franka_raw = readMessages(franka_msg,1);  %Get initial franka pose msg

%Extract gripper messages from gripper bag
gripper_msg = select(gripper, 'Topic', '/gripper_state');
time_gripper = gripper_msg.MessageList.Time;

%Extract objectA and objectB messages from manager bag
objA_msg = select(manager, 'Time', [manager.StartTime manager.EndTime], 'Topic', '/objA');
time_objA = objA_msg.MessageList.Time;
objB_msg = select(manager, 'Time', [manager.StartTime manager.EndTime], 'Topic', '/objB');
time_objB = objB_msg.MessageList.Time;

%Extract RGB and depth messages from RealSense bag
RS_color_msg = select(RS, 'Topic', '/ImageCol_Ext');
RS_depth_msg = select(RS, 'Topic', '/RGBDImage_Ext');
time_RS_color = RS_color_msg.MessageList.Time;
time_RS_depth = RS_depth_msg.MessageList.Time;

%Extract RGB and depth messages from FemtoBolt bag
femto_color_msg = select(femto, 'Topic', '/ImageColor');
femto_depth_msg = select(femto, 'Topic', '/ImageDepth');
% time_femto_color = femto_color_msg.MessageList.Time;
% time_femto_depth = femto_depth_msg.MessageList.Time;

tic
image_femto = readMessages(femto_color_msg);
depth_femto = readMessages(femto_depth_msg);
toc
disp('Femto Messages read')
% image_femto = readMessages(femto_color_msg, 'DataFormat', 'struct');
% depth_femto = readMessages(femto_depth_msg, 'DataFormat', 'struct');
% 
time_femto_color = zeros(length(image_femto),1);
for ii = 1: length(image_femto)
    time_femto_color(ii)  = image_femto{ii,1}.Header.Stamp.Sec + (image_femto{ii,1}.Header.Stamp.Nsec)*10^(-9);
end

time_femto_depth = zeros(length(depth_femto),1);
for ii = 1: length(depth_femto)
    time_femto_depth(ii)  = depth_femto{ii,1}.Header.Stamp.Sec + (depth_femto{ii,1}.Header.Stamp.Nsec)*10^(-9);
end


%Filter out message time according to task variable: Initialize empty
%arrays for each message

idx_hand_on = zeros(1, length(time_on));

idx_head_on = zeros(1, length(time_on));
idx_gripper_on = zeros(1, length(time_on));

% idx_femto_color_on = zeros(1, length(time_on));
% idx_femto_depth_on = zeros(1, length(time_on));


idx_RS_color_on = zeros(1, length(time_on));
idx_RS_depth_on = zeros(1, length(time_on));

idx_objA_on = zeros(1,length(time_on));
idx_objB_on = zeros(1,length(time_on));


%Synchronize time instant wrt rs stream
for jj = 1: length(time_on)

    %Hand pose
    [~, idx_min_hand] = min(abs(time_on(jj) - time_hand));
    idx_hand_on (jj) = idx_min_hand;
    % 
     %Head pose
    [~, idx_min_head] = min(abs(time_on(jj) - time_head)); 
    idx_head_on (jj) = idx_min_head;

    %Gripper
    [~, idx_min_gripper] = min(abs(time_on(jj) - time_gripper));
    idx_gripper_on (jj) = idx_min_gripper;

    %Femto (color and depth)
    % [~, idx_min_col_femto] = min(abs(time_on(jj) - time_femto_color));
    % idx_femto_color_on (jj) = idx_min_col_femto;
    % 
    % [~, idx_min_depth_femto] = min(abs(time_on(jj) - time_femto_depth));
    % idx_femto_depth_on (jj) = idx_min_depth_femto;

    %RS color and depth
    [~, idx_min_RS_col] = min(abs(time_on(jj) - time_RS_color));
    idx_RS_color_on (jj) = idx_min_RS_col;

    [~, idx_min_RS_depth] = min(abs(time_on(jj) - time_RS_depth));
    idx_RS_depth_on (jj) = idx_min_RS_depth;
    % 
     [~, idx_min_objA] = min(abs(time_on(jj) - time_objA));
    idx_objA_on (jj) = idx_min_objA;

     [~, idx_min_objB] = min(abs(time_on(jj) - time_objB));
    idx_objB_on (jj) = idx_min_objB;

end


%Avoid indeces repetition
idx_hand = unique(idx_hand_on, 'stable');
idx_head = unique(idx_head_on, 'stable');
idx_gripper = unique(idx_gripper_on, 'stable');
idx_RS_color = unique(idx_RS_color_on, 'stable');
idx_RS_depth = unique(idx_RS_depth_on, 'stable');
% idx_femto_c = unique(idx_femto_color_on, 'stable');
% idx_femto_d = unique(idx_femto_depth_on, 'stable');
idx_objA = unique(idx_objA_on, 'stable');
idx_objB = unique(idx_objB_on, 'stable');

%Get only synchrinizedc timestamps with task active
time_hand_on = time_hand(idx_hand);
time_head_on = time_head(idx_head);
time_gripper_on = time_gripper(idx_gripper);
time_RS_color_on = time_RS_color(idx_RS_color);
time_RS_depth_on = time_RS_depth(idx_RS_depth);
% time_femto_c_on = time_femto_color(idx_femto_c);
% time_femto_d_on = time_femto_depth(idx_femto_d);
time_objA_on = time_objA(idx_objA);
time_objB_on = time_objB(idx_objB);

time_femto_c_on = time_femto_color;
time_femto_d_on = time_femto_depth;

initial_ = false(length(time_RS_color_on),1);
final_ = false(length(time_RS_color_on),1);

for ii = 1: length(time_RS_color_on) -1
    if ii == 1
        initial_(ii) = 1;
        delta_prev = 0;
        delta_next = 0;
    else
        delta_next = time_RS_color_on(ii + 1) - time_RS_color_on(ii);
        delta_prev = time_RS_color_on(ii) - time_RS_color_on( ii - 1 );
    end
    
    if delta_next > 1
        final_(ii) = 1;
    elseif delta_prev > 1
        initial_(ii) = 1;
    end
    if ii == length(time_RS_color_on)-1
        final_(ii+1) = 1;
    end
end

initial_ = double(initial_);
final_ = double(final_);
Ndemo = sum(double(initial_));
Ndemo = sum(double(final_));

%Read filtered sensor messages:only where task is active
hand_on = readMessages(hand_msg, idx_hand);
head_on = readMessages(head_msg, idx_head);  %CHECk
gripper_on = readMessages(gripper_msg, idx_gripper);
image_RS_on = readMessages(RS_color_msg, idx_RS_color);
depth_RS_on = readMessages(RS_depth_msg, idx_RS_depth);
% image_femto_on = readMessages(femto_color_msg, idx_femto_color);
% depth_femto_on = readMessages(femto_depth_msg, idx_femto_depth);
image_femto_on = image_femto;
depth_femto_on = depth_femto;
objA_on = readMessages(objA_msg, idx_objA);
objB_on = readMessages(objB_msg, idx_objB);

%Convert gripper messages to double
gripper_bool = zeros(length(gripper_on),1);
for ii = 1: length(gripper_on)
    gripper_bool(ii) = gripper_on{ii}.Data;
end

% offset mistake correction:
offset_x = 0.0;
offset_y = 0.14;
offset_z = 0.01;
%offset_y = 0.0;
dist_x = -0.02;  %offset marker-thumb
dist_y = -0.04;
dist_z = -0.02;

%Initialiaze empty arrays for hand, head pose
hand_xyz = zeros(length(hand_on), 3);
hand_xyz_corr = zeros(length(hand_on), 3);
hand_orient = zeros(length(hand_on), 4);
hand_orient_angles = zeros(length(hand_on), 3);

head_xyz = zeros(length(head_on), 3);
head_raw_xyz = zeros(length(head_on), 3);
%head_orient = zeros(length(head_on), 4);
%head_orient_angles = zeros(length(head_on), 3);


%%%%%% -------Uncomment ONLY IF reading optitrack raw messages------- 
%Calculate geometric transformation from optitrack frame to robot frame. 
%[transform] = optitrack2robot(franka_raw);

%hand_raw_pose = zeros(length(hand_on),3);
%hand_raw_orient = zeros(length(hand_on),4);
%hand_raw_quat = zeros(4, length(hand_on));
%final_pose = zeros(3, length(hand_on));
%hand_franka_orient = cell(length(hand_on),1);

%for ii = 1: length(hand_on)

   % hand_raw_pose(ii,1) = hand_on{ii}.Pose.Position.X;
    %hand_raw_pose(ii,2) = hand_on{ii}.Pose.Position.Y;
    %hand_raw_pose(ii,3) = hand_on{ii}.Pose.Position.Z;
    %hand_raw_orient(ii,1) = hand_on{ii}.Pose.Orientation.W;
    %hand_raw_orient(ii,2) = hand_on{ii}.Pose.Orientation.X;
    %hand_raw_orient(ii,3) = hand_on{ii}.Pose.Orientation.Y;
    %hand_raw_orient(ii,4) = hand_on{ii}.Pose.Orientation.Z;
    %hand_raw_quat(:, ii) = hand_raw_orient(ii, :)';
    %hand_orient = quat2rotm(hand_raw_orient(ii, :));
    %hand_franka_orient{ii,1} = transform*[hand_orient;[0,0,0]];
    %hand_raw_pose_h = [hand_raw_pose(ii,:)'; 1]; % Convert to homogeneous coordinates
    %hand_franka_pose_h = transform * hand_raw_pose_h;
    %hand_franka_pose = hand_franka_pose_h(1:3) + [offset_x; offset_y; offset_z]; % Extract new position and correct the offset
    %hand_xyz(ii, :) = hand_franka_pose'; 
    %[quat_] = rotm2quat(hand_franka_orient{ii,1}(1:3,:));
    %[roll, pitch, yaw] = quat2angle(quat_, 'XYZ');
    %hand_orient_angles(ii, :) = [roll, pitch, yaw];    %transformed orientation
    %hand_xyz_corr(ii, 1) = hand_xyz(ii, 1) + dist_x*(cos(roll)*cos(yaw));
    %hand_xyz_corr(ii, 2) =  hand_xyz(ii, 2) + dist_y*(cos(roll)*sin(yaw));
    %hand_xyz_corr(ii, 3) = hand_xyz(ii, 3) + dist_z*(sin(roll));
%end
%for ii = 1: length(head_on)
    %head_raw_xyz(ii, 1) = head_on{ii}.Pose.Position.X;
    %head_raw_xyz(ii, 2) = head_on{ii}.Pose.Position.Y;
    %head_raw_xyz(ii, 3) = head_on{ii}.Pose.Position.Z;
    %head_pose_h = [head_raw_xyz(ii, :)'; 1]; %homogeneous head coordinates
    %head_xyz_transformed = transform * head_pose_h; 
    %head_xyz(ii, :) = head_xyz_transformed(1:3)+ [offset_x; offset_y; offset_z];   %take only relevant coordinates
%end
%%%%%-------- End of the raw_messages VERSION--------%%%%%%

%%%%% Comment these lines if using raw optitrack messages %%%%%%
% Define hand pose as nx3 matrix
for ii = 1: length(hand_on)
    hand_xyz(ii, 1) = hand_on{ii}.Pose.Position.X + offset_x;
    hand_xyz(ii, 2) = hand_on{ii}.Pose.Position.Y + offset_y;
    hand_xyz(ii, 3) = hand_on{ii}.Pose.Position.Z;
    hand_orient(ii, 1) = hand_on{ii}.Pose.Orientation.W;
    hand_orient(ii, 2) = hand_on{ii}.Pose.Orientation.X;
    hand_orient(ii, 3) = hand_on{ii}.Pose.Orientation.Y;
    hand_orient(ii, 4) = hand_on{ii}.Pose.Orientation.Z;
    [roll, pitch, yaw] = quat2angle(hand_orient(ii,:), 'XYZ');
    hand_orient_angles(ii, :) = [roll, pitch, yaw];
    hand_xyz_corr(ii, 1) = hand_xyz(ii, 1) + dist*(cos(roll)*cos(yaw));
    hand_xyz_corr(ii, 2) =  hand_xyz(ii, 2) + dist*(cos(roll)*sin(yaw));
    hand_xyz_corr(ii, 3) = hand_xyz(ii, 3) + dist*(sin(roll));
 end

%Extract head poses as nx3 matrix OLD VERSION
 for ii = 1: length(head_on)
    head_xyz(ii, 1) = head_on{ii}.Pose.Position.X;
    head_xyz(ii, 2) = head_on{ii}.Pose.Position.Y;
    head_xyz(ii, 3) = head_on{ii}.Pose.Position.Z;
 end
%%%%-------%%%%%

  %Synchronize data wrt RS stream
  [time_gripper_sync_RS, time_hand_sync_RS, time_rgbd_sync_RS, time_head_sync_RS, time_objA_sync, time_objB_sync, time_femto_color_sync, time_femto_deth_sync, femto_color_sync, femto_depth_sync,  gripper_bool_sync_RS, hand_xyz_sync_RS2, hand_xyz_sync_corr, hand_angles_RS, images_depth_sync_RS, head_xyz_sync_RS, objectA_sync, objectB_sync] = ...
        synchronizeData(time_RS_color_on, image_RS_on,  depth_RS_on, time_gripper_on, time_hand_on, time_head_on, time_RS_depth_on, time_objA_on, time_objB_on, time_femto_c_on, time_femto_d_on, image_femto_on, depth_femto_on, gripper_bool, hand_xyz, hand_xyz_corr, hand_orient_angles, head_xyz, objA_on, objB_on);

disp('Synchronization Done')

hand_xyz_sync_RS = hand_xyz_sync_corr;
%Round the pose at 3rd decimal digit : i.e. 1 mm accuracy
delta_hand_pose = zeros(length(hand_xyz_sync_RS), size(hand_xyz_sync_RS, 2));
delta_hand_rot = zeros(length(hand_angles_RS), size(hand_angles_RS,2));
hand_xyz_sync_RS = round(hand_xyz_sync_RS, 3);  %mm accuracy
hand_angles_RS = round(hand_angles_RS, 2);  % 1/100 rad accuracy--> 0.5 deg
head_xyz_sync_RS = round(head_xyz_sync_RS, 3);
%%
%Compute delta pose and delta rotation eof
for jj = 1: length(delta_hand_pose)
    if jj < (length(delta_hand_pose) - 1)
        delta_hand_pose(jj, :) = hand_xyz_sync_RS(jj+1, :) - hand_xyz_sync_RS(jj, :);
        delta_hand_rot(jj, :) = hand_angles_RS(jj+1, :) - hand_angles_RS(jj, :);
    else
        delta_hand_pose(jj, :) = zeros(1, 3);
        delta_hand_rot(jj, :) = zeros(1,3);
    end
end

%Initialize dataset
dataset_old_RS = cell(1,18);
%Define task type
task_type = "pick and place";
%Specify folder path to save images and dataset
folder_path = './output';

% Glasses processing section
opts = detectImportOptions(glasses_path, 'TextType', 'string');
glasses_data = readtable(glasses_path, opts);

%Read initial and final instant columns
% initial = find(table2array(glasses_data(:, 6)));
% final = find(table2array(glasses_data(:, 7)));

%Retrieve number of demos
% Ndemo = length(initial);

%Read images names and gaze coordinates
frames_name = table2array(glasses_data(:,1));
gaze_coord = table2array(glasses_data(:, 2:3));

%Get objectA and objectB
objA_list = table2array(glasses_data(:, 8)); 
objB_list = table2array(glasses_data(:, 9));

%Get image resolution 
img_height = table2array(glasses_data(1,10));
img_width = table2array(glasses_data(1,11));
img_channels = table2array(glasses_data(1,12));

%frame_reshaped = reshape(frame_1, [img_height, img_width, img_channels]);  %bgr 24

%Initialize empy glasses dataset
dataset_old_glasses = cell(1,6);


%Check correct acquisition
%if length(initial) == length(final)
%   print('Size matches, no incomplete demonstrations')
       %N_demo = length(initial)
%else
%   print('Unmatched size! Incomplete demonstrations)
%end


%Initialize time arrays with task active
time_frame_on = [];
time_gaze_on = [];

time_frame = table2array(glasses_data(:, 4));
time_gaze = table2array(glasses_data(:, 5));
time_frame_on = time_frame;

%Set rs stream as reference stream
time_ref = time_RS_color_on;
%Synchronize glasses stream with rs stream
[time_sync, idx_sync] = synchronize_glasses(time_ref, time_frame_on);

%NB time_sync should have same length of time_RS_on etc

%% Adjust cropping index 
min_width = 1;   %default = 1;
max_width = 800;  %default = 848;
min_height = 150;   %default = 1;
max_height = 480;  %default = 480;

%Read and save data camera1
img_color1 = cell(length(image_RS_on),1);
img_depth1 = cell(length(image_RS_on),1);

%Cropping index camera2:
min_width_f = 1;   %default = 1;
max_width_f = 1270;  %default = 1270;
min_height_f = 1;   %default = 1;
max_height_f = 720;  %default = 720;

min_width_depth = 1;   %default = 1;
max_width_depth = 1024;  %default = 1270;
min_height_depth = 1;   %default = 1;
max_height_depth = 1024;  %default = 720;

min_width2 = 150;   %default = 1;
max_width2 = 1200;  %default = 1270;
min_height2 = 200;   %default = 1;
max_height2 = 690;  %default = 720;

%Read and save data camera2
img_color2 = cell(length(image_RS_on),1);
img_depth2 = cell(length(image_RS_on),1);

counter = 0;
n_demo = 0;

%Path for saving RS images 
folderName = source_path + "/images_1";

%Path for reading gaze images
% gaze_dir = dir([source_path, '/gaze_images/']);
gaze_dir = source_path +  "/gaze_images/";

folderName_glasses = source_path+ "/glasses_images2";

counter_step =0;
tic

for ll = 1: length(image_RS_on)
    %Step counter
    counter = counter+1;
    counter_step = counter_step+1;

    %Progressive demo counter
    n_demo= sum(initial_(1: ll));

    if initial_(ll) == 1
        counter_step = 1;
    end

    %Get RS Color Image and Depth map [camera1]
    img_c1 = readImage(image_RS_on{ll});
    img_d1 = readImage(images_depth_sync_RS{ll});
    %Undistort images
    [img_color1{ll}, img_depth1{ll}] = setIntrinsics(img_c1, img_d1, "realsense");
    
    %Get femto Color Image and Depth map [camera2]
    img_c2 = readImage(femto_color_sync{ll});
    img_d2_original = readImage(femto_depth_sync{ll});

    img_d2 = resize_with_aspect_ratio(img_d2_original, size(img_c2, 2), size(img_c2,1));
    %Undistort images
    [img_color2{ll}, img_depth2{ll}] = setIntrinsics(img_c2, img_d2, "femto");

    % Get gaze coordinates
    gaze_xy = gaze_coord( idx_sync(ll) , :); %Check if inside the height, width limit
    % Read glasses images
    % image_path = gaze_dir + frames_name(idx_sync(ll));
    % frame = imread(image_path);

    %Resize if needed
    %frame = imresize(frame, [ img_height, img_width ]);
    
    % %Get object A and object B info (redundant)
    object_A =  objA_list( idx_sync(ll) );
    object_B = objB_list( idx_sync(ll) );
    
    %Check if gaze is valid
    if gaze_xy(1) > img_width || gaze_xy(2) > img_height
        disp('Invalid gaze data')
    end

    %Create the folder at first iteration for RS images
    if isfolder(folderName) ==  0
        mkdir(folderName);
    end
    
    %Create the folder for glasses images
     if isfolder(folderName_glasses) ==  0
        mkdir(folderName_glasses);
    end

    %Navigate into the RS folder
    cd(folderName)
    
    %Saving RS Images and depth maps:
    idx = counter_step - 1;
    color_name = "D" + n_demo + "_" + "RS" +  "_RGB_" + idx +".jpeg";
    imwrite(img_color1{ll}(min_height : max_height, min_width : max_width , :), color_name);

    depth_name = "D"+ n_demo + "_" +  "RS" +  "_depth_" + idx + ".csv";
    writematrix(img_depth1{ll}(min_height : max_height, min_width : max_width , :), depth_name);
    % writematrix(img_d1(min_height : max_height, min_width : max_width , :), depth_name);

    %Saving femto Images and depth maps:
    color_name2 = "D" + n_demo + "_" + "femto" +  "_RGB_" + idx +".jpeg";
    imwrite(img_color2{ll}(min_height2 : max_height2, min_width2 : max_width2 , :), color_name2);
    depth_name2 = "D"+ n_demo + "_" +  "femto" +  "_depth_" + idx + ".csv";
    writematrix(img_depth2{ll}(min_height_f : max_height_f, min_width_f : max_width_f , :), depth_name2);

    %Navigate back to parent folder
    cd ..;
    
    %Navigate into the glasses folder
    cd(folderName_glasses)

    %Save the glasses image 
    name_glasses = frames_name(idx_sync(ll), : );

    %Full path
    %new_gaze_dir = dir([folderName_glasses]);
    
    moving_path =  gaze_dir + name_glasses;

    %Move the image to the current folder to filter out only relevant
    %images (task on)

    % Filter out glasses frames by copything them in a different folder
    copyfile(moving_path);

    %Navigate back to parent folder
    cd ..;

    %Store the demonstration frame into the dataset
    [dataset_new_RS] = createDataset(dataset_old_RS, counter, hand_xyz_sync_RS(ll, :), head_xyz_sync_RS(ll, :), hand_angles_RS(ll, :),  gripper_bool_sync_RS(ll), object_A, object_B, color_name, depth_name, color_name2, depth_name2, task_type, delta_hand_pose(ll, :), delta_hand_rot(ll, :), n_demo, initial_(ll), final_(ll));
    [dataset_new_glasses] = createGlassesDataset(dataset_old_glasses, counter, name_glasses,gaze_xy, object_A, object_B, n_demo);

    dataset_old_RS = dataset_new_RS;
    dataset_old_glasses = dataset_new_glasses;

end
toc


DS = cell2table(dataset_new_RS);
%
DS.Properties.VariableNames = {'Step', 'eof [m]', 'eof orient [rad]', 'obj_pose_cm and depth','obj_pose px', 'gripper_status', 'obj1', 'obj2', 'task_type', 'camera 1 RGB and depth', 'camera 2 RGB and depth' 'delta_pose [m]',' delta_rot [rad]',  'demo number', 'head_pose [m]', 'pointer', 'Initial', 'Final'};

writetable(DS, 'Dataset_1.xlsx');

DSg = cell2table(dataset_new_glasses);
%
DSg.Properties.VariableNames = {'Image name', 'gaze x [px]', 'gaze y [px]', 'object A', 'object B', 'Demo number'};

writetable(DSg, 'Dataset_glasses_1.xlsx');

%% 
%%%%%%----- PLOTTING ---------%%%%

%Cropping index:
% min_width = 250;   %default = 1;
% max_width = 800;  %default = 848;
% min_height = 160;   %default = 1;
% max_height = 450;  %default = 480;

for ii = 19% : 18 %length(time_RS_color_on)

    figure(1)
    set(gcf,'units','points','position',[50,50,1600,900])

    %Hand pose perspective view
    subplot(2,3,1)
    %plot3(hand_xyz_sync(ii,1), hand_xyz_sync(ii,2), hand_xyz_sync(ii,3),  '+', 'MarkerSize', 10)
    grid on
    hold on
    %plot3(hand_xyz_sync_RS(ii, 1), hand_xyz_sync_RS(ii, 2), hand_xyz_sync_RS(ii, 3),  '-*', 'MarkerSize', 10)
    plot3(hand_xyz_sync_corr(ii, 1), hand_xyz_sync_corr(ii, 2), hand_xyz_sync_corr(ii, 3),  '-*', 'MarkerSize', 10)
    xlim([-0.30, 0.60]);
    ylim([-0.60, 0.80]);
    zlim([-0.10, 0.90]);
    xlabel('X [m]')
    ylabel('Y [m]')
    zlabel('Z [m]')
    title('Hand pose: perspective view')
    view(70,10)

    %Hand pose: front view
    % subplot(2,3,2)
    % grid on
    % hold on
    % plot3(hand_xyz_sync_RS(ii, 1), hand_xyz_sync_RS(ii, 2), hand_xyz_sync_RS(ii, 3),  '-*', 'MarkerSize', 10)
    % view(90,0)
    % xlim([-0.30, 0.60]);
    % ylim([-0.60, 0.80]);
    % zlim([-0.10, 0.90]);
    % xlabel('X [m]')
    % ylabel('Y [m]')
    % zlabel('Z [m]')
    % title('Hand pose: front view')
    % pic_color = readImage(image_RS_on{ii});

    %Gripper info
    subplot(2,3,2)
    plot(time_gripper_sync_RS (ii) - time_gripper_sync_RS(1), gripper_bool_sync_RS(ii), 'b.', 'MarkerSize', 20)
    yticks([0 1]); % Set y-axis ticks for 0 and 1
    yticklabels({'Closed', 'Open'}); % Label the ticks
    ylim([-0.5, 1.1]);
    grid on
    xlabel('Time [s]')
    %ylabel('Status')
    title('Gripper status: open/closed')
    
    %Camera 1 color stream
    subplot(2,3,3)
    %Cropping:
    pic_color = readImage(image_RS_on{ii});
    imshow(pic_color(min_height : max_height, min_width : max_width, :));

    %Camera1 depth stream
    subplot(2,3,4)
    pic_depth = readImage(images_depth_sync_RS{ii});
    max_depth = double(max(pic_depth(min_height : max_height, min_width : max_height), [],'all')); % Maximum possible depth value
    min_depth = double(min(pic_depth(min_height : max_height, min_width : max_height),[],'all')); % Minimum depth value

    % Avoid division by zero
    if max_depth > min_depth
        normalized_depth = (double(pic_depth(min_height : max_height, min_width : max_width)) - min_depth) / (max_depth - min_depth);
    else
        normalized_depth = zeros(size(pic_depth(min_height : max_height, min_width : max_height), 1)); % All values are the same
    end
    colormap = jet(256); % A 256-level colormap
    colored_depth = ind2rgb(round(normalized_depth * 255), colormap);
    imshow(colored_depth);

    %Camera 2 color stream
    subplot(2,3,5)
    %Cropping:
    pic_color2 = readImage(femto_color_sync{ii});
    imshow(pic_color2(min_height2 : max_height2, min_width2: max_width2, :));

    %Camera 2 depth stream
    % subplot(2,3,6)
    % pic_depth2 = readImage(femto_depth_sync{ii});
    % max_depth2 = double(max(pic_depth2(min_height_depth : max_height_depth, min_width_depth: max_height_depth), [],'all')); % Maximum possible depth value
    % min_depth2 = double(min(pic_depth2(min_height_depth : max_height_depth, min_width_depth : max_height_depth),[],'all')); % Minimum depth value
    % 
    %  % Avoid division by zero
    % if max_depth2 > min_depth2
    %     normalized_depth2 = (double(pic_depth2(min_height_depth : max_height_depth, min_width_depth : max_width_depth)) - min_depth2) / (max_depth2 - min_depth2);
    % else
    %     normalized_depth2 = zeros(size(pic_depth2(min_height_depth : max_height_depth, min_width_depth : max_height_depth), 1)); % All values are the same
    % end
    % colormap = jet(256); % A 256-level colormap
    % colored_depth2 = ind2rgb(round(normalized_depth2 * 255), colormap);
    % imshow(colored_depth2);

    name = source_path + "/gaze_images/" + frames_name(idx_sync(ii));
    % img_c1 = readImage(image_RS_on{kk});
    % img_femto = readImage(image_femto_on{idx_femto_n(kk)});
    %img_d1 = readImage(images_depth_sync_RS{kk});
    img = imread(name);
    subplot(2,3,6)
    imshow(img)

end

    close all




