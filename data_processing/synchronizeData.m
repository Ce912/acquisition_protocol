function [time_gripper_sync, time_hand_sync, time_rgbd_sync, time_head_sync, time_objA_sync, time_objB_sync, time_color2_sync, time_depth2_sync, images_color2_sync,images_depth2_sync, gripper_bool_sync, hand_xyz_sync, hand_xyz_corr_sync, hand_angles_corr, images_depth_sync, head_xyz_sync, objA_sync, objB_sync ] = ...
    synchronizeData(time_ref, image_color_ref, image_depth, time_gripper, time_hand, time_head, time_rgbd, time_objA, time_objB, time_color2, time_depth2, images_color2, images_depth2, gripper_bool, hand_xyz, hand_xyz_corr, hand_orient_angles, head_xyz,  object_A, object_B)


%Initlize empty arrays for synchronized time and data
%Gripper
time_gripper_sync = zeros(1, length(image_color_ref));
gripper_bool_sync= zeros(1, length(image_color_ref));

%Camera 2 color
time_color2_sync = zeros(1, length(image_color_ref));
images_color2_sync = cell(length(image_color_ref), 1); %Check dimension

%Camera 2 depth
time_depth2_sync = zeros(1, length(image_color_ref));
images_depth2_sync = cell(length(image_color_ref), 1); %Check dimension

%Hand pose
time_hand_sync = zeros(1, length(image_color_ref));
hand_xyz_sync = zeros(length(image_color_ref), 3);
hand_xyz_corr_sync = zeros(length(image_color_ref), 3);
hand_angles_corr= zeros(length(image_color_ref), 3);

%Head pose
time_head_sync = zeros(1, length(image_color_ref));
head_xyz_sync= zeros(length(image_color_ref), 3);

%Depth Images camera1
time_rgbd_sync = zeros(1, length(image_color_ref));
images_depth_sync = cell(length(image_color_ref), 1); %Check dimension

%ObjectA and objectB
time_objA_sync = zeros(1, length(image_color_ref));
objA_sync= cell(length(image_color_ref),1);
time_objB_sync = zeros(1, length(image_color_ref));
objB_sync= cell(length(image_color_ref),1);

%Synchronize wrt camera1
for kk = 1: length(image_color_ref)

    %Synchronize gripper
    [~, idx_min_grip] = min(abs(time_gripper - time_ref(kk)));
    time_gripper_sync(kk) = time_gripper(idx_min_grip);
    gripper_bool_sync(kk) = gripper_bool(idx_min_grip);

    %Synchronize hand pose and corrected hand pose
    [~, idx_min_hand] = min(abs(time_hand - time_ref(kk)));
    time_hand_sync(kk) = time_hand(idx_min_hand);
    hand_xyz_sync(kk, :) = hand_xyz(idx_min_hand, :);
    hand_xyz_corr_sync(kk, :) = hand_xyz_corr(idx_min_hand, :);
    hand_angles_corr(kk, :) = hand_orient_angles(idx_min_hand, :);

    %Synchronize depth images camera1
    [~, idx_min_depth] = min(abs(time_rgbd - time_ref(kk)));
    time_rgbd_sync(kk) = time_rgbd(idx_min_depth);
    images_depth_sync{kk} = image_depth{idx_min_depth};

    %Synchronize color images camera2
    [~, idx_min_color2] = min(abs(time_color2 - time_ref(kk)));
    time_color2_sync(kk) = time_color2(idx_min_color2);
    images_color2_sync{kk} = images_color2{idx_min_color2};
    
    %Synchronize depth images camera2
    [~, idx_min_depth2] = min(abs(time_depth2 - time_ref(kk)));
    time_depth2_sync(kk) = time_depth2(idx_min_depth2);
    images_depth2_sync{kk} = images_depth2{idx_min_depth2};
    
    %Synchronize head pose
    [~, idx_min_head] = min(abs(time_head - time_ref(kk)));
    time_head_sync(kk) = time_head(idx_min_head);
    head_xyz_sync(kk, :) = head_xyz(idx_min_head, :);

    %Synchronize object A
    [~, idx_min_objA] = min(abs(time_objA - time_ref(kk)));
    time_objA_sync(kk) = time_objA(idx_min_objA);
    objA_sync{kk, 1} = object_A{idx_min_objA, 1};

    %Synchronize object B
    [~, idx_min_objB] = min(abs(time_objB - time_ref(kk)));
    time_objB_sync(kk) = time_objB(idx_min_objB);
    objB_sync{kk, 1} = object_B{idx_min_objB, 1};

end
end