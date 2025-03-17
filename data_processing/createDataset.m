%% Save data in a table to create the dataset


function [dataset_new] = createDataset( dataset_old, idx, eof_pose, head_pose, eof_orient, gripper_status, obj1, obj2, camera1_rgb_name, camera1_depth_name, camera2_rgb_name, camera2_depth_name, task_type, delta_pose, delta_rot, ndemo, is_initial, is_final)

dataset_old{idx, 1}  = idx - 1; %STEP
dataset_old{idx, 2} = ['x: ', num2str(eof_pose(1,1)), ' y: ', num2str(eof_pose(1,2)), ' z: ', num2str(eof_pose(1,3))];
dataset_old{idx, 3} = ['x: ', num2str(eof_orient(1,1)), ' y: ', num2str(eof_orient(1,2)), ' z: ', num2str(eof_orient(1,3))];
dataset_old{idx, 4} = [];
dataset_old{idx, 5} = [];

if gripper_status == 1
    dataset_old{idx, 6} = 'open';
elseif gripper_status == 0
    dataset_old{idx, 6} = 'closed';
end

dataset_old{idx, 7} = string(obj1);
dataset_old{idx, 8} = string(obj2);
dataset_old{idx, 9} = string(task_type);
dataset_old{idx, 10} = [camera1_rgb_name camera1_depth_name];
dataset_old{idx, 11} = [camera2_rgb_name camera2_depth_name];
dataset_old{idx, 12} = ['x: ', num2str(delta_pose(1,1)), ' y: ', num2str(delta_pose(1,2)), ' z: ', num2str(delta_pose(1,3))];
dataset_old{idx, 13} = ['x: ', num2str(delta_rot(1,1)), ' y: ', num2str(delta_rot(1,2)), ' z: ', num2str(delta_rot(1,3))];
dataset_old{idx, 14} = ndemo;
dataset_old{idx, 15} = ['x: ', num2str(head_pose(1,1)), ' y: ', num2str(head_pose(1,2)), ' z: ', num2str(head_pose(1,3))];
dataset_old{idx, 16} = [];
dataset_old{idx, 17} = is_initial;
dataset_old{idx, 18} = is_final;

dataset_new = dataset_old; 
end

