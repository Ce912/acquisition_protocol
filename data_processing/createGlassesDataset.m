function [dataset_new] = createGlassesDataset(dataset_old, idx, image_name, gaze_coord, object_A, object_B, demo_num)

dataset_old{idx, 1} = image_name;
dataset_old{idx, 2} = gaze_coord(1); %Gaze x
dataset_old{idx, 3} = gaze_coord(2); %Gaze y
dataset_old{idx, 4} = object_A;
dataset_old{idx, 5} = object_B;
dataset_old{idx, 6} = demo_num;

dataset_new = dataset_old;
end


