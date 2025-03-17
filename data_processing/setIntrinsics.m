function [undist_img_rgb, undist_img_depth] = setIntrinsics (img_color, img_depth, camera_model)

% Set intrinsic parameters from calibration to undistort the images according to camera model
%camera = toStruct(cameraParameters);

if camera_model == "femto"

    fx =742.97;
    fy = 741.38;
    cx = 640.16;
    cy = 363.72;
    k1 = 6.38e-02;
    k2 = -2.22e-01;
    k3 = 5.804e-01;
    p1= 3.52e-03;
    p2 = -3.87e-04;

    % Reshape in matlab camera matrix format
    K = [ fx, 0, cx;
        0, fy,  cy;
        0,  0 , 1];
    RadialDistortion = [k1, k2, k3];
    TangentialDistortion = [p1, p2];
    camera = cameraParameters("K",K, "TangentialDistortion", TangentialDistortion, "RadialDistortion", RadialDistortion(1:end) );


    [undist_img_rgb, ~]  = undistortImage(img_color, camera);
    [undist_img_depth, ~] = undistortImage(img_depth, camera);
    % undist_img_depth = img_depth;


elseif camera_model == "realsense"

    % 
    % fx = 472.38;
    % fy = 472.01;
    % cx = 460.59;
    % cy = 240.66;
    % k1 = 1.071e-01;
    % k2 = -1.5260e+00;
    % k3 = 5.1291e+00;
    % p1 =  -2.4e-03;
    % p2 = 2.58e-02;
    fx = 437.83;
    fy = 436.75;
    cx = 414.79;
    cy = 245.21;
    k1 = -8.70544701e-02;
    k2 =  1.60496712e-01;
    p1 =  6.01124850e-04;
    p2 =  -1.17599595e-04;
    k3 =  -1.36271718e-01;

    K = [ fx, 0, cx;
            0, fy,  cy;
            0,  0 , 1];
    RadialDistortion = [k1, k2, k3];
    TangentialDistortion = [p1, p2];

    camera = cameraParameters("K",K, "TangentialDistortion", TangentialDistortion,"RadialDistortion", RadialDistortion(1:3));
    [undist_img_rgb, ~]  = undistortImage(img_color, camera);
    [undist_img_depth, ~] = undistortImage(img_depth, camera);
    
end
end







