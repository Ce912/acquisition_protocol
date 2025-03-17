function padded_image = resize_with_aspect_ratio(image, target_width, target_height)
%Resize depth and color streams to get one-to-one correspondence between depth cells and color pixels
    % Get original dimensions
    [h, w, ~] = size(image);
    
    % Calculate scaling factors
    scale_w = target_width / w;
    scale_h = target_height / h;
    scale = min(scale_w, scale_h); % Preserve aspect ratio
    
    % Calculate new dimensions
    new_w = round(w * scale);
    new_h = round(h * scale);
    
    % Resize the image
    resized = imresize(image, [new_h, new_w], 'nearest'); % Nearest-neighbor interpolation
    
    % Calculate padding
    delta_w = target_width - new_w;
    delta_h = target_height - new_h;
    top = floor(delta_h / 2);
    bottom = delta_h - top;
    left = floor(delta_w / 2);
    right = delta_w - left;
    
    % Pad the resized image to match the target dimensions
    if size(image, 3) == 1  % Grayscale image
        padded_image = padarray(resized, [top, left], 0, 'pre'); % Add padding on top and left
        padded_image = padarray(padded_image, [bottom, right], 0, 'post'); % Add padding on bottom and right
    else  % Color image
        padded_image = padarray(resized, [top, left, 0], 0, 'pre'); % Add padding on top and left
        padded_image = padarray(padded_image, [bottom, right, 0], 0, 'post'); % Add padding on bottom and right
    end
end