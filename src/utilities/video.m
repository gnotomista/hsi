strVideo = sprintf('master_w%i_h%i',1920,1080);
strExtensionAvi = '.avi';
strExtensionMp4 = '.mp4';
strVidAvi = strcat(strVideo,strExtensionAvi);
strVidMp4 = strcat(strVideo,strExtensionMp4);
vidObjAvi = VideoWriter(strVidAvi);
vidObjAvi.FrameRate = 20;
% Open the video file.
disp('Opening the video ...')
open(vidObjAvi);
disp('Making the video...')
for k=1:length(master_frames)
% Write each frame to the file.
currFrame = master_frames(k);
writeVideo(vidObjAvi,currFrame);
end
% Close the video file.
disp('Closing the video...')
close(vidObjAvi);
disp('For some reason the following lines do not work if I execute them in MATLAB but they do work if you paste them one by one in your terminal:')
% Remove the old mp4 video
% system(sprintf('rm %s',strVidMp4))
% Compress the video created (from avi to mp4)
fprintf('$>: ffmpeg -i %s -c:v libx264 -crf 20 -preset veryslow -profile:v baseline -level 3.0 %s\n\n',strVidAvi,strVidMp4)
% % Remove the avi video created
% system(sprintf('rm %s',strVidAvi))
% Clear useless variables
clear strVideo strExtensionMp4 strExtensionAvi time distIJ distIJEst
strVideo = sprintf('slave_w%i_h%i',1920,1080);
strExtensionAvi = '.avi';
strExtensionMp4 = '.mp4';
strVidAvi = strcat(strVideo,strExtensionAvi);
strVidMp4 = strcat(strVideo,strExtensionMp4);
vidObjAvi = VideoWriter(strVidAvi);
vidObjAvi.FrameRate = 20;
% Open the video file.
disp('Opening the video ...')
open(vidObjAvi);
disp('Making the video...')
for k=1:length(slave_frames)
% Write each frame to the file.
currFrame = slave_frames(k);
writeVideo(vidObjAvi,currFrame);
end
% Close the video file.
disp('Closing the video...')
close(vidObjAvi);
disp('For some reason the following lines do not work if I execute them in MATLAB but they do work if you paste them one by one in your terminal:')
% Remove the old mp4 video
% system(sprintf('rm %s',strVidMp4))
% Compress the video created (from avi to mp4)
fprintf('$>: ffmpeg -i %s -c:v libx264 -crf 20 -preset veryslow -profile:v baseline -level 3.0 %s\n\n',strVidAvi,strVidMp4)
% % Remove the avi video created
% system(sprintf('rm %s',strVidAvi))
% Clear useless variables
clear strVideo strExtensionMp4 strExtensionAvi time distIJ distIJEst