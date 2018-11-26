close all
clear 
clc

% 
% vid = VideoReader('ExptVid0131_0001.avi');
% nFrames = vid.Numberofframes;
% 
% % vidWidth = vid.Width;
% % vidHeight = vid.Height;
% 
% f1 = 0;
% for f=1:nFrames
%   if (mod(f,5)==0)
%       f1 = f1+1;
%   thisframe=read(vid,f);
%   figure(1);imagesc(thisframe);
%   %thisfile=sprintf('C:\Users\lhuang28\Documents\GitHub\MagneticController\lihuang\imageprocessing\frame_%04d.jpg',f);
%   thisfile = sprintf('%04d.tif',f1);
%   imwrite(thisframe,thisfile);
%   end  
% end

cd('C:\Users\lhuang28\Documents\GitHub\MagneticController\lihuang\GlobalAgg\New folder')
                doVid = true;

                if doVid
                    writerObj = VideoWriter('Agg.mp4','MPEG-4');
                    writerObj.FrameRate = 60;
                    open(writerObj);
                end
                
                                    
                list = dir('Agg*.tif');
                for ii = 1:length(list)                                
                    frame = imread(list(ii).name);
                    writeVideo(writerObj,frame);

                end   
                                   
           if doVid
              close(writerObj);
           end
           
