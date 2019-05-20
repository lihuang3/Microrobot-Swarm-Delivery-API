skel = obj.Skel;
bw = obj.BW;
bw = bw - skel;
RGB = double(cat(3, ~bw, ~bw, ~bw));
rgb_skel = double(cat(3, skel, skel, skel));
RGB(:,:,1) = RGB(:,:,1).*182./255+ double(bw);
RGB(:,:,2) = RGB(:,:,2).*228./255+ double(bw);
RGB(:,:,3) = RGB(:,:,3).*255./255+ double(bw);                
RGB(rgb_skel>0) = 0;
imshow(RGB)
gcf=figure(1);
set(gcf, 'Position', [100,100, 1000, 1000*474/474]);
figure(1); hold on
f1 = scatter(obj.BrchPts0(:,2),obj.BrchPts0(:,1),40,'filled');
f1.CData = [1 0 0];
f2 = scatter(obj.EdPts0(:,2),obj.EdPts0(:,1),40,'filled');
f2.CData = [0 0 1];
imwrite(frame2im(getframe), 'VBskel.png');
