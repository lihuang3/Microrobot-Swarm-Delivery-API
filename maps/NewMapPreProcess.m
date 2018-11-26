imshow('maze4.png');
fig = getframe;
close(figure(1))
fig = fig.cdata;
bw = im2bw(fig,0.2);



bw1 = imresize(bw,0.4);
figure
imshow(~bw1)
figure
rgb1 = uint8(cat(3,bw1,bw1,bw1));
rgb0 = uint8(cat(3,~bw1,~bw1,~bw1));
tmp = rgb1(:,:,1)==1;
tmp0 = rgb0(:,:,1)==1;
tmp = uint8(tmp);
tmp0 = uint8(tmp0);
rgb1(:,:,1) = rgb1(:,:,1).*tmp.*182;
rgb0(:,:,1) = rgb0(:,:,1).*tmp0.*255;
rgb1(:,:,2)=rgb1(:,:,2).*tmp.*228;
rgb0(:,:,2) = rgb0(:,:,2).*tmp0.*255;
rgb1(:,:,3)=rgb1(:,:,3).*tmp.*255;
rgb0(:,:,3) = rgb0(:,:,3).*tmp0.*255;

rgb = rgb1+rgb0;


imshow(rgb)
