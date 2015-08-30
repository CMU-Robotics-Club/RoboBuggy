function textureSegment(I)
E = entropyfilt(I);
Eim = mat2gray(E);
%   g = rgb2gray(Eim);
%    getbigedge(g);
%    Ed= edge(g,'canny',.7);
%    Ed = bwareaopen(Ed, 100);
subplot(2,1,1)
imshow(Eim);
subplot(2,1,2)
imshow(I)


% colorSpaceKmean(Eim)
% pause()
end