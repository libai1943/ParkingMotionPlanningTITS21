function SaveFigure(imgname)
frame = getframe(gcf);
im = frame2im(frame);
imwrite(im, imgname);
end