% Display the Scout camera until the image window is closed.

clientDirectory = fileparts(fileparts(mfilename("fullpath")));
addpath(clientDirectory);

scout = MoorebotScout();
cleanup = onCleanup(@() delete(scout)); %#ok<NASGU>
window = figure("Name", "Moorebot Scout", "NumberTitle", "off");
picture = [];

while isgraphics(window)
    frame = scout.readImage();
    if isempty(picture) || ~isgraphics(picture)
        picture = image(frame);
        axis image off;
    else
        picture.CData = frame;
    end
    drawnow limitrate;
end
