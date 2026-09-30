function rgb = decodeScoutJpeg(jpeg)
%DECODESCOUTJPEG Decode in-memory JPEG bytes without temporary files.

arguments
    jpeg (1,:) uint8
end

if ~usejava("jvm")
    error("MoorebotScout:JavaRequired", ...
        "JPEG decoding requires desktop MATLAB with its Java runtime enabled.");
end

stream = java.io.ByteArrayInputStream(typecast(jpeg(:), "int8"));
cleanup = onCleanup(@() stream.close()); %#ok<NASGU>
buffered = javax.imageio.ImageIO.read(stream);
if isempty(buffered)
    error("MoorebotScout:Jpeg", "MATLAB could not decode the Scout JPEG.");
end

width = buffered.getWidth();
height = buffered.getHeight();
argb = buffered.getRGB(0, 0, width, height, [], 0, width);
argb = typecast(int32(argb), "uint32");

red = uint8(bitand(bitshift(argb, -16), uint32(255)));
green = uint8(bitand(bitshift(argb, -8), uint32(255)));
blue = uint8(bitand(argb, uint32(255)));

rgb = zeros(height, width, 3, "uint8");
rgb(:, :, 1) = reshape(red, width, height).';
rgb(:, :, 2) = reshape(green, width, height).';
rgb(:, :, 3) = reshape(blue, width, height).';
end
