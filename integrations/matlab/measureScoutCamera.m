function samples = measureScoutCamera(scout, count)
%MEASURESCOUTCAMERA Time frame receipt and JPEG decoding separately.
%   samples = measureScoutCamera(scout) measures 30 frames using an existing
%   MoorebotScout connection. It does not open another control connection,
%   display images, or send movement commands.
%
%   Receipt time includes waiting for a new camera frame and transferring
%   its bytes; it is not a measurement of network latency alone.

arguments
    scout (1,1) MoorebotScout
    count (1,1) double {mustBeFinite, mustBeInteger, mustBePositive} = 30
end

% Keep connection setup and Java initialization out of steady-state samples.
started = tic;
jpeg = scout.readJpeg();
firstReceive = toc(started);
started = tic;
decodeScoutJpeg(jpeg);
firstDecode = toc(started);

receiveSeconds = zeros(count, 1);
decodeSeconds = zeros(count, 1);
jpegBytes = zeros(count, 1);
for index = 1:count
    started = tic;
    jpeg = scout.readJpeg();
    receiveSeconds(index) = toc(started);
    started = tic;
    decodeScoutJpeg(jpeg);
    decodeSeconds(index) = toc(started);
    jpegBytes(index) = numel(jpeg);
end

samples = table(receiveSeconds, decodeSeconds, jpegBytes, ...
    "VariableNames", ["ReceiveSeconds", "DecodeSeconds", "JpegBytes"]);
fprintf("First frame: receive %.1f ms, decode %.1f ms.\n", ...
    1000 * firstReceive, 1000 * firstDecode);
fprintf("Next %d frames: receive %.1f ms, decode %.1f ms on average.\n", ...
    count, 1000 * mean(receiveSeconds), 1000 * mean(decodeSeconds));
fprintf("Receipt + decoding: %.1f frames/s; display/analysis excluded.\n", ...
    1 / mean(receiveSeconds + decodeSeconds));
fprintf("Receipt includes waiting for the Scout's next camera frame.\n");
end
