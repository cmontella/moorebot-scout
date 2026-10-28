function testCameraRecovery(controlPort, cameraPort)
%TESTCAMERARECOVERY Run through the isolated mock-bridge Python runner.
%   python integrations/matlab/tests/run_camera_recovery.py --matlab PATH

clientDirectory = fileparts(fileparts(mfilename("fullpath")));
addpath(clientDirectory);
scout = MoorebotScout("127.0.0.1", controlPort, cameraPort, 0.4);
cleanup = onCleanup(@() delete(scout));
expected = uint8([255 216 255 217]);
names = ["partial header", "partial payload", "invalid length", "invalid markers"];

for index = 1:numel(names)
    caught = false;
    started = tic;
    try
        scout.readJpeg();
    catch cause
        caught = true;
        if index <= 2
            % MATLAB versions may throw their native read failure before
            % readExact receives an empty result. Preserve either error.
            assert(any(strcmp(cause.identifier, ...
                ["MoorebotScout:Connection", "MATLAB:networklib:tcpclient:readFailed"])), ...
                "Unexpected timeout error: %s", cause.identifier);
            assert(toc(started) < 5, "Camera timeout exceeded its deadline.");
        else
            assert(strcmp(cause.identifier, "MoorebotScout:Protocol"), ...
                "Unexpected framing error: %s", cause.identifier);
        end
    end
    assert(caught, "The %s fixture did not fail.", names(index));
    % Recovery must preserve the same control owner and reconnect the camera.
    scout.ping();
    assert(isequal(scout.readJpeg(), expected), ...
        "Camera did not recover after %s.", names(index));
    scout.ping();
    fprintf("PASS camera recovery after %s\n", names(index));
end
end
