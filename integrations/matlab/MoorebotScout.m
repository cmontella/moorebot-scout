classdef MoorebotScout < handle
    %MOOREBOTSCOUT Client for a running `moorebot-scout bridge` process.
    %
    %   scout = MoorebotScout() connects to the default localhost ports.
    %   Velocity commands use [vx, vy, vtheta], where positive vx is
    %   forward, positive vy is left, and positive vtheta is a
    %   counter-clockwise turn.

    properties (SetAccess = private)
        MaxCommandTimeout (1,1) double = 1.0
    end

    properties (Access = private)
        Host (1,1) string
        ControlPort (1,1) double
        CameraPort (1,1) double
        Timeout (1,1) double
        Control
        Camera
        ControlBuffer (1,:) uint8 = uint8([])
        HandshakeComplete (1,1) logical = false
        Closed (1,1) logical = false
    end

    methods
        function obj = MoorebotScout(host, controlPort, cameraPort, timeout)
            arguments
                host (1,1) string = "127.0.0.1"
                controlPort (1,1) double {mustBeInteger, mustBePositive} = 43000
                cameraPort (1,1) double {mustBeInteger, mustBePositive} = 43001
                timeout (1,1) double {mustBeFinite, mustBePositive} = 5.0
            end

            obj.Host = host;
            obj.ControlPort = controlPort;
            obj.CameraPort = cameraPort;
            obj.Timeout = timeout;
            try
                obj.Control = tcpclient(host, controlPort, "Timeout", timeout);
                configureTerminator(obj.Control, "LF");
                hello = obj.readResponse();
                if ~isfield(hello, "type") || ~isequal(hello.type, 'hello') || ...
                        ~isfield(hello, "protocol") || ~isequal(hello.protocol, 1) || ...
                        ~isfield(hello, "max_command_timeout_ms") || ...
                        ~isnumeric(hello.max_command_timeout_ms) || ...
                        ~isscalar(hello.max_command_timeout_ms) || ...
                        ~isfinite(hello.max_command_timeout_ms) || ...
                        hello.max_command_timeout_ms <= 0
                    error("MoorebotScout:Protocol", ...
                        "The local process returned an unsupported bridge greeting.");
                end
                obj.MaxCommandTimeout = double(hello.max_command_timeout_ms) / 1000;
                obj.HandshakeComplete = true;
            catch cause
                % A rejected or silent connection never acquired control.
                % Do not send stop and wait for a second response during delete.
                obj.closeSockets();
                rethrow(cause);
            end
        end

        function applied = setVelocity(obj, vx, vy, vtheta, timeout)
            %SETVELOCITY Send [vx, vy, vtheta] with a short deadman timeout.
            arguments
                obj
                vx (1,1) double {mustBeFinite}
                vy (1,1) double {mustBeFinite}
                vtheta (1,1) double {mustBeFinite}
                timeout (1,1) double {mustBeFinite, mustBePositive} = 0.5
            end
            if timeout > obj.MaxCommandTimeout
                error("MoorebotScout:Timeout", ...
                    "Command timeout may not exceed %.3g seconds.", ...
                    obj.MaxCommandTimeout);
            end
            request = struct( ...
                "type", "velocity", ...
                "vx", vx, ...
                "vy", vy, ...
                "vtheta", vtheta, ...
                "timeout_ms", max(1, round(timeout * 1000)));
            response = obj.request(request);
            applied = double(response.applied(:).');
            if numel(applied) ~= 3
                error("MoorebotScout:Protocol", ...
                    "The bridge returned an invalid velocity response.");
            end
        end

        function stop(obj)
            %STOP Immediately replace the active command with zero velocity.
            if obj.Closed || ~obj.HandshakeComplete || isempty(obj.Control)
                return
            end
            response = obj.request(struct("type", "stop"));
            if ~isfield(response, "type") || string(response.type) ~= "stop"
                error("MoorebotScout:Protocol", ...
                    "The bridge returned an invalid stop response.");
            end
        end

        function ping(obj)
            %PING Check that the command bridge is responsive.
            response = obj.request(struct("type", "ping"));
            if ~isfield(response, "type") || string(response.type) ~= "pong"
                error("MoorebotScout:Protocol", ...
                    "The bridge returned an invalid ping response.");
            end
        end

        function jpeg = readJpeg(obj)
            %READJPEG Request the newest frame as a row vector of JPEG bytes.
            obj.assertOpen();
            if isempty(obj.Camera)
                obj.Camera = tcpclient(obj.Host, obj.CameraPort, ...
                    "Timeout", obj.Timeout);
            end
            write(obj.Camera, uint8(1), "uint8");
            header = obj.readExact(obj.Camera, 4);
            length = double(header(1)) * 16777216 + ...
                double(header(2)) * 65536 + ...
                double(header(3)) * 256 + double(header(4));
            if length < 1 || length > 16 * 1024 * 1024
                error("MoorebotScout:Protocol", ...
                    "The bridge returned an invalid JPEG length: %d.", length);
            end
            jpeg = obj.readExact(obj.Camera, length);
            if numel(jpeg) < 4 || ~isequal(jpeg(1:2), uint8([255 216])) || ...
                    ~isequal(jpeg(end-1:end), uint8([255 217]))
                error("MoorebotScout:Protocol", ...
                    "The bridge returned data without JPEG markers.");
            end
        end

        function frame = readImage(obj)
            %READIMAGE Request and decode the newest frame as an RGB array.
            frame = decodeScoutJpeg(obj.readJpeg());
        end

        function delete(obj)
            if obj.Closed
                return
            end
            if obj.HandshakeComplete
                try
                    obj.stop();
                catch
                    % Disconnect and the bridge deadman also stop movement.
                end
            end
            obj.closeSockets();
        end
    end

    methods (Access = private)
        function response = request(obj, request)
            obj.assertOpen();
            try
                writeline(obj.Control, jsonencode(request));
                response = obj.readResponse();
            catch cause
                if ~strcmp(cause.identifier, "MoorebotScout:Bridge")
                    % After a lost reply the request/response stream is no
                    % longer synchronized. Close it before any later command.
                    obj.closeSockets();
                end
                rethrow(cause);
            end
        end

        function response = readResponse(obj)
            line = obj.readControlLine();
            try
                response = jsondecode(char(line));
            catch cause
                error("MoorebotScout:Protocol", ...
                    "The bridge returned invalid JSON: %s", cause.message);
            end
            if ~isstruct(response) || ~isscalar(response) || ...
                    ~isfield(response, "ok") || ~isequal(response.ok, true)
                message = "The bridge rejected the request.";
                if isstruct(response) && isscalar(response) && isfield(response, "message")
                    message = string(response.message);
                end
                if isstruct(response) && isscalar(response) && ...
                        isfield(response, "code") && isequal(response.code, 'busy')
                    error("MoorebotScout:Busy", "%s", message);
                end
                error("MoorebotScout:Bridge", "%s", message);
            end
        end

        function line = readControlLine(obj)
            % Read only bytes already available, with one deadline per line.
            % This avoids readline's warning and an extra destructor timeout.
            started = tic;
            maximum = 4096;
            while true
                newline = find(obj.ControlBuffer == 10, 1);
                if ~isempty(newline)
                    line = obj.ControlBuffer(1:newline-1);
                    obj.ControlBuffer(1:newline) = [];
                    return
                end
                if numel(obj.ControlBuffer) >= maximum
                    error("MoorebotScout:Protocol", ...
                        "The bridge control response exceeds 4096 bytes.");
                end
                if toc(started) >= obj.Timeout
                    if ~obj.HandshakeComplete
                        error("MoorebotScout:Connection", ...
                            "No bridge greeting from %s:%d. Check the Rust terminal says " + ...
                            "'Integration bridge ready', use the commands port (normally 43000), " + ...
                            "and close any other MATLAB or Python Scout client before reconnecting.", ...
                            obj.Host, obj.ControlPort);
                    end
                    error("MoorebotScout:Connection", ...
                        "The bridge did not return a control response. " + ...
                        "The connection has been closed; check the bridge terminal before reconnecting.");
                end
                available = obj.Control.NumBytesAvailable;
                if available > 0
                    count = min(available, maximum - numel(obj.ControlBuffer));
                    chunk = read(obj.Control, count, "uint8");
                    obj.ControlBuffer = [obj.ControlBuffer, reshape(chunk, 1, [])];
                else
                    pause(min(0.005, max(0, obj.Timeout - toc(started))));
                end
            end
        end

        function closeSockets(obj)
            obj.Closed = true;
            obj.HandshakeComplete = false;
            obj.Camera = [];
            obj.Control = [];
            obj.ControlBuffer = uint8([]);
        end

        function bytes = readExact(~, connection, count)
            bytes = zeros(1, count, "uint8");
            offset = 0;
            while offset < count
                chunk = read(connection, count - offset, "uint8");
                if isempty(chunk)
                    error("MoorebotScout:Connection", ...
                        "Timed out while reading camera data.");
                end
                bytes(offset + (1:numel(chunk))) = chunk;
                offset = offset + numel(chunk);
            end
        end

        function assertOpen(obj)
            if obj.Closed || isempty(obj.Control)
                error("MoorebotScout:Closed", "The Scout client is closed.");
            end
        end
    end
end
