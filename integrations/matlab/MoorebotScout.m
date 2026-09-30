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
        CameraPort (1,1) double
        Timeout (1,1) double
        Control
        Camera
        Closed (1,1) logical = false
    end

    methods
        function obj = MoorebotScout(host, controlPort, cameraPort, timeout)
            arguments
                host (1,1) string = "127.0.0.1"
                controlPort (1,1) double {mustBeInteger, mustBePositive} = 43000
                cameraPort (1,1) double {mustBeInteger, mustBePositive} = 43001
                timeout (1,1) double {mustBePositive} = 5.0
            end

            obj.Host = host;
            obj.CameraPort = cameraPort;
            obj.Timeout = timeout;
            obj.Control = tcpclient(host, controlPort, "Timeout", timeout);
            configureTerminator(obj.Control, "LF");
            hello = obj.readResponse();
            if ~isfield(hello, "type") || string(hello.type) ~= "hello" || ...
                    ~isfield(hello, "protocol") || hello.protocol ~= 1
                delete(obj);
                error("MoorebotScout:Protocol", ...
                    "The local process returned an unsupported bridge greeting.");
            end
            obj.MaxCommandTimeout = double(hello.max_command_timeout_ms) / 1000;
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
            if obj.Closed || isempty(obj.Control)
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
            try
                obj.stop();
            catch
                % The Rust bridge's deadman still stops a disconnected client.
            end
            obj.Closed = true;
            obj.Camera = [];
            obj.Control = [];
        end
    end

    methods (Access = private)
        function response = request(obj, request)
            obj.assertOpen();
            writeline(obj.Control, jsonencode(request));
            response = obj.readResponse();
        end

        function response = readResponse(obj)
            line = readline(obj.Control);
            if isempty(line)
                error("MoorebotScout:Connection", ...
                    "The bridge did not return a control response.");
            end
            try
                response = jsondecode(char(line));
            catch cause
                error("MoorebotScout:Protocol", ...
                    "The bridge returned invalid JSON: %s", cause.message);
            end
            if ~isstruct(response) || ~isfield(response, "ok") || ~response.ok
                message = "The bridge rejected the request.";
                if isstruct(response) && isfield(response, "message")
                    message = string(response.message);
                end
                error("MoorebotScout:Bridge", "%s", message);
            end
        end

        function bytes = readExact(obj, connection, count)
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
