% Refresh one velocity command for two seconds, then stop.
% For the first run, put the Scout on a stable stand with all wheels clear.

runContinuousCommands();

function runContinuousCommands()
% A function scope releases the client on completion, errors, or Ctrl-C.
clientDirectory = fileparts(fileparts(mfilename("fullpath")));
addpath(clientDirectory);

scout = MoorebotScout();
cleanup = onCleanup(@() delete(scout));
started = tic;

while toc(started) < 2.0
    scout.setVelocity(0.05, 0.0, 0.0, 0.25);
    pause(0.05);
end
scout.stop();
end
