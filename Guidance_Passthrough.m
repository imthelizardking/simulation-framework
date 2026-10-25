classdef Guidance_Passthrough < BaseGuidance
    % Relays sensor_bus straight into guid_bus, unchanged (plus
    % AccelCmd=0.0, matching Munition.updateGNC's own no-guidance
    % default, in case anything downstream expects that field).
    %
    % Exists because Munition.updateGNC only ever hands sensor_bus to
    % Guidance.step - Controller.step gets guid_bus/nav_state/veh_cfg/env,
    % never the raw sensor bus. A Controller that needs a sensor reading
    % (e.g. Controller_CourseCorrection reading guid_bus.MVR) needs
    % something sitting in the Guidance slot to carry it across that
    % boundary; this is the minimal, generic way to do that without
    % inventing real guidance logic for a round that doesn't otherwise
    % need any.
    methods
        function guid_bus = step(~, ~, ~, ~, sensor_bus)
            guid_bus = sensor_bus;
            guid_bus.AccelCmd = 0.0;
        end
    end
end
