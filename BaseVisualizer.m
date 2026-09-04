classdef (Abstract) BaseVisualizer < handle
    properties
        UpdateDecimation (1,1) double = 10
        StepCounter      (1,1) double = 0
        FigureHandle
    end
    methods
        function step(obj, t, X, target_state, gnc_bus)
            obj.StepCounter = obj.StepCounter + 1;
            if mod(obj.StepCounter, obj.UpdateDecimation) == 0
                obj.updatePlot(t, X, target_state, gnc_bus);
                drawnow limitrate;
            end
        end
    end
    methods (Abstract)
        init(obj, x0, target0, bounds)
        updatePlot(obj, t, X, target_state, gnc_bus)
        close(obj)
    end
end