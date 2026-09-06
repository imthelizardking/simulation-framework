%% File: ParticleModelFactory.m
classdef ParticleModelFactory
    % Fragmentation-model presets, one per caliber this framework already
    % models (25mm/35mm/76mm - see MunitionFactory). All values (fragment
    % count, radius, axial span, mass, CD) are illustrative order-of-
    % magnitude placeholders scaled to caliber, not transcribed from a real
    % fragmentation/arena test - same caveat as MunitionFactory/
    % ChargeFactory's other illustrative presets. Positions is the N-by-3
    % body-frame matrix BaseParticleModel expects; callers can also build a
    % ForwardExplosionParticleModel directly with their own custom
    % Positions matrix instead of using these presets.
    methods (Static)

        %% 25mm HE - small fragment count/size, 2 rings of 8
        function model = createForward25mm()
            radius     = 0.32 * 0.025; % m, just inside the 25mm casing wall
            axial_span = 0.05;         % m
            positions  = ParticleModelFactory.cylindricalPositions(2, 8, radius, axial_span);
            dims = struct('Diameter', 0.004, 'Length', 0.006);
            model = ForwardExplosionParticleModel(positions, 'Cylinder', dims, 1.1, 0.0008, 'Max');
        end

        %% 35mm HE-T - 4 rings of 8
        function model = createForward35mm()
            radius     = 0.34 * 0.035;
            axial_span = 0.09;
            positions  = ParticleModelFactory.cylindricalPositions(4, 8, radius, axial_span);
            dims = struct('Diameter', 0.006, 'Length', 0.009);
            model = ForwardExplosionParticleModel(positions, 'Cylinder', dims, 1.15, 0.0025, 'Max');
        end

        %% 76mm HE - 8 rings of 8, cube-shaped fragments (exercises the other shape)
        function model = createForward76mm()
            radius     = 0.34 * 0.076;
            axial_span = 0.20;
            positions  = ParticleModelFactory.cylindricalPositions(8, 8, radius, axial_span);
            dims = struct('Side', 0.008);
            model = ForwardExplosionParticleModel(positions, 'Cube', dims, 1.05, 0.006, 'Max');
        end

    end

    methods (Static, Access = private)
        function positions = cylindricalPositions(num_rings, per_ring, radius, axial_span)
            % Rings of particles evenly spaced in azimuth around the body
            % axis, rings evenly spaced along the axial span (centered on
            % the CG, so this matrix already sums to ~zero moment about the
            % CG). Column 1 = axial offset, columns 2-3 = lateral offsets.
            if num_rings > 1
                axial = linspace(-axial_span/2, axial_span/2, num_rings);
            else
                axial = 0;
            end
            theta = linspace(0, 2*pi, per_ring + 1);
            theta = theta(1:end-1); % drop the duplicate 2*pi point

            positions = zeros(num_rings * per_ring, 3);
            row = 1;
            for i = 1:num_rings
                for j = 1:per_ring
                    positions(row, :) = [axial(i), radius*cos(theta(j)), radius*sin(theta(j))];
                    row = row + 1;
                end
            end
        end
    end
end
