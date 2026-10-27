classdef TestUavSeedPlanting < matlab.unittest.TestCase
    methods (Test)
        function defaultScenarioReleasesAndLandsSeeds(tc)
            randomState = rng;
            r = runner_uav_seed_planting(false);
            % Local dispenser randomness must leave the caller's stream intact.
            tc.verifyEqual(rng, randomState);
            tc.verifyGreaterThan(r.releasePosition(3), 0);
            tc.verifyGreaterThan(r.releasePosition(1), 0);
            tc.verifyTrue(all(isfinite(r.telemetry.pos(:))));
            tc.verifyEqual(r.telemetry.mass(1)-r.telemetry.mass(end),40*2e-5,'AbsTol',1e-12);
            tc.verifyEqual(r.telemetry.mass(end),12,'AbsTol',1e-12);
            tc.verifyEqual(size(r.seeds.landingPositions), [40 3]);
            tc.verifyTrue(all(r.seeds.landed), 'Default example must complete the seed drop.');
            tc.verifyTrue(all(isfinite(r.seeds.landingPositions(:))));
            tc.verifyEqual(r.seeds.landingPositions(:,3), zeros(40,1), 'AbsTol', 1e-12);
            tc.verifyGreaterThan(r.seeds.landingTimes, r.releaseTime);
            % Every seed must start at the dispenser's airborne position,
            % not at the target or at a NED coordinate interpreted as altitude.
            starts = reshape(r.seeds.positions(1,:,:), 3, 40)';
            tc.verifyEqual(starts, repmat(r.releasePosition', 40, 1), 'AbsTol', 1e-12);
            tc.verifyGreaterThanOrEqual(r.seeds.positions(:,3,:), 0);
        end
    end
end
