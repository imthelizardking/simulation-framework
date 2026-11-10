classdef DynamicsCompound6DoF < Dynamics6DoF
    properties
        Servos cell % 12 adet Servo objesini tutacak
        AeroModel   % Hinge Moment okumak için Aero modeline opsiyonel referans
    end
    
    methods
        function obj = DynamicsCompound6DoF()
            obj = obj@Dynamics6DoF();
            obj.StateDimension = 33; % 21 (Base 6DoF) + 12 (Servos)
            obj.Servos = cell(12,1);
            for i = 1:12
                obj.Servos{i} = Servo_70mm(0.05); % 50ms gecikme
            end
        end
        
        function X0 = buildLaunchState(obj, launcher_pos, v0, elevation, azimuth, mass, p0, inertia)
            % Base 6-DoF launch state'ini al
            X0_base = buildLaunchState@Dynamics6DoF(obj, launcher_pos, v0, elevation, azimuth, mass, p0, inertia);
            % 12 adet servo başlangıç açısını (0) ekle
            X0 = [X0_base; zeros(12,1)]; 
        end
        
        function dX = computeDerivatives(obj, t, X, forces, moments, mass_prop, m_dot)
            % 1. Rijit Cisim Dinamikleri (İlk 21 state)
            dX_base = computeDerivatives@Dynamics6DoF(obj, t, X(1:21), forces, moments, mass_prop, m_dot);
            
            % 2. Servo Motor Dinamikleri (Son 12 state)
            dX_servos = zeros(12,1);
            
            % Otopilottan gelen 12'li komut vektörü
            cmd = forces.FinDeflection; 
            if isempty(cmd) || length(cmd) < 12
                cmd = zeros(12,1);
            end
            
            % Hinge Moment limitörü (Şimdilik delay çalışması için HM 0 varsayılıyor, 
            % ileride obj.AeroModel.HingeMoments okunarak eklenebilir)
            for i = 1:12
                dX_servos(i) = obj.Servos{i}.compute_derivative(X(21+i), cmd(i), 0);
            end
            
            % 3. Birleştirilmiş 33'lük türev vektörü
            dX = [dX_base; dX_servos];
        end
    end
end