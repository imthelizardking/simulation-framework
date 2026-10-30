function aero = createCivilianUAVExampleAero()
%CREATECIVILIANUAVEXAMPLEAERO Synthetic data for software demonstration only.
% Independent of the DATCOM parser, datasets, and vehicle factories.
    stowed = struct('CD0',0.06,'CL_alpha',1,'Cm_alpha',-0.1,'Cm_q',-1, ...
        'Cl_p',-0.1,'Cn_r',-0.1,'CY_beta',-0.2,'Cn_beta',0.05);
    deployed = struct('CD0',0.10,'CL_alpha',4,'Cm_alpha',-0.4,'Cm_q',-3, ...
        'Cl_p',-0.3,'Cn_r',-0.3,'CY_beta',-0.6,'Cn_beta',0.15, ...
        'InducedDrag',0.04,'CL_elevator',0.2,'Cm_elevator',-0.2, ...
        'Cl_aileron',0.1,'Cn_rudder',-0.1);
    aero = AeroCivilianUAV();
    aero.setConfiguration('Stowed',stowed);
    aero.setConfiguration('WingsOpen',deployed);
    aero.ActiveTableKey = 'Stowed';
    aero.DeployedTableKey = 'WingsOpen';
end
