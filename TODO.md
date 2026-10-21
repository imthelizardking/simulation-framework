# TODO

1. [ ] Add heating to seekers
2. [x] Add regression tests
3. [ ] Include .zdata read
4. [ ] Add burst type for munitions (natural, pre-fragmented, inert)
5. [ ] Add particle drag model code
6. [ ] Add 2D/3D object models
7. [ ] Improve object-particle hit algorithm using 2D/3D models
8. [ ] Add penetration algorithm
9. [ ] Assign material (or equivalent material) to targets
10. [x] Make `BaseAero` capable of obtaining data from various files
11. [x] Rename target objects to start with `Target`
12. [ ] Full pipeline: FCC → trajectory → burst → particle-target engagement (in progress)
13. [ ] New target block with realistic altitude-changing motion
14. [ ] Target with incoming-munition detection + evasive maneuvers
15. [ ] METCM reader (name unconfirmed)
16. [ ] IRDAM + balloon sensors (IRDAM name unconfirmed)
17. [x] Muzzle velocity measurement + burst-time correction (25/35/76mm)
18. [ ] Check MPMM equations
19. [ ] Check yaw-of-repose and spin-decay equations
20. [ ] Fixed-point iteration logic for approximation parameters
21. [ ] CCF logic algorithm (acronym unconfirmed)
22. [x] Use mils rather than degrees (launch angles)
23. [ ] Detailed model of IIR, SAL, GNSS, IMU
24. [ ] External-model adapter layer (Simulink done; third-party/`.exe` still open)
25. [ ] CG shift + verify CG input convention (calibers-from-nose)
26. [ ] Configurable axis reference convention (x,z,y option, new default), update plots
27. [ ] Standardize runner header comments (launch conditions, munition attrs, environment, target)
28. [ ] Stop using charge name/code in plot titles
29. [ ] NAD sensor object for muzzle velocity measurement with configurable error
30. [ ] Realistic timing/latency modeling (measurement time, calculation time, etc.)
31. [ ] Add QD, QM, IFORM, FL to equations and DATCOM reading logic
