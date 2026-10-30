# TODO

1. [ ] 3d model target
2. [ ] Read prods formatted output
3. [ ] Add heating to seekers
4. [x] Add regression tests
5. [ ] Include .zdata read
6. [ ] Add burst type for munitions (natural, pre-fragmented, inert)
7. [ ] Add particle drag model code
8. [ ] Add 2D/3D object models
9. [ ] Improve object-particle hit algorithm using 2D/3D models
10. [ ] Add penetration algorithm
11. [ ] Assign material (or equivalent material) to targets
12. [x] Make `BaseAero` capable of obtaining data from various files
13. [x] Rename target objects to start with `Target`
14. [X] Full pipeline: FCC → trajectory → burst → particle-target engagement (in progress)
15. [ ] New target block with realistic altitude-changing motion
16. [ ] Target with incoming-munition detection + evasive maneuvers
17. [X] METCM reader (name unconfirmed)
18. [ ] IRDAM + balloon sensors (IRDAM name unconfirmed)
19. [x] Muzzle velocity measurement + burst-time correction (25/35/76mm)
20. [X] Check MPMM equations
21. [ ] Check yaw-of-repose and spin-decay equations
22. [ ] Fixed-point iteration logic for approximation parameters
23. [ ] CCF logic algorithm (acronym unconfirmed)
24. [x] Use mils rather than degrees (launch angles)
25. [ ] Detailed model of IIR, SAL, GNSS, IMU
26. [X] External-model adapter layer (Simulink done; third-party/`.exe` still open)
27. [ ] CG shift + verify CG input convention (calibers-from-nose)
28. [ ] Configurable axis reference convention (x,z,y option, new default), update plots
29. [X] Standardize runner header comments (launch conditions, munition attrs, environment, target)
30. [X] Stop using charge name/code in plot titles
31. [ ] NAD sensor object for muzzle velocity measurement with configurable error
32. [ ] Realistic timing/latency modeling (measurement time, calculation time, etc.)
33. [X] Add QD, QM, IFORM, FL to equations and DATCOM reading logic
34. [ ] Apply axial coefficient convention
