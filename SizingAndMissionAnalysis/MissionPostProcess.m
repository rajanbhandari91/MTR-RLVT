function Mission = MissionPostProcess(Mission)
% any custom user-specified post-processing to be done


Mission.TripDist_km = Mission.DistTable_km(4,7);




