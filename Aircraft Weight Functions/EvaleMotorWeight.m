function n_motor_mass = EvaleMotorWeight(n_motor,p2m_aircraft,p2m_motor,WTO_kg)

Pwr_rating_total = WTO_kg*p2m_aircraft;                                     % [kW]
Pwr_rating_n_motor = Pwr_rating_total/n_motor;                              % [kW]
n_motor_mass = Pwr_rating_n_motor/p2m_motor;                                % [kg]
end