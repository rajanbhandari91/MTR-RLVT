

function SectionalProp = CallXfoil(foil,M,Re)

PropDef = Aero_XFOIL(foil,M,Re);
save('Airfoil\foilName.mat','PropDef');
copyfile('Airfoil\foilName.mat',['Airfoil\',foil,'.mat']);

SectionalProp = PropDef