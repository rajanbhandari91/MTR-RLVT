m2inch = 39.3701;
TailDim.Span_in = Vehicle.Geom.LHTail.Span*2*m2inch;
TailDim.Chord_in = Vehicle.Geom.LHTail.Stn.c(1) * m2inch;
WingDim.Chord_in = Vehicle.Geom.LWing.Stn.c(1) * m2inch;
WingDim.Span_in = Vehicle.Geom.LWing.Span*2*m2inch;

disp('Wing')
disp(WingDim);
disp('Tail')
disp(TailDim);