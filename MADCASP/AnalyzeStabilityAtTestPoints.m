clc
% clear


load TrimPointsFull

[ncases,~] = size(TrimPoints);



RunCases = [1:ncases];

% RunCases = [21,22];

LongVarNames = {'u','w','q','x','z','th'};
LongVarRef = {'th'};

LatVarNames = {'v','p','r','y','ph','ps'};
LatVarRef = {'ph'};

for k = 1:length(RunCases)
    
    i = RunCases(k);
    
    Init.X = TrimPoints.X(i,:)';
    Init.U = TrimPoints.Controls(i,:)';
    Init.Ucv = TrimPoints.UTrimVec(i,:);
    Init.FltCon = TrimPoints.FltCon(i);
    
    fprintf('Case %0.0f: Linearizing...',i)
    
    VehTemp = TrimPoints.Vehicle(i);
    
    
    Ref.Velocity = TrimPoints.KTAS(i)*0.51444;
    Ref.RefLongLength = VehTemp.Geom.RWing.MAC;
    Ref.RefLatLength = VehTemp.Geom.RWing.Span*2;
    
    latcon = [];
    longcon = [];
    
    [LinModel] = LinearizeModel(Init,VehTemp,longcon,latcon,Ref);
    fprintf('Analyzing modes...\n')
    
    EVFactor = 2*Ref.Velocity/Ref.RefLongLength;
    [ModesTemp.LongModes] = AnalyzeModes(LinModel.Alongnd,EVFactor,LongVarNames,LongVarRef);
    
    EVFactor = 2*Ref.Velocity/Ref.RefLatLength;
    [ModesTemp.LatModes] = AnalyzeModes(LinModel.Alatnd,EVFactor,LatVarNames,LatVarRef);

    Modes(i,1) = ModesTemp;
end


TrimPoints.Modes(1:max(RunCases)) = Modes;

save('./MADCASP/TrimPointsWithModes.mat','TrimPoints')

% PostprocessModes
% PlotModes