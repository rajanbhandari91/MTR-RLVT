function [state, options,optchanged] = outputfunction(options,state,flag)
a =     state.Generation;
b =     state.Population;
SaveResults = ['Results and Plots/Outputs','/'];
SaveFileName = [SaveResults,'Generation',num2str(a),'.mat'];
save(SaveFileName,'b')
optchanged = false;