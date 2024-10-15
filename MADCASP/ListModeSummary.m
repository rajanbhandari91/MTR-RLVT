clc

figure
hold on

subplot(1,2,1)
hold on
title('Longitudinal')
xlabel('Real')
ylabel('Imaginary')
box('on')
grid('on')


subplot(1,2,2)
hold on
title('Lateral')
xlabel('Real')
ylabel('Imaginary')
box('on')
grid('on')

for i = 1:ncases
    
   Soln(i,:)
   fprintf('Longitudinal \n')
   Modes(i).LongModes
      rgb = interp1([0,250],[0,1,0;1,0,0],Soln.KEAS(i),'linear');

   x1 = real(Modes(i).LongModes.Eigenvalue);
   y1 = imag(Modes(i).LongModes.Eigenvalue);
   
   subplot(1,2,1); hold on;
   scatter(x1,y1,[],rgb,'filled');
   
%    rgb = repmat(rgb,[length(x1),1]);
   
   x2 = real(Modes(i).LatModes.Eigenvalue);
   y2 = imag(Modes(i).LatModes.Eigenvalue);
   
   subplot(1,2,2); hold on;
   scatter(x2,y2,[],rgb,'filled');   
   
%    fprintf('Lateral \n')
%    Modes(i).LatModes
    
    
    
end