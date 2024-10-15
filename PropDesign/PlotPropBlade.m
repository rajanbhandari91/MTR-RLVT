
close all

% Expects the following to be in the workspace
% QMIL: parsed QMIL output



% find twist at 0.75R
beta75R = interp1(QMIL.r_R,QMIL.beta,0.75,'linear');

% subtract this value from pitches of all stations
QMIL.relbeta = QMIL.beta - beta75R;

% 0.75R stations remains "flat". Adjust chords of all other stations
QMIL.c_R_proj = QMIL.c_R.*cosd(QMIL.relbeta);


% hub
th = [0:0.1:360]';
xh = (PropDef.r_h/PropDef.r_t) * cosd(th);
yh = (PropDef.r_h/PropDef.r_t) * sind(th);


ymax = 1.2 * max(yh);
ymin = 1.2 * min(yh);

figure


set(gca,'FontSize',18) 
ChordMin = -0.15;
ChordMax = +0.15;
YRange = ymax - ymin;
axis square
xlim([0 1])
ylabel('Blade pitch (deg)','FontWeight', 'bold')
grid('on')
hold on
plot(QMIL.r_R,QMIL.beta,'b','linewidth',2)
ax1 = gca; % current axes
ax1.XColor = 'b';
ax1.YColor = 'b';
ax1.FontWeight = 'bold';
ax1_pos = ax1.Position; % position of first axes
ax1.PlotBoxAspectRatio = [1 YRange 0.1];

ax2 = axes('Position',ax1_pos,...
    'XAxisLocation','bottom',...
    'YAxisLocation','right',...
    'Color','none',...
    'FontWeight','bold');
axis equal
set(gca,'FontSize',18) 

xlim([0 1])
ylim([ymin ymax])
xlabel('Normalized radius (r/R)','FontWeight', 'bold')

ylabel('Normalized chord (c/R)','FontWeight', 'bold')
hold on
plot(QMIL.r_R,QMIL.c_R/2,'k--','linewidth',2,'Parent',ax2)
plot(QMIL.r_R,QMIL.c_R_proj/2,'linewidth',2,'Parent',ax2,'Color','k')

plot(QMIL.r_R,-QMIL.c_R/2,'k--','linewidth',2,'Parent',ax2)
plot(QMIL.r_R,-QMIL.c_R_proj/2,'linewidth',2,'Parent',ax2,'Color','k')

plot(xh,yh)

legend('Chord (untwisted)','Planform (normal to 0.75R chord)','location','southwest')

if exist('TitleString','var')
    title(TitleString)
end
