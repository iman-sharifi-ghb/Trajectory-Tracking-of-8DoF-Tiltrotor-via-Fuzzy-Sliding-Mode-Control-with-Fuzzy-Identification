function [] = imanPlot(x1,x2, Title, labelX, labelY)

linewidth = 2;
fontsize = 16;
    
if nargin == 2
    Title = '';
    labelX = '';
    labelY = '';
elseif nargin == 3
    labelX = '';
    labelY = '';
elseif nargin ==4
    labelY = '';
end

plot(x1,x2,'linewidth',linewidth)
grid on
set(gca, 'fontsize', fontsize)
title(Title);
xlabel(labelX)
ylabel(labelY)

end

