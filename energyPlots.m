function energyPlots()
  % Just some plots of functions to try
  tt = linspace(0, 900, 300);
  plot(tt, -exp(-tt/150) + 2*cos(tt/60))
end
