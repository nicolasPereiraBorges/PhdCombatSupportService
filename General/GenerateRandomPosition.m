function position = GenerateRandomPosition(xlim, ylim, zlim)

x = GenerateRandomValue(xlim);
y = GenerateRandomValue(ylim);
z = GenerateRandomValue(zlim);

position = Position3D(x,y,z);

end

function value = GenerateRandomValue(lim)

r = rand();
value = (lim(2) - lim(1))* r + lim(1);

end