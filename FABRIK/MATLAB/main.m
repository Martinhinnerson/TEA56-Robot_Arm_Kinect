function [] = main()

node_positions = [0 0;-4 0;-4 4;0 4];
thetas = [pi/2 pi/2; pi/2 pi/2; 3*pi/4 3*pi/4; 0 0];
t = [5 3];
node_positions = FABRIK(node_positions,t,thetas);

end