function h = road_height(x)
height = 0.1;
starting_pos = 1;

h = height*(tanh(100*(x-starting_pos))+1)/2;
