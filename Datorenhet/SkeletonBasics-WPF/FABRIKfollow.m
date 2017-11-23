function [] = FABRIKfollow(varargin)
iter = 1;
%node_positions = [0 0 -0.1061 0.1061 -0.2058 0.2058 -0.3154 0.3154];
if(nargin == 1||nargin == 3)
    iter = varargin{1};
end
endpos1 = [0.5*rand ,-0.1 + 0.6*rand];
for i=1:iter
% startpos = [-5 + (5+5)*rand , 5*rand];
% node_positions = [linspace(0,startpos(1),4);linspace(0,startpos(2),4)]';
endpos2 = [0.5*rand ,-0.2 + 0.6*rand];
array_of_pos = [linspace(endpos1(1),endpos2(1),30);linspace(endpos1(2),endpos2(2),30)]';
endpos1 = endpos2;

for i = 1:length(array_of_pos)-1
FABRIK_2([0 0 -0.1061 0.1061 -0.2058 0.2058],array_of_pos(i,:));
pause(1/60);
end
end