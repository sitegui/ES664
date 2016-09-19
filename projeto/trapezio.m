function [ x, xp, xpp, t ] = trapezio( x0, x1, T, a, dt )
%trapezio Monta um perfil trapezoidal de velocidade
%	[x, xp, xpp, t] = trapezio(x0, x1, T, a, dt)
%	Entradas:
%   - x0: valor inicial
%	- x1: valor final
%	- T: tempo total
%	- a: porcentagem de tempo de aceleração (opcional, padrão 0.1)
%	- dt: tempo de discretização (opcional, padrão T/100)
%	Saídas:
%	- x: valores da variável em cada instante do tempo
%	- xp: primeira derivada
%	- xpp: segunda derivada
%	- t: valores de tempo

if nargin < 4
	a = 0.1;
end

if nargin < 5
	dt = T/100;
end

t = 0:dt:T;
n = numel(t);
t1 = a*T;
n1 = round(t1/dt);
t2 = T-t1;
n2 = round(t2/dt);
xpmax = (x1-x0)/(T*(1-a));
xppmax = xpmax/t1;

xpp = zeros(1, n);
xp = zeros(1, n);
x = zeros(1, n);
i = 1;
for tt=t
	if tt < t1
		xpp(i) = xppmax;
		xp(i) = xppmax*tt;
		x(i) = x0 + xppmax*tt*tt/2;
	elseif tt < t2
		xpp(i) = 0;
		xp(i) = xpmax;
		x(i) = x0 + xpmax*t1/2 + xpmax*(tt-t1);
	else
		xpp(i) = -xppmax;
		tt2 = tt-t2;
		xp(i) = xpmax - xppmax*tt2;
		x(i) = x0 + xpmax*t1/2 + xpmax*(t2-t1) + xpmax*tt2 - xppmax*tt2*tt2/2;
	end
	i = i+1;
end

end