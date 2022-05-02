% Matlab function to calculate the CoG 

function cg = calculate_cg(beta,M,m,l,h)
    cg(1) = -(m/(M+4*m))*l*(sin(beta(1)) - sin(beta(2)));  
    cg(2) = -(m/(M+4*m))*l*(cos(beta(1)) + cos(beta(2)));
    cg(3) = 2*(m/(M+4*m))*h;
end