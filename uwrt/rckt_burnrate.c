function br = rocketburnrate( RVar, hvm, sw, t)
md = RVar(2);
m = hvm(3);

if ((t < 6)||(m > md))
    br = sw*(-(0.7-0.0167*t));
else
    br =0;
end

%if ((t < 8)||(m > md))
    %br = -sw*(0.3434 - 0.007642*t);
%    br = -(0.66-0.0275*t)*sw;
%else 
%    br = 0;
%end
%end
