waypoints = [0    0   0;
             1    1   1;
             2    0   2;
             3    -1  1;
             4    0   0]';

A = zeros(8*4, 8*4);

%Pi(t=0)=wi
b=polyT(8,0,0);
m=0;
for k=1:4
    for i=1:8
        A(k,i+m)=b(i);
    end
    m=m+8;
end
m=0;

%Pi(t=1)=w(i+1)
b=polyT(8,0,1);
for k=5:8
    for i=1:8
        A(k,i+m)=b(i);
    end
    m=m+8;
end


m=0;

for k=9:11
    %P1(k)(t=0)=0
    b=polyT(8,k-8,1);
    for i=1:8
        A(k,i)=b(i);
    end
end

for k=12:14
    %P1(k)(t=0)=0
    b=polyT(8,k-11,1);
    for i=25:32
        A(k,i)=b(i-24);
    end
end

for k=0:5
    for row=(15+3*k):(17+3*k)
        b=polyT(8,k+1,1);
        c=polyT(8,k+1,0);
        for i=(1+8*k):(8+8*k)
            A(row,i)=b(i-8*k);
            A(row,i+8)=-c(i-8*k);
        end
    end    
end


        







function [T] = polyT(n,k,t)
%n is the polynom number of coefficients, k is the requested derivative and
%t is the actual value of t (this can be anything, not just 0 or 1).
T = zeros(n,1);
D = zeros(n,1);
%Init:
for i=1:n
D(i) = i-1;
T(i) = 1;
end
%Derivative:
for j=1:k
    for i=1:n
        T(i) = T(i) * D(i);

        if D(i) > 0
            D(i) = D(i) - 1;
        end
    end
end
%put t value
for i=1:n
T(i) = T(i) * t^D(i);
end
T = T';
end



         
      

