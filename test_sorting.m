% 
clear all
aa=[   1525
    1524
    1523
    1520
        1487
        1484
        1480
        1407
        1314
        1303
        12990
        12380];
    l=1;
    k=1;
    zz(1)=aa(1);
for i=(l+1):length(aa)
    if abs(aa(l)-aa(i))<10
    zz(k)=(aa(l)+aa(i))/2;
    else 
        k=k+1;
        zz(k)=aa(i);
    end
    l=l+1;
end
l
zz