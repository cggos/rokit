num1=[1]; den1=[1 1];   %1/s+1的分子和分母多项式
num2=[1]; den2=[1 2];
num3=[1 2]; den3=[1 4 5];
num4=[1]; den4=[1 3];
[nump,denp]=parallel(num2,den2,num3,den3);%两传递函数并联
[nums,dens]=series(num1,den1,nump,denp);%两传递函数串联
[num,den]=feedback(nums,dens,num4,den4,-1);%负反馈连接
printsys(num,den)                   %在屏幕上打印系统