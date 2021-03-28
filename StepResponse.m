w=[0:0.1:1];
for i=1:length(w)
    G=tf(1,[1 2*w(i) 1]);
    hold on
    step(G);
end
hold off