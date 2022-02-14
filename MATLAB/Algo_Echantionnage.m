function Fx = Algo_Echantionnage(t,N,G,F, TIME)

    Fx(1,:) = F(1,:);
    Fx(2,:) = F(2,:);
    Fx(1, size(F,2)+1) = TIME(length(TIME));
    Fx(2, size(F,2)+1) = mean(G(t-N+1:t));
    
end