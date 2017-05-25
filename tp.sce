
// Initialization 
  Tf=ZZZZ; 
  Nx=ZZZZ; 
  state=[1 : Nx]; 
  // Initialization of the matrix used to store Bellman values 
  V=ones(Tf,Nx) * %inf; 
  // Compute Bellman function at final time 
  V(Tf,:)=ZZZZ 
  // make a time backward loop 
  for t=Tf - 1 :  -1 :  Ti do 
    // make a loop on possible states 
    for x=ZZZZ do 
      // make a loop on admissible controls (for state x) to obtain the best possible 
      // expected cost starting from state x in cost_x 
      cost_x=%inf 
      for u=ZZZZ do 
        // make a loop on the random perturbation to compute the expected 
        // cost cost_x_u 
        for w=ZZZZ do 
          // for a given perturbation compute the cost associated to 
          // control u and state x using the instantaneous cost and the 
          // Bellman function at time t+1 
          cost_x_u_w=(cost_t(x,u) + V(t + 1,ZZZZ)) * p(w) 
        end 
        // update cost_x_u with cost_x_u_w 
        // compare the current cost_x_u to cost_x and 
        // update cost_x if cost_x_u is better than the current cost_x 
        // 
      end 
      // store cost_x in V(t,x) 
    end 
  end 
  
  
  
  function cost=simulation_mc(x0,policy,N) 
    cost=0; 
    for i=1 : N do 
      x=x0; 
      for t=1 : TF - 1 do 
        w=W(grand(1,1,"uin",1,2)); 
        x=x + policy(t,x) + w; 
      end 
      cost=cost + (x - xref) ^ 2 
    end 
    cost=cost / N; 
  endfunction 
  
  
  
  
      // Exact computation with the law of W 
    Wa=all_w(TF - 1); 
    cost=0; 
    for i=1 : size(Wa,"r") do 
      x=x0; 
      for t=1 : TF - 1 do 
        x=x + policy(t,x) + Wa(i,t); 
      end 
      cost=cost + (x - xref) ^ 2 
    end 
    cost=cost / size(Wa,"r"); 
  endfunction 
   
  function W=all_w(n) 
    // generated all the possible (W_1,...,W_(TF-1)) 
    if n == 1 then 
      W=[-1;1] 
    else 
      Wn=all_w(n - 1); 
      W=[-1 * ones(size(Wn,"r"),1),Wn;1 * ones(size(Wn,"r"),1),Wn]; 
    end 
  endfunction; 
  
  
  
  function costs=simulation_dp(policy) 
    // evaluation by dynamic programming with fixed policy 
    Vs=ones(TF,length(X)) * %inf; 
    // Bellman function at time TF 
    Vs(TF,:)=(X - xref) .^ 2; 
    // Compute final value functions 
    // Loop backward over time: 
    for t=(TF - 1) :  -1 :  1 do 
      for x=1 : 10 do 
        // loop on noises 
        EV=0; 
        for iw=1 : size(W,"*") do 
          next_state=x + policy(t,x) + W(iw); 
          EV=EV + P(iw) * Vs(t + 1,next_state); 
        end 
        Vs(t,x)=EV; 
      end 
    end 
    costs=Vs(1,:); 
  endfunction 
