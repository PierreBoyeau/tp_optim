// Initialization 
  Ti=1; 
  Tf=10; 
  Nx=9; 
  state=[1 : Nx]; 
  xref = 5;
  
function B=get_B(x)
    if x==1 then
        B = [1];
    elseif x==2 then
        B = [0, 1];
    elseif x==Nx then
        B = [-1];
    elseif x==Nx-1 then
        B = [0, -1];
    else
        B = [-1, 0, 1];
    end
endfunction

function proba=p(w)
    proba=0.5;
endfunction

function cost=cost_t(x,u)
    cost=0;
endfunction

  // Initialization of the matrix used to store Bellman values 
  V=ones(Tf,Nx) * %inf; 
  // Initailization of the control history
  U=ones(Tf,Nx) * %inf;
  // Compute Bellman function at final time 
  V(Tf,:)=(state - xref) .^ 2;
  // make a time backward loop 
  for t=Tf - 1 :  -1 :  Ti do 
    // make a loop on possible states 
    for x=state do 
      // make a loop on admissible controls (for state x) to obtain the best possible 
      // expected cost starting from state x in cost_x 
      cost_x=%inf;
      optimal_control=%inf;
      for u=get_B(x) do 
        // make a loop on the random perturbation to compute the expected 
        // cost cost_x_u 
        cost_x_u = 0;
        for w=[-1, 1] do 
          // for a given perturbation compute the cost associated to 
          // control u and state x using the instantaneous cost and the 
          // Bellman function at time t+1 
          cost_x_u_w=(cost_t(x,u) + V(t + 1,x + u + w)) * p(w);
          cost_x_u = cost_x_u + cost_x_u_w;
        end 
        // update cost_x_u with cost_x_u_w 
        // compare the current cost_x_u to cost_x and 
        // update cost_x if cost_x_u is better than the current cost_x 
        cost_x = min(cost_x, cost_x_u);
        if cost_x_u < cost_x then
            cost_x = cost_x_u;
            optimal_control = u;
        end
      end 
      // store cost_x in V(t,x) 
      V(t,x)=cost_x;
      U(t,x)=optimal_control;
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
