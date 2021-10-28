function [ J_opt, u_opt_ind ] = PolicyIteration(P, G)
%POLICYITERATION Policy iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%       P:
%           A (k x k x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (k x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%       J_opt:
%       	A (k x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (k x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

%% policy iteration initialization

u_old = zeros(K,1); 

% initialization to be done accurately taking into account only possible
% inputs (maybe checking from G matrix cell not INF).
%[~,u_old(i)] = min (G(i,:)); 
% Putting all hover only if there is wind 
% PROPER POLICY
for i = 1:K
    u_old(i)=5;
end
u_old(TERMINAL_STATE_INDEX)= 5;

A = zeros(K,K);   % to write a linear system to be solved
B = zeros(K,1);   % NOT consider terminal state in order to avoid rank lost

for i = 1 : K   
    for j = 1:K
        A(i,j) = -P(i,j,u_old(i));
        if (i==j)
            A(i,j)= -P(i,j,u_old(i)) + 1;   
        end
    end
end

for i = 1:K 
    B(i)= G(i,u_old(i));
end


A(TERMINAL_STATE_INDEX,:) = []; % to remove the terminal state effect
A(:,TERMINAL_STATE_INDEX) = [];
B(TERMINAL_STATE_INDEX) = []; 

% Check for bad conditioned matrix. Matlab problem with values near zero.
% This can happen when the map is too big.
% Matlab thinks that the probability matrix is not full ranked.
% Most of the time the policy evaluation will find anyway the correct
% solution.
if rank(A)< length(A)  
    disp('WARNING ::: Rank error in policy iteration system ::: WARNING')
    fprintf('Matrix dimension = %d, while rank = %d, \n', length(A), rank(A))
    %pause
end

warning('off','all')
dA = decomposition(A);

J_old_temp = dA\B;    %linsolve can't be used with sparse matrices
J_old = zeros(K,1);
J_old = [J_old_temp(1:(TERMINAL_STATE_INDEX-1));0;J_old_temp(TERMINAL_STATE_INDEX:end)];



% try to find a solution if the result is a negative cost
for i = 1: length(J_old)
    if (J_old(i) < 0) || isnan(J_old(i)) 
        disp('The costs are negative after initialization, trying to fix ...')
        %pause
        for k = 1: length(J_old)
            J_old(k) = abs(J_old(k));
        end
        
        break
    end
end


%% iteration
it = 0;
cond = 1;
u_new = zeros(K,1);
J_new = zeros(K,1);
J_vett = zeros(1,5);  % to collect the 5 possibilities 
temp_cost = 0; 
stamp1 = 1;
stamp2 = 1;

while (cond)
    
    for i = 1:K  % starting state
        for j = 1:5 % input
            temp_cost = 0 ;
            for k = 1:K % to find cost of previous states
                temp_cost = temp_cost + P(i,k,j)*J_old(k); 
            end
            J_vett(j)= G(i,j) + temp_cost;
        end
        [J_new(i), u_new(i)] = min(J_vett); 
    end
    
    u_new(TERMINAL_STATE_INDEX)= 5;
    temp = 0;
    cond = 1;
    for u = 1:K
        temp = temp + abs(u_old(u)- u_new(u));
    end
    cond = cond*temp;
    
    % security check
%     if it>100
%         for u = 1:K
%             if abs(u_old(u)- u_new(u))>0
%                 fprintf('Position : %d\n',u)
%                 fprintf('Old = %d, New = %d\n',u_old(u), u_new(u))
%             end
%         end
%     end
    
    u_old = u_new; 
    
    % recompute costs for new inputs
    for i = 1 : K   
        for j = 1:K
            A(i,j) = -P(i,j,u_old(i));
            if (i==j)
                A(i,j)= -P(i,j,u_old(i)) + 1;   
            end
        end
    end

    for i = 1:K 
        B(i)= G(i,u_old(i));
    end


    A(TERMINAL_STATE_INDEX,:) = []; % to remove the terminal state effect
    A(:,TERMINAL_STATE_INDEX) = [];
    B(TERMINAL_STATE_INDEX) = []; 

    dA = decomposition(A);

    J_old_temp = dA\B;    
    J_old = [J_old_temp(1:(TERMINAL_STATE_INDEX-1));0;J_old_temp(TERMINAL_STATE_INDEX:end)];
    
    % try to find a solution if the result is a negative cost
    for i = 1: length(J_old)
        if (J_old(i) < 0) || isnan(J_old(i))
            if stamp1
                disp('The costs are negative during iterations, trying to fix ...')
                stamp1 = 0;
            end
            if J_old(i) < 0
                for k = 1: length(J_old)
                J_old(k) = abs(J_old(k));
                end
            else
                if stamp2
                    disp('Linear solver failed, BROKEN ...')
                    stamp2 = 0;
                end
                for k = 1: length(J_old)
                J_old(k) = 1e15;
                J_old(TERMINAL_STATE_INDEX) = 0;
                end
            end
            
            break
        end
    end

    
    it = it + 1;
    if it>150
        break
    end
end

u_new(TERMINAL_STATE_INDEX)= 5; % using min is automatically selected 1, must solve here
if it == 151
    fprintf('Finished\n\n');
else
    fprintf('Finished after %d iterations\n\n', it);
end

warning('on','all')
J_opt = J_old;
u_opt_ind = u_new; 

end
