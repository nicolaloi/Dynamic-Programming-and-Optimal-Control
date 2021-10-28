function [ J_opt , u_opt_ind ] = LinearProgramming(P, G)
%LINEARPROGRAMMING Linear Programming
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
%
%   Input arguments:
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER 

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

%% Computing the maximum in the stage cost matrix
new_stage_cost_matrix = zeros(K , 5);

for ii = 1:K
    for jj = 1:5
        if isinf(G(ii , jj))
            new_stage_cost_matrix(ii , jj) = 0;
        else
            new_stage_cost_matrix(ii , jj) = G(ii , jj);
        end
    end
end

%find the maximum in the provious matrix
max_stage_cost_vect = zeros(K,1);
max_stage_cost = 0;

for iii = 1:K
    max_stage_cost_vect(iii) = max(new_stage_cost_matrix(iii , :));
end

max_stage_cost = max(max_stage_cost_vect);
        



%% Computing the matrix of constraints for Linear Programming
%It will be a matrix of KL x K which defines a constraint for each possible
%initial state and for every possible control input


A = zeros(K*5 , K);
indexrow = 1;   %matlab indexes start from one
indexcol = 1;
newinput = 1;

for indexrow = 1:K*5
    
    if newinput==6
        newinput=1;
    end
    
    for indexcol = 1:K
        if (indexcol == ceil(indexrow/5))
            A(indexrow , indexcol) = 1 - P(indexcol, indexcol, newinput);
        else
            A(indexrow , indexcol) = -P(ceil(indexrow/5), indexcol, newinput);
        end
    end
    
    newinput = newinput + 1;
end

%computing the matrix in order not to make the problem unbounded
A_cut = zeros(5*(K-1) , K-1);
A_cut_up = [A(1:5*(TERMINAL_STATE_INDEX-1),1:(TERMINAL_STATE_INDEX-1)) , A(1:5*(TERMINAL_STATE_INDEX-1),(TERMINAL_STATE_INDEX+1):end)];
A_cut_down = [A((5*(TERMINAL_STATE_INDEX)+1):end ,1:(TERMINAL_STATE_INDEX-1)) , A((5*(TERMINAL_STATE_INDEX)+1):end,(TERMINAL_STATE_INDEX+1):end)];
A_cut = [A_cut_up ; A_cut_down];


%% Computing The known terms vector form the stage cost matrix
b = zeros(5*K ,1);
col = 0;
row = 0;
num_rows = 1;

for row = 1:K
    for col = 1:5
        if (isinf(G(row , col)))
            b(num_rows , 1) = 100000 * max_stage_cost;
        else
            b(num_rows , 1) = G(row , col);
        end
        num_rows = num_rows + 1;
    end
end

%computing the vector in order not to make the problem unbounded
b_cut = [b(1:5*(TERMINAL_STATE_INDEX-1) , 1) ; b(5*TERMINAL_STATE_INDEX+1:end , 1)];
%% Solving the SSP Problem using Linear Programming
%using the function linprog

J_opt_no_terminal = zeros(K-1,1);
J_opt = zeros(K,1);

f = -ones(K-1 , 1);
Aeq = [];
beq = [];
lower_bound = zeros(K-1,1);
upper_bound = [];

J_opt_no_terminal = linprog(f , A_cut , b_cut , Aeq , beq , lower_bound , upper_bound);
J_opt_up = zeros(TERMINAL_STATE_INDEX,1);
J_opt_up = [J_opt_no_terminal(1:TERMINAL_STATE_INDEX-1 , 1) ; 0.0];
J_opt_down = [J_opt_no_terminal(TERMINAL_STATE_INDEX:end , 1)];
J_opt = [J_opt_up ; J_opt_down];

%% Calculate the optimal policy

departure_state = 1;
current_input = 1;

u_opt_ind = ones(K , 1);
u_opt_ind_no_terminal = 5*ones(K-1 , 1);

right_hand_side = zeros(K-1 , 5);
expected_stage_cost_matrix_cut = zeros(K-1 , 5);
expected_stage_cost_matrix_cut = [G(1:TERMINAL_STATE_INDEX-1 , 1:end) ; G(TERMINAL_STATE_INDEX+1:end , 1:end)];

probability_matrix_cut = zeros(K-1,K-1,5);
probability_matrix_cut_up = [P(1:TERMINAL_STATE_INDEX-1 , 1:TERMINAL_STATE_INDEX-1 , 1:end) , P(1:TERMINAL_STATE_INDEX-1 , TERMINAL_STATE_INDEX+1:end , 1:end)];
probability_matrix_cut_down = [P(TERMINAL_STATE_INDEX+1:end , 1:TERMINAL_STATE_INDEX-1 , 1:end) , P(TERMINAL_STATE_INDEX+1:end , TERMINAL_STATE_INDEX+1:end , 1:end)];
probability_matrix_cut = [probability_matrix_cut_up ; probability_matrix_cut_down];

summation = zeros(K-1 , 5);

for departure_state = 1:K-1
    for current_input = 1:5
        summation(departure_state , current_input) = probability_matrix_cut(departure_state , 1:end , current_input)*J_opt_no_terminal;
        right_hand_side(departure_state , current_input) = expected_stage_cost_matrix_cut(departure_state , current_input) + summation(departure_state , current_input);
    end
end

count = 1;
minimum = zeros(K-1 , 1);

for count = 1:K-1
    minimum(count , 1) = min(right_hand_side(count, 1:end));
    u_opt_ind_no(count) = find( right_hand_side(count, 1:end)==minimum(count ,1) , 1 );
    
end


u_opt_ind_up = [u_opt_ind_no(1:TERMINAL_STATE_INDEX-1) HOVER];
u_opt_ind_down = [u_opt_ind_no(TERMINAL_STATE_INDEX:end)];
u_opt_ind = [u_opt_ind_up' ; u_opt_ind_down'];

