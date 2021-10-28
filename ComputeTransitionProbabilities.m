function P = ComputeTransitionProbabilities(stateSpace, map)
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, map) 
%   computes the transition probabilities between all states in the state 
%   space for all control inputs.
%
%   Input arguments:
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX

P = zeros(K,K,5);
width = size(map,1);
height = size(map,2);

%% ======= INFO ========

%        |       |       |       |
%        |       |   4   |       |  
%        |       |       |       |
% ---------------------------------------
%        |       |       |       |
%        |   1   |   2   |   3   |
%        |       |       |       |
% ---------------------------------------
%        |       |       |       |
%    10  |   9   |(m,n,x)|   11  |   12 
%        |       |       |       |
% ---------------------------------------
%        |       |       |       |
%        |   5   |   6   |   7   |
%        |       |       |       |
% ---------------------------------------
%        |       |       |       |
%        |       |   8   |       |   
%        |       |       |       |

% NORTH 1
% SOUTH 2
% EAST 3
% WEST 4
% HOVER 5

%% ======= FIND INDEXES ==========

shooters = [];
for m = 1 : size(map, 1)
    for n = 1 : size(map, 2)
        if map(m, n) == SHOOTER
            shooters = [shooters;
                          m, n];
        elseif map(m, n) == BASE
            base = [m,n];
        elseif map(m, n) == PICK_UP
            pick_up = [m,n];
        elseif map(m, n) == DROP_OFF
            drop_off = [m,n];
        end
    end
end

base_no_packIndex = find(stateSpace(:,1)== base(1) & stateSpace(:,2) == base(2) & stateSpace(:,3) == 0);
base_packIndex = base_no_packIndex + 1;
pick_upIndex = find(stateSpace(:,1)== pick_up(1) & stateSpace(:,2) == pick_up(2) & stateSpace(:,3) == 0);
drop_offIndex = find(stateSpace(:,1)== drop_off(1) & stateSpace(:,2) == drop_off(2) & stateSpace(:,3) == 1);
%Correct_if_equal_5 = map(stateSpace(base_no_packIndex,1),stateSpace(base_no_packIndex,2))
%Correct_if_equal_5 = map(stateSpace(base_packIndex,1),stateSpace(base_packIndex,2))
%Correct_if_equal_3 = map(stateSpace(pick_upIndex,1),stateSpace(pick_upIndex,2))
%Correct_if_equal_4 = map(stateSpace(drop_offIndex,1),stateSpace(drop_offIndex,2))

% for k =1:size(shooters,1)
%     shootersIndex = find(stateSpace(:,1) == shooters(k,1) & stateSpace(:,2) == shooters(k,2) & stateSpace(:,3) == 0);
%     Correct_if_equal_2 = [map(stateSpace(shootersIndex(1),1),stateSpace(shootersIndex(1),2))]
% end

%% ======= HIT PROBABILITY ==========

prob_hit = zeros(K,1);
prob_no_hit = zeros(K,1);

for i = 1:2:K

    distance = zeros(size(shooters,1),1);
    prob_hit_vector = ones(size(shooters,1),1);
    
    for n = 1:size(shooters,1)
        distance(n) = norm([stateSpace(i,1), stateSpace(i,2)]-[shooters(n,1), shooters(n,2)],1);

        if 0<=distance(n) && distance(n)<=R
            prob_hit_vector(n) = GAMMA/(distance(n) + 1);
        else
            prob_hit_vector(n) = 0;
        end
        
        prob_hit(i) = prob_hit(i) + (1 - prob_hit(i))*prob_hit_vector(n);
        prob_hit(i+1) = prob_hit(i);
        
    end
    
    prob_no_hit(i) = 1 - prob_hit(i);
    prob_no_hit(i+1) = 1 - prob_hit(i+1);
end

%% ======= TRANSITION PROBABILITY ==========

for pack = 0:1                                        % repeat 2 times, for states with and without the pack
    
    for i = pack+1:2:K                                % i indexes half of the total states (also j)
        
        % --  STATES (m,n,pack) --
        
        %% ===  NORTH  === %%
        
        % input north at (m,n)
        if (stateSpace(i,2) ~= height)                                                 % input north at (m,n)
            if map(stateSpace(i,1),stateSpace(i,2)+1) ~= TREE
                
                crash_prob = 0;
                
                % turn back
                P(i,i,1) = P_WIND/4 * prob_no_hit(i);                                     % turn back
                crash_prob = crash_prob + P_WIND/4 *prob_hit(i);
                
                for j = pack+1:2:K
                    
                    if stateSpace(i,2)+1 == stateSpace(j,2)                            % select the entire row above the cell (m,n)
                        
                        % cell n.1
                        if stateSpace(i,1)-1 == stateSpace(j,1)                        % cell n.1
                            P(i,j,1) = P_WIND/4 * prob_no_hit(j);
                            crash_prob = crash_prob + P_WIND/4 *prob_hit(j);
                            
                            % cell n.2
                        elseif stateSpace(i,1) == stateSpace(j,1)                          % cell n.2
                            P(i,j,1) = (1 - P_WIND) * prob_no_hit(j);
                            crash_prob = crash_prob + (1 - P_WIND) *prob_hit(j);
                            
                            % cell n.3
                        elseif stateSpace(i,1)+1 == stateSpace(j,1)                        % cell n.3
                            P(i,j,1) = P_WIND/4 * prob_no_hit(j);
                            crash_prob = crash_prob + P_WIND/4 *prob_hit(j);
                        end
                    end
                    
                    % cell n.4
                    if stateSpace(i,1) == stateSpace(j,1) &&...                       % cell n.4
                            stateSpace(i,2)+2 == stateSpace(j,2)
                        P(i,j,1) = P_WIND/4 * prob_no_hit(j);
                        crash_prob = crash_prob + P_WIND/4 *prob_hit(j);
                    end
                    
                    
                end
                
                if stateSpace(i,1)>1
                    if map(stateSpace(i,1)-1, stateSpace(i,2)+1) == TREE      % cell n.1 TREE
                        crash_prob = crash_prob + P_WIND/4;
                    end
                else
                    crash_prob = crash_prob + P_WIND/4;
                end
                
                if stateSpace(i,1)<width
                    if map(stateSpace(i,1)+1, stateSpace(i,2)+1) == TREE         % cell n.3 TREE
                        crash_prob = crash_prob + P_WIND/4;
                    end
                else
                    crash_prob = crash_prob + P_WIND/4;
                end
                
                if stateSpace(i,2)<height-1
                    if map(stateSpace(i,1), stateSpace(i,2)+2) == TREE        % cell n.4 TREE
                        crash_prob = crash_prob + P_WIND/4;
                    end
                else
                    crash_prob = crash_prob + P_WIND/4;
                end
                
                P(i,base_no_packIndex,1) = P(i,base_no_packIndex,1) + crash_prob;
                
            end
        end
        
        %% ===  SOUTH  === %%
        
        % input south at (m,n)
        if stateSpace(i,2) ~= 1                                                              % input south at (m,n)
            if map(stateSpace(i,1),stateSpace(i,2)-1) ~= TREE
                
                crash_prob = 0;
                
                % turn back
                P(i,i,2) = P_WIND/4 * prob_no_hit(i);                                                   % turn back
                crash_prob = crash_prob + P_WIND/4 *prob_hit(i);
                
                for j = pack+1:2:K
                    if stateSpace(i,2)-1 == stateSpace(j,2)                            % select the entire row under the cell (m,n)
                        
                        % cell n.5
                        if stateSpace(i,1)-1 == stateSpace(j,1)                        % cell n.5
                            P(i,j,2) = P_WIND/4 * prob_no_hit(j);
                            crash_prob = crash_prob + P_WIND/4 *prob_hit(j);
                            
                            % cell n.6
                        elseif stateSpace(i,1) == stateSpace(j,1)                          % cell n.6
                            P(i,j,2) = (1 - P_WIND) * prob_no_hit(j);
                            crash_prob = crash_prob + (1 - P_WIND) *prob_hit(j);
                            
                            % cell n.7
                        elseif stateSpace(i,1)+1 == stateSpace(j,1)                        % cell n.7
                            P(i,j,2) = P_WIND/4 * prob_no_hit(j);
                            crash_prob = crash_prob + P_WIND/4 *prob_hit(j);
                        end
                    end
                    
                    % cell n.8
                    if stateSpace(i,1) == stateSpace(j,1) &&...                       % cell n.8
                            stateSpace(i,2)-2 == stateSpace(j,2)
                        P(i,j,2) = P_WIND/4 * prob_no_hit(j);
                        crash_prob = crash_prob + P_WIND/4 *prob_hit(j);
                        
                    end
                end
                
                if stateSpace(i,1)>1
                    if map(stateSpace(i,1)-1, stateSpace(i,2)-1) == TREE      % cell n.5 TREE
                        crash_prob = crash_prob + P_WIND/4;
                    end
                else
                    crash_prob = crash_prob + P_WIND/4;
                end
                
                if stateSpace(i,1)<width
                    if map(stateSpace(i,1)+1, stateSpace(i,2)-1) == TREE         % cell n.7 TREE
                        crash_prob = crash_prob + P_WIND/4;
                    end
                else
                    crash_prob = crash_prob + P_WIND/4;
                end
                
                if stateSpace(i,2)>2
                    if map(stateSpace(i,1), stateSpace(i,2)-2) == TREE        % cell n.8 TREE
                        crash_prob = crash_prob + P_WIND/4;
                    end
                else
                    crash_prob = crash_prob + P_WIND/4;
                end
                
                P(i,base_no_packIndex,2) = P(i,base_no_packIndex,2) + crash_prob;
                
            end
        end
        
        %% ===  EAST  === %%
        
        % input east at (m,n)
        if stateSpace(i,1) ~= width                                                        % input east at (m,n)
            if map(stateSpace(i,1)+1,stateSpace(i,2)) ~= TREE
                
                crash_prob = 0;
                
                % turn back
                P(i,i,3) = P_WIND/4 * prob_no_hit(i);                                     % turn back
                crash_prob = crash_prob + P_WIND/4 *prob_hit(i);
                
                for j = pack+1:2:K
                    if stateSpace(i,1)+1 == stateSpace(j,1)                            % select the entire right column to the cell (m,n)
                        
                        % cell n.7
                        if stateSpace(i,2)-1 == stateSpace(j,2)                        % cell n.7
                            P(i,j,3) = P_WIND/4 * prob_no_hit(j);
                            crash_prob = crash_prob + P_WIND/4 *prob_hit(j);
                            
                            % cell n.11
                        elseif stateSpace(i,2) == stateSpace(j,2)                          % cell n.11
                            P(i,j,3) = (1 - P_WIND) * prob_no_hit(j);
                            crash_prob = crash_prob + (1 - P_WIND) *prob_hit(j);
                            
                            % cell n.3
                        elseif stateSpace(i,2)+1 == stateSpace(j,2)                        % cell n.3
                            P(i,j,3) = P_WIND/4 * prob_no_hit(j);
                            crash_prob = crash_prob + P_WIND/4 *prob_hit(j);
                        end
                    end
                    
                    % cell n.12
                    if stateSpace(i,2) == stateSpace(j,2) &&...                       % cell n.12
                            stateSpace(i,1)+2 == stateSpace(j,1)
                        P(i,j,3) = P_WIND/4 * prob_no_hit(j);
                        crash_prob = crash_prob + P_WIND/4 *prob_hit(j);
                        
                    end
                end
                
                if stateSpace(i,2)>1
                    if map(stateSpace(i,1)+1, stateSpace(i,2)-1) == TREE      % cell n.7 TREE
                        crash_prob = crash_prob + P_WIND/4;
                    end
                else
                    crash_prob = crash_prob + P_WIND/4;
                end
                
                if stateSpace(i,2)<height
                    if map(stateSpace(i,1)+1, stateSpace(i,2)+1) == TREE         % cell n.3 TREE
                        crash_prob = crash_prob + P_WIND/4;
                    end
                else
                    crash_prob = crash_prob + P_WIND/4;
                end
                
                if stateSpace(i,1)<width-1
                    if map(stateSpace(i,1)+2, stateSpace(i,2)) == TREE        % cell n.12 TREE
                        crash_prob = crash_prob + P_WIND/4;
                    end
                else
                    crash_prob = crash_prob + P_WIND/4;
                end
                
                P(i,base_no_packIndex,3) = P(i,base_no_packIndex,3) + crash_prob;
                
            end
        end
        
        %% ===  WEST  === %%
        
        % input west at (m,n)
        if stateSpace(i,1) ~= 1                                                        % input west at (m,n)
            if  map(stateSpace(i,1)-1,stateSpace(i,2)) ~= TREE
                
                crash_prob = 0;
                
                % turn back
                P(i,i,4) = P_WIND/4 * prob_no_hit(i);                                     % turn back
                crash_prob = crash_prob + P_WIND/4 *prob_hit(i);
                
                for j = pack+1:2:K
                    if stateSpace(i,1)-1 == stateSpace(j,1)                            % select the entire left column to the cell (m,n)
                        
                        % cell n.5
                        if stateSpace(i,2)-1 == stateSpace(j,2)                        % cell n.5
                            P(i,j,4) = P_WIND/4 * prob_no_hit(j);
                            crash_prob = crash_prob + P_WIND/4 *prob_hit(j);
                            
                            % cell n.9
                        elseif stateSpace(i,2) == stateSpace(j,2)                          % cell n.9
                            P(i,j,4) = (1 - P_WIND) * prob_no_hit(j);
                            crash_prob = crash_prob + (1 - P_WIND) *prob_hit(j);
                            
                            % cell n.1
                        elseif stateSpace(i,2)+1 == stateSpace(j,2)                        % cell n.1
                            P(i,j,4) = P_WIND/4 * prob_no_hit(j);
                            crash_prob = crash_prob + P_WIND/4 *prob_hit(j);
                        end
                    end
                    
                    % cell n.10
                    if stateSpace(i,2) == stateSpace(j,2) &&...                       % cell n.10
                            stateSpace(i,1)-2 == stateSpace(j,1)
                        P(i,j,4) = P_WIND/4 * prob_no_hit(j);
                        crash_prob = crash_prob + P_WIND/4 *prob_hit(j);
                        
                    end
                end
                
                if stateSpace(i,2)>1
                    if map(stateSpace(i,1)-1, stateSpace(i,2)-1) == TREE      % cell n.5 TREE
                        crash_prob = crash_prob + P_WIND/4;
                    end
                else
                    crash_prob = crash_prob + P_WIND/4;
                end
                
                if stateSpace(i,2)<height
                    if map(stateSpace(i,1)-1, stateSpace(i,2)+1) == TREE         % cell n.1 TREE
                        crash_prob = crash_prob + P_WIND/4;
                    end
                else
                    crash_prob = crash_prob + P_WIND/4;
                end
                
                if stateSpace(i,1)>2
                    if map(stateSpace(i,1)-2, stateSpace(i,2)) == TREE        % cell n.10 TREE
                        crash_prob = crash_prob + P_WIND/4;
                    end
                else
                    crash_prob = crash_prob + P_WIND/4;
                end
                
                P(i,base_no_packIndex,4) = P(i,base_no_packIndex,4) + crash_prob;
                
            end
        end
        
        %% ===  HOVER  === %%
        
        % input hover at (m,n)
        
        crash_prob = 0;
        
        % stay
        P(i,i,5) = (1 - P_WIND) * prob_no_hit(i);                                                   % stay
        crash_prob = crash_prob + (1 - P_WIND) *prob_hit(i);
        
        for j = pack+1:2:K
            
            % cell n.2
            if stateSpace(i,2)+1 == stateSpace(j,2) &&...                        % cell n.2
                    stateSpace(i,1) == stateSpace(j,1)
                P(i,j,5) = P_WIND/4 * prob_no_hit(j);
                crash_prob = crash_prob + P_WIND/4 *prob_hit(j);
                
                % cell n.6
            elseif stateSpace(i,2)-1 == stateSpace(j,2)  &&...                        % cell n.6
                    stateSpace(i,1) == stateSpace(j,1)
                P(i,j,5) = P_WIND/4 * prob_no_hit(j);
                crash_prob = crash_prob + P_WIND/4 *prob_hit(j);
                
                % cell n.9
            elseif stateSpace(i,2) == stateSpace(j,2)  &&...                      % cell n.9
                    stateSpace(i,1)-1 == stateSpace(j,1)
                P(i,j,5) = P_WIND/4 * prob_no_hit(j);
                crash_prob = crash_prob + P_WIND/4 *prob_hit(j);
                
                % cell n.11
            elseif stateSpace(i,2) == stateSpace(j,2)  &&...                      % cell n.11
                    stateSpace(i,1)+1 == stateSpace(j,1)
                P(i,j,5) = P_WIND/4 * prob_no_hit(j);
                crash_prob = crash_prob + P_WIND/4 *prob_hit(j);
            end
        end
        
        if stateSpace(i,2)<height
            if map(stateSpace(i,1), stateSpace(i,2)+1) == TREE      % cell n.2 TREE
                crash_prob = crash_prob + P_WIND/4;
            end
        else
            crash_prob = crash_prob + P_WIND/4;
        end
        
        if stateSpace(i,2)>1
            if map(stateSpace(i,1), stateSpace(i,2)-1) == TREE         % cell n.6 TREE
                crash_prob = crash_prob + P_WIND/4;
            end
        else
            crash_prob = crash_prob + P_WIND/4;
        end
        
        if stateSpace(i,1)>1
            if map(stateSpace(i,1)-1, stateSpace(i,2)) == TREE        % cell n.9 TREE
                crash_prob = crash_prob + P_WIND/4;
            end
        else
            crash_prob = crash_prob + P_WIND/4;
        end
        
        if stateSpace(i,1)<width
            if map(stateSpace(i,1)+1, stateSpace(i,2)) == TREE        % cell n.11 TREE
                crash_prob = crash_prob + P_WIND/4;
            end
        else
            crash_prob = crash_prob + P_WIND/4;
        end
        
        P(i,base_no_packIndex,5) = P(i,base_no_packIndex,5) + crash_prob;
        
      %% ===== PICK UP STATE PROBABILITY =====
      
      if pack == 0
        P(i,pick_upIndex+1,:) = P(i,pick_upIndex,:);
        P(i,pick_upIndex,:) = 0;
      end
      
      %% ===== DROP OFF STATE PROBABILITY =====
      
      P(drop_offIndex,:,:) = 0;
      P(drop_offIndex,drop_offIndex,:) = 1;
    end

end

end

