function G = ComputeStageCosts(stateSpace, map)
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, map) 
%   computes the stage costs for all states in the state space for all
%   control inputs.
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
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and 
%           apply control input l.

    global GAMMA R P_WIND Nc
    global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
    global NORTH SOUTH EAST WEST HOVER
    global K
    global TERMINAL_STATE_INDEX
    
    
    P = zeros(K,K,5);
    G = 9999*ones(K,5);
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

%% ======= PROBABILITY AND COST ==========

for pack = 0:1                                        % repeat 2 times, for states with and without the pack
    
    for i = pack+1:2:K                                % i indexes half of the total states (also j)
        
        input_feasible = false; 
        
        % --  STATES (m,n,pack) --
        
        crash_prob = zeros(1,5);
        
        %% ===  NORTH  === %%
        
        % input north at (m,n)
        if (stateSpace(i,2) ~= height)                                                 % input north at (m,n)
            if map(stateSpace(i,1),stateSpace(i,2)+1) ~= TREE
                
                input_feasible = true;
          
                % turn back
                P(i,i,1) = P_WIND/4 * prob_no_hit(i);                                     % turn back
                crash_prob(1) = crash_prob(1) + P_WIND/4 *prob_hit(i);
                
                for j = pack+1:2:K
                    
                    if stateSpace(i,2)+1 == stateSpace(j,2)                            % select the entire row above the cell (m,n)
                        
                        % cell n.1
                        if stateSpace(i,1)-1 == stateSpace(j,1)                        % cell n.1
                            P(i,j,1) = P_WIND/4 * prob_no_hit(j);
                            crash_prob(1) = crash_prob(1) + P_WIND/4 *prob_hit(j);
                            
                            % cell n.2
                        elseif stateSpace(i,1) == stateSpace(j,1)                          % cell n.2
                            P(i,j,1) = (1 - P_WIND) * prob_no_hit(j);
                            crash_prob(1) = crash_prob(1) + (1 - P_WIND) *prob_hit(j);
                            
                            % cell n.3
                        elseif stateSpace(i,1)+1 == stateSpace(j,1)                        % cell n.3
                            P(i,j,1) = P_WIND/4 * prob_no_hit(j);
                            crash_prob(1) = crash_prob(1) + P_WIND/4 *prob_hit(j);
                        end
                    end
                    
                    % cell n.4
                    if stateSpace(i,1) == stateSpace(j,1) &&...                       % cell n.4
                            stateSpace(i,2)+2 == stateSpace(j,2)
                        P(i,j,1) = P_WIND/4 * prob_no_hit(j);
                        crash_prob(1) = crash_prob(1) + P_WIND/4 *prob_hit(j);
                    end
                    
                    
                end
                
                if stateSpace(i,1)>1
                    if map(stateSpace(i,1)-1, stateSpace(i,2)+1) == TREE      % cell n.1 TREE
                        crash_prob(1) = crash_prob(1) + P_WIND/4;
                    end
                else
                    crash_prob(1) = crash_prob(1) + P_WIND/4;
                end
                
                if stateSpace(i,1)<width
                    if map(stateSpace(i,1)+1, stateSpace(i,2)+1) == TREE         % cell n.3 TREE
                        crash_prob(1) = crash_prob(1) + P_WIND/4;
                    end
                else
                    crash_prob(1) = crash_prob(1) + P_WIND/4;
                end
                
                if stateSpace(i,2)<height-1
                    if map(stateSpace(i,1), stateSpace(i,2)+2) == TREE        % cell n.4 TREE
                        crash_prob(1) = crash_prob(1) + P_WIND/4;
                    end
                else
                    crash_prob(1) = crash_prob(1) + P_WIND/4;
                end
                
            end
        end
                
       % ===== PICK UP STATE PROBABILITY =====
      
      if pack == 0
        P(i,pick_upIndex+1,1) = P(i,pick_upIndex,1);
        P(i,pick_upIndex,1) = 0;
      end
      
      % ===== DROP OFF STATE PROBABILITY =====
      
      P(drop_offIndex,:,1) = 0;
      P(drop_offIndex,drop_offIndex,1) = 1;
      
      % ===== COST =====
      
      if input_feasible 
          g = ones(K,1);
          G(i,1) = P(i,:,1) * g + crash_prob(1)*Nc;
      else
          G(i,1) = Inf;
      end

      input_feasible = false;
      
        %% ===  SOUTH  === %%
        
        % input south at (m,n)
        if stateSpace(i,2) ~= 1                                                              % input south at (m,n)
            if map(stateSpace(i,1),stateSpace(i,2)-1) ~= TREE
                
                input_feasible = true;

                % turn back
                P(i,i,2) = P_WIND/4 * prob_no_hit(i);                                                   % turn back
                crash_prob(2) = crash_prob(2) + P_WIND/4 *prob_hit(i);
                
                for j = pack+1:2:K
                    if stateSpace(i,2)-1 == stateSpace(j,2)                            % select the entire row under the cell (m,n)
                        
                        % cell n.5
                        if stateSpace(i,1)-1 == stateSpace(j,1)                        % cell n.5
                            P(i,j,2) = P_WIND/4 * prob_no_hit(j);
                            crash_prob(2) = crash_prob(2) + P_WIND/4 *prob_hit(j);
                            
                            % cell n.6
                        elseif stateSpace(i,1) == stateSpace(j,1)                          % cell n.6
                            P(i,j,2) = (1 - P_WIND) * prob_no_hit(j);
                            crash_prob(2) = crash_prob(2) + (1 - P_WIND) *prob_hit(j);
                            
                            % cell n.7
                        elseif stateSpace(i,1)+1 == stateSpace(j,1)                        % cell n.7
                            P(i,j,2) = P_WIND/4 * prob_no_hit(j);
                            crash_prob(2) = crash_prob(2) + P_WIND/4 *prob_hit(j);
                        end
                    end
                    
                    % cell n.8
                    if stateSpace(i,1) == stateSpace(j,1) &&...                       % cell n.8
                            stateSpace(i,2)-2 == stateSpace(j,2)
                        P(i,j,2) = P_WIND/4 * prob_no_hit(j);
                        crash_prob(2) = crash_prob(2) + P_WIND/4 *prob_hit(j);
                        
                    end
                end
                
                if stateSpace(i,1)>1
                    if map(stateSpace(i,1)-1, stateSpace(i,2)-1) == TREE      % cell n.5 TREE
                        crash_prob(2) = crash_prob(2) + P_WIND/4;
                    end
                else
                    crash_prob(2) = crash_prob(2) + P_WIND/4;
                end
                
                if stateSpace(i,1)<width
                    if map(stateSpace(i,1)+1, stateSpace(i,2)-1) == TREE         % cell n.7 TREE
                        crash_prob(2) = crash_prob(2) + P_WIND/4;
                    end
                else
                    crash_prob(2) = crash_prob(2) + P_WIND/4;
                end
                
                if stateSpace(i,2)>2
                    if map(stateSpace(i,1), stateSpace(i,2)-2) == TREE        % cell n.8 TREE
                        crash_prob(2) = crash_prob(2) + P_WIND/4;
                    end
                else
                    crash_prob(2) = crash_prob(2) + P_WIND/4;
                end

            end
        end
        
        
      % ===== PICK UP STATE PROBABILITY =====
      
      if pack == 0
        P(i,pick_upIndex+1,2) = P(i,pick_upIndex,2);
        P(i,pick_upIndex,2) = 0;
      end
      
      % ===== DROP OFF STATE PROBABILITY =====
      
      P(drop_offIndex,:,2) = 0;
      P(drop_offIndex,drop_offIndex,2) = 1;        
      
      % ===== COST =====
      
      if input_feasible 
          g = ones(K,1);
          G(i,2) = P(i,:,2) * g + crash_prob(2)*Nc;
      else
          G(i,2) = Inf;
      end
      
      input_feasible = false;
      
        %% ===  EAST  === %%
        
        % input east at (m,n)
        if stateSpace(i,1) ~= width                                                        % input east at (m,n)
            if map(stateSpace(i,1)+1,stateSpace(i,2)) ~= TREE
                
                input_feasible = true;

                % turn back
                P(i,i,3) = P_WIND/4 * prob_no_hit(i);                                     % turn back
                crash_prob(3) = crash_prob(3) + P_WIND/4 *prob_hit(i);
                
                for j = pack+1:2:K
                    if stateSpace(i,1)+1 == stateSpace(j,1)                            % select the entire right column to the cell (m,n)
                        
                        % cell n.7
                        if stateSpace(i,2)-1 == stateSpace(j,2)                        % cell n.7
                            P(i,j,3) = P_WIND/4 * prob_no_hit(j);
                            crash_prob(3) = crash_prob(3) + P_WIND/4 *prob_hit(j);
                            
                            % cell n.11
                        elseif stateSpace(i,2) == stateSpace(j,2)                          % cell n.11
                            P(i,j,3) = (1 - P_WIND) * prob_no_hit(j);
                            crash_prob(3) = crash_prob(3) + (1 - P_WIND) *prob_hit(j);
                            
                            % cell n.3
                        elseif stateSpace(i,2)+1 == stateSpace(j,2)                        % cell n.3
                            P(i,j,3) = P_WIND/4 * prob_no_hit(j);
                            crash_prob(3) = crash_prob(3) + P_WIND/4 *prob_hit(j);
                        end
                    end
                    
                    % cell n.12
                    if stateSpace(i,2) == stateSpace(j,2) &&...                       % cell n.12
                            stateSpace(i,1)+2 == stateSpace(j,1)
                        P(i,j,3) = P_WIND/4 * prob_no_hit(j);
                        crash_prob(3) = crash_prob(3) + P_WIND/4 *prob_hit(j);
                        
                    end
                end
                
                if stateSpace(i,2)>1
                    if map(stateSpace(i,1)+1, stateSpace(i,2)-1) == TREE      % cell n.7 TREE
                        crash_prob(3) = crash_prob(3) + P_WIND/4;
                    end
                else
                    crash_prob(3) = crash_prob(3) + P_WIND/4;
                end
                
                if stateSpace(i,2)<height
                    if map(stateSpace(i,1)+1, stateSpace(i,2)+1) == TREE         % cell n.3 TREE
                        crash_prob(3) = crash_prob(3) + P_WIND/4;
                    end
                else
                    crash_prob(3) = crash_prob(3) + P_WIND/4;
                end
                
                if stateSpace(i,1)<width-1
                    if map(stateSpace(i,1)+2, stateSpace(i,2)) == TREE        % cell n.12 TREE
                        crash_prob(3) = crash_prob(3) + P_WIND/4;
                    end
                else
                    crash_prob(3) = crash_prob(3) + P_WIND/4;
                end

            end
        end
 
      % ===== PICK UP STATE PROBABILITY =====
      
      if pack == 0
        P(i,pick_upIndex+1,3) = P(i,pick_upIndex,3);
        P(i,pick_upIndex,3) = 0;
      end
      
      % ===== DROP OFF STATE PROBABILITY =====
      
      P(drop_offIndex,:,3) = 0;
      P(drop_offIndex,drop_offIndex,3) = 1;        
      
      % ===== COST =====
      
      if input_feasible 
          g = ones(K,1);
          G(i,3) = P(i,:,3) * g + crash_prob(3)*Nc;
      else
          G(i,3) = Inf;
      end
      
      input_feasible = false;
      
        %% ===  WEST  === %% 
        
        % input west at (m,n)
        if stateSpace(i,1) ~= 1                                                        % input west at (m,n)
            if  map(stateSpace(i,1)-1,stateSpace(i,2)) ~= TREE
                
                input_feasible = true;
                
                % turn back
                P(i,i,4) = P_WIND/4 * prob_no_hit(i);                                     % turn back
                crash_prob(4) = crash_prob(4) + P_WIND/4 *prob_hit(i);
                
                for j = pack+1:2:K
                    if stateSpace(i,1)-1 == stateSpace(j,1)                            % select the entire left column to the cell (m,n)
                        
                        % cell n.5
                        if stateSpace(i,2)-1 == stateSpace(j,2)                        % cell n.5
                            P(i,j,4) = P_WIND/4 * prob_no_hit(j);
                            crash_prob(4) = crash_prob(4) + P_WIND/4 *prob_hit(j);
                            
                            % cell n.9
                        elseif stateSpace(i,2) == stateSpace(j,2)                          % cell n.9
                            P(i,j,4) = (1 - P_WIND) * prob_no_hit(j);
                            crash_prob(4) = crash_prob(4) + (1 - P_WIND) *prob_hit(j);
                            
                            % cell n.1
                        elseif stateSpace(i,2)+1 == stateSpace(j,2)                        % cell n.1
                            P(i,j,4) = P_WIND/4 * prob_no_hit(j);
                            crash_prob(4) = crash_prob(4) + P_WIND/4 *prob_hit(j);
                        end
                    end
                    
                    % cell n.10
                    if stateSpace(i,2) == stateSpace(j,2) &&...                       % cell n.10
                            stateSpace(i,1)-2 == stateSpace(j,1)
                        P(i,j,4) = P_WIND/4 * prob_no_hit(j);
                        crash_prob(4) = crash_prob(4) + P_WIND/4 *prob_hit(j);
                        
                    end
                end
                
                if stateSpace(i,2)>1
                    if map(stateSpace(i,1)-1, stateSpace(i,2)-1) == TREE      % cell n.5 TREE
                        crash_prob(4) = crash_prob(4) + P_WIND/4;
                    end
                else
                    crash_prob(4) = crash_prob(4) + P_WIND/4;
                end
                
                if stateSpace(i,2)<height
                    if map(stateSpace(i,1)-1, stateSpace(i,2)+1) == TREE         % cell n.1 TREE
                        crash_prob(4) = crash_prob(4) + P_WIND/4;
                    end
                else
                    crash_prob(4) = crash_prob(4) + P_WIND/4;
                end
                
                if stateSpace(i,1)>2
                    if map(stateSpace(i,1)-2, stateSpace(i,2)) == TREE        % cell n.10 TREE
                        crash_prob(4) = crash_prob(4) + P_WIND/4;
                    end
                else
                    crash_prob(4) = crash_prob(4) + P_WIND/4;
                end

            end
        end
      
      % ===== PICK UP STATE PROBABILITY =====
      
      if pack == 0
        P(i,pick_upIndex+1,4) = P(i,pick_upIndex,4);
        P(i,pick_upIndex,4) = 0;
      end
      
      % ===== DROP OFF STATE PROBABILITY =====
      
      P(drop_offIndex,:,4) = 0;
      P(drop_offIndex,drop_offIndex,4) = 1;
      
      % ===== COST =====
      
      if input_feasible 
          g = ones(K,1);
          G(i,4) = P(i,:,4) * g + crash_prob(4)*Nc;
      else
          G(i,4) = Inf;
      end
 
      input_feasible = false;
      
        %% ===  HOVER  === %%
        
        % input hover at (m,n)
        
        input_feasible = true;
        
        % stay
        P(i,i,5) = (1 - P_WIND) * prob_no_hit(i);                                                   % stay
        crash_prob(5) = crash_prob(5) + (1 - P_WIND) *prob_hit(i);
                        
        for j = pack+1:2:K
            
            % cell n.2
            if stateSpace(i,2)+1 == stateSpace(j,2) &&...                        % cell n.2
                    stateSpace(i,1) == stateSpace(j,1)
                P(i,j,5) = P_WIND/4 * prob_no_hit(j);
                crash_prob(5) = crash_prob(5) + P_WIND/4 *prob_hit(j);
                
                % cell n.6
            elseif stateSpace(i,2)-1 == stateSpace(j,2)  &&...                        % cell n.6
                    stateSpace(i,1) == stateSpace(j,1)
                P(i,j,5) = P_WIND/4 * prob_no_hit(j);
                crash_prob(5) = crash_prob(5) + P_WIND/4 *prob_hit(j);
                
                % cell n.9
            elseif stateSpace(i,2) == stateSpace(j,2)  &&...                      % cell n.9
                    stateSpace(i,1)-1 == stateSpace(j,1)
                P(i,j,5) = P_WIND/4 * prob_no_hit(j);
                crash_prob(5) = crash_prob(5) + P_WIND/4 *prob_hit(j);
                
                % cell n.11
            elseif stateSpace(i,2) == stateSpace(j,2)  &&...                      % cell n.11
                    stateSpace(i,1)+1 == stateSpace(j,1)
                P(i,j,5) = P_WIND/4 * prob_no_hit(j);
                crash_prob(5) = crash_prob(5) + P_WIND/4 *prob_hit(j);
            end
        end
        
        if stateSpace(i,2)<height
            if map(stateSpace(i,1), stateSpace(i,2)+1) == TREE      % cell n.2 TREE
                crash_prob(5) = crash_prob(5) + P_WIND/4;
            end
        else
            crash_prob(5) = crash_prob(5) + P_WIND/4;
        end
        
        if stateSpace(i,2)>1
            if map(stateSpace(i,1), stateSpace(i,2)-1) == TREE         % cell n.6 TREE
                crash_prob(5) = crash_prob(5) + P_WIND/4;
            end
        else
            crash_prob(5) = crash_prob(5) + P_WIND/4;
        end
        
        if stateSpace(i,1)>1
            if map(stateSpace(i,1)-1, stateSpace(i,2)) == TREE        % cell n.9 TREE
                crash_prob(5) = crash_prob(5) + P_WIND/4;
            end
        else
            crash_prob(5) = crash_prob(5) + P_WIND/4;
        end
        
        if stateSpace(i,1)<width
            if map(stateSpace(i,1)+1, stateSpace(i,2)) == TREE        % cell n.11 TREE
                crash_prob(5) = crash_prob(5) + P_WIND/4;
            end
        else
            crash_prob(5) = crash_prob(5) + P_WIND/4;
        end
        
      % ===== PICK UP STATE PROBABILITY =====
      
      if pack == 0
        P(i,pick_upIndex+1,5) = P(i,pick_upIndex,5);
        P(i,pick_upIndex,5) = 0;
      end
      
      % ===== DROP OFF STATE PROBABILITY =====
      
      P(drop_offIndex,:,5) = 0;
      P(drop_offIndex,drop_offIndex,5) = 1;
      
      % ===== COST =====
      
      if input_feasible 
          g = ones(K,1);
          G(i,5) = P(i,:,5) * g + crash_prob(5)*Nc;
      else
          G(i,5) = Inf;
      end
      
      input_feasible = false;
      
    end

end

G(TERMINAL_STATE_INDEX,:) = 0;

end

