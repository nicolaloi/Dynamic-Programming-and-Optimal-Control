function stateIndex = ComputeTerminalStateIndex(stateSpace, map)
%ComputeTerminalStateIndex Compute the index of the terminal state in the
%stateSpace matrix
%
%   stateIndex = ComputeTerminalStateIndex(stateSpace, map) 
%   Computes the index of the terminal state in the stateSpace matrix
%   Input arguments:
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the terrain of the estate map. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%       stateIndex:
%           An integer that is the index of the terminal state in the
%           stateSpace matrix

global DROP_OFF

found = false;

for m = 1 : size(map, 1)
    for n = 1 : size(map, 2)
        if map(m, n) == DROP_OFF
            found = true;
            break
        end
    end
    if found
        break
    end
end

stateIndex = find(stateSpace(:,1)== m & stateSpace(:,2) == n & stateSpace(:,3) == 1);
%state = [stateSpace(stateIndex,1) stateSpace(stateIndex,2) stateSpace(stateIndex,3)]
%Correct_if_equal_4 = map(stateSpace(stateIndex,1), stateSpace(stateIndex,2))

end
