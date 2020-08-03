%% The OPEN and CLOSED lists are going to be cell arrays, and each node is going to be separate structs.

clc
clear
%% Reading "nodes.csv" and "edges.csv" files:
nodes = readmatrix('nodes.csv');
edges = readmatrix('edges.csv');

%% Creating two variables for the two nodes of each edge:
edgeStart = edges(:,2);
edgeEnd = edges(:,1);
%% Creating a variable which represents the cost of each edge:
edgeCost = edges(:,3);
%% Creating a G matrix which consists of the cost of going from one node to the next, Inf means the nodes are not connected:
G = Inf(length(nodes), 'double');

for i = 1 : length(nodes)
    
    for j = 1 : length(nodes)
        
        if(ismember(i, edgeStart) && ismember(j, edgeEnd))
            
            start = find(edgeStart == i);
            endd = find(edgeEnd == j);
            index = intersect(start, endd);
            
            if ~isempty(index)

                G(i, j) = edgeCost(index);
            end
        end
        
    end
end
G(length(G), length(G)) = 0;

%% Assigning the heuristic values and node number of each node:

heuristics = nodes(:,4);
for i = 1 : length(nodes)
    node(i).h = heuristics(i);
    node(i).order = i;
end

%% Initializing the f (totalCost) and g (pastCost) values of each node:
node(1).g = 0;
node(1).f = node(1).g + node(1). h;
for i = 2 : length(nodes)
    node(i).g = Inf;
    node(i).f = node(i).g + node(i). h;
end

%% Setting up the start and goal nodes:
startNode = nodes(1);
goalNode = nodes(length(nodes));

%% Finding the successor(s)/adjacent(s) of each node from matrix G:
for i = 1 : length(G)
    for j = 1 : length(G)
        if G(i, j) ~= 0 && G(i, j) ~= Inf
            node(i).successor(j) = j;
        end
    end
    node(i).successor = node(i).successor(node(i).successor ~= 0);
end

%% Initializing the OPEN and CLOSED lists:

OPEN = {};
CLOSED = {};

%% Assigning the first node the OPEN list:
OPEN{length(OPEN) + 1} = node(1);

%% Starting the A* Search Algorithm:

while(~isempty(OPEN))
    nodeCurrent = OPEN{1};
    
    if (node(nodeCurrent.order).order == goalNode)
        disp('The search is completed, path is:')
        path(1) = goalNode;
        k = 2;
        while (1)
            path(k) = node(path(k-1)).parent;
            
            if path(k) == startNode
                disp(flip(path))
                writematrix(flip(path), 'path.csv')
                return
            end
            
            k = k + 1;
        end
        
    end
    
    for i = 1 : length(OPEN)
        openListOrder(i) = OPEN{i}.order;
    end
    
    for i = 1 : length(CLOSED)
        closedListOrder = CLOSED{i}.order;
    end
    
    for i = 1 : length(node(nodeCurrent.order).successor)        
        successorCurrentCost = node(nodeCurrent.order).g + G(nodeCurrent.order, nodeCurrent.successor(i));
        
        if length(OPEN) >= 1 && ismember(nodeCurrent.successor(i), openListOrder)
            if node(nodeCurrent.successor(i)).g <= successorCurrentCost
               continue 
            end
        elseif length(CLOSED) >= 1 && ismember(node(nodeCurrent.order).successor(i), closedListOrder)
            if node(nodeCurrent.successor(i)).g <= successorCurrentCost
                continue
                
            end
                CLOSED(length(CLOSED)) = [];
                OPEN{length(OPEN) + 1} = node(nodeCurrent.successor(i));
        else
            node(nodeCurrent.successor(i)).h = node(nodeCurrent.successor(i)).h;
        end
        node(nodeCurrent.successor(i)).g = successorCurrentCost;
        node(nodeCurrent.successor(i)).f = node(nodeCurrent.successor(i)).g + node(nodeCurrent.successor(i)).h;
        node(nodeCurrent.successor(i)).parent = nodeCurrent.order;
        OPEN{length(OPEN) + 1} = node(nodeCurrent.successor(i));
    end
    
    for i = 1 : length(OPEN)
        openListOrder(i) = OPEN{i}.order;
    end
    removeIndex = find(openListOrder == nodeCurrent.order);
    OPEN(removeIndex) = [];
    CLOSED{length(CLOSED) + 1} = nodeCurrent;
    

    
    [~,I] = sort(cellfun(@(s) getfield(s,"f"), OPEN));
    OPEN = OPEN(I);
end
disp('The search is completed, no path was found.')