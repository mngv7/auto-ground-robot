% Credit to Elad Kivelelvitch for original Held-Karp MATLAB implementation
% Modifed by: n11592931 Zackariya Taylor for assignment purposes

function [optimal_waypoints, paths] = optimize_waypoints(waypoints, init_state, map)
    % Starts at init_state, visits all waypoints, does not return to the start.

    waypoints = [init_state; waypoints];
    [num_of_waypoints, ~] = size(waypoints);
    Primes = primes(num_of_waypoints * 10);

    %% Distance matrix
    D = diag(inf(1, num_of_waypoints));
    wp_interval = 2;

    paths = zeros(num_of_waypoints, num_of_waypoints, wp_interval, 2);  

    for i = 1:num_of_waypoints
        for j = i+1:num_of_waypoints
            start = waypoints(i,:);
            goal = waypoints(j,:);
            [D(i,j), path] = astar_search(start, goal, map, wp_interval);
            paths(i,j,1:size(path,1), :) = path;
            D(j,i) = D(i,j);
        end
    end

    %% Initialize data structure
    NumOfDataSets = 1;
    for i = 2:num_of_waypoints
        NumOfDataSets = NumOfDataSets + nchoosek(num_of_waypoints, i);
    end

    Data(NumOfDataSets).S = [];
    Data(NumOfDataSets).l = 0;
    Data(NumOfDataSets).cost = inf;
    Data(NumOfDataSets).Pre = [];
    Data(NumOfDataSets).m = [];
    LookUpTable(NumOfDataSets) = 0;

    Data(1).S = 1;
    Data(1).l = 1;
    Data(1).cost = 0;
    Data(1).Pre = [];
    Data(1).m = [];

    % Initialize single-step paths
    for s = 2:num_of_waypoints
        Data(s).S = [1, s];
        Data(s).l = s;
        Data(s).cost = D(1,s);
        Data(s).Pre = 1;
        Data(s).m = 1;
        LookUpTable(s) = calcLUT(Data(s).S, s, Primes);
    end

    IndexStartPrevStep = 2;
    IndexLastStep = num_of_waypoints;
    CurrentData = IndexLastStep;

    %% Main Dynamic Programming loop
    for s = 3:num_of_waypoints
        TempSets = nchoosek(2:num_of_waypoints, s-1);
        [NumOfSets, ~] = size(TempSets);

        for j = 1:NumOfSets
            for k = 1:s-1
                SminuskSet = [1, TempSets(j, [1:k-1, k+1:end])];
                candidatecost = inf(1, length(SminuskSet));
                indices = zeros(1, length(SminuskSet));

                for mm = 2:length(SminuskSet)
                    LUV = calcLUT(SminuskSet, SminuskSet(mm), Primes);
                    index = find(LUV == LookUpTable(IndexStartPrevStep:IndexLastStep));
                    index = index + IndexStartPrevStep - 1;
                    if ~isempty(index)
                        candidatecost(mm) = Data(index).cost + D(SminuskSet(mm), TempSets(j,k));
                        indices(mm) = index;
                    end
                end

                [mincost_step, indexcost] = min(candidatecost(2:end));
                CurrentData = CurrentData + 1;
                Data(CurrentData).S = [1, TempSets(j,:)];
                Data(CurrentData).l = TempSets(j,k);
                Data(CurrentData).cost = mincost_step;
                Data(CurrentData).Pre = indices(indexcost + 1);
                Data(CurrentData).m = SminuskSet(indexcost + 1);
                LookUpTable(CurrentData) = calcLUT(Data(CurrentData).S, TempSets(j,k), Primes);
            end
        end
        IndexStartPrevStep = IndexLastStep + 1;
        IndexLastStep = CurrentData;
    end

    %% Find minimum cost path (not a cycle)
    candidatecost = inf(1, IndexLastStep - IndexStartPrevStep + 1);
    for i = IndexStartPrevStep:IndexLastStep
        candidatecost(i - IndexStartPrevStep + 1) = Data(i).cost;
    end

    [~, indexcost] = min(candidatecost);
    Temp = Data(IndexStartPrevStep + indexcost - 1);

    %% Reconstruct optimal path
    optimal_waypoints = [Temp.l];
    while ~isempty(Temp.Pre)
        Temp = Data(Temp.Pre);
        optimal_waypoints = [Temp.l, optimal_waypoints];
    end

    % Convert indices to coordinates
    n = length(optimal_waypoints);
    coords = zeros(n,2);
    for i = 1:n
        coords(i,:) = waypoints(optimal_waypoints(i),:);
    end
    optimal_waypoints = coords(2:end,:);
end

function LUT = calcLUT(vec, last, Primes)
    LUT = Primes(last);
    for i = 2:length(vec)
        LUT = LUT * Primes(vec(i));
    end
end