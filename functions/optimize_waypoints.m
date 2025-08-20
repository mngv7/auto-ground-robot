function optimal_waypoints = optimize_waypoints(waypoints, init_state)
    waypoints = [init_state; waypoints];
    
    [num_of_waypoints, ~] = size(waypoints);
    Primes = primes(num_of_waypoints * 10);

    %% Distance matrix
    D = diag(inf(1, num_of_waypoints)); % Inf cost to self
    for i = 1:num_of_waypoints
        for j = i+1:num_of_waypoints
            D(i,j) = norm(waypoints(i,:) - waypoints(j,:));
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

    % Base case: only city 1
    Data(1).S = 1;
    Data(1).l = 1;
    Data(1).cost = 0;
    Data(1).Pre = [];
    Data(1).m = [];

    % Initialize single-step paths
    for s = 2:num_of_waypoints
        Data(s).S = [1, s];
        Data(s).l = s;
        Data(s).cost = D(s,1);
        Data(s).Pre = 1;
        Data(s).m = 1;
        LookUpTable(s) = calcLUT(Data(s).S, s, Primes);
    end

    IndexStartPrevStep = 2;
    IndexLastStep = num_of_waypoints;
    CurrentData = IndexLastStep;

    %% Main Dynamic Programming loop
    for s = 3:num_of_waypoints
        % Generate sets of s-1 waypoints excluding city 1
        TempSets = nchoosek(2:num_of_waypoints, s-1);
        [NumOfSets, ~] = size(TempSets);

        for j = 1:NumOfSets
            for k = 1:s-1
                % Set S minus city k
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

    %% Add distance back to city 1
    candidatecost = inf(1, IndexLastStep - IndexStartPrevStep + 1);
    for i = IndexStartPrevStep:IndexLastStep
        candidatecost(i - IndexStartPrevStep + 1) = Data(i).cost + D(Data(i).l, 1);
    end

    % Find minimum total cost
    [~, indexcost] = min(candidatecost);
    Temp = Data(IndexStartPrevStep + indexcost - 1);

    %% Reconstruct optimal tour
    optimal_waypoints = 1;
    while ~isempty(Temp.Pre)
        optimal_waypoints = [optimal_waypoints, Temp.l];
        Temp = Data(Temp.Pre);
    end
    optimal_waypoints_order = [optimal_waypoints, 1];

    % Preallocate waypoints array.
    n = size(waypoints, 1);
    optimal_waypoints = [n, size(waypoints, 2)];
    
    for i = 1:size(waypoints, 1)
        optimal_waypoints(i, :) = waypoints(optimal_waypoints_order(i), :);
    end
end

%% Helper function to calculate Look-Up Table
function LUT = calcLUT(vec, last, Primes)
    LUT = Primes(last);
    for i = 2:length(vec)
        LUT = LUT * Primes(vec(i));
    end
end