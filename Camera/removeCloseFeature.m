function temp = removeCloseFeature(feature, distance)
    % A stupid way to remove feature that are too close to each other
    n = 1;
    temp = [];
    temp(:, 1) = feature(:, 1);

    for i = 1:1:length(feature)
        count = 0;
        for j = i + 1 : 1 : length(feature)
            if norm(feature(:, i) - feature(:, j))  <= distance
                count = 1;
            end
        end
        if count == 0
            temp(:, n) = feature(:, i);
            n = n + 1;
        end
    end
end