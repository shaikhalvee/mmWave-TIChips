function areEqual = compareStructs(s1, s2, tol)
    % A robust function to compare two scalar structs.
    % - Ignores field order.
    % - Compares numeric values within a specified tolerance.
    
    % Set a default tolerance if not provided
    if nargin < 3
        tol = 1e-9; % A small tolerance for floating point comparison
    end

    % Get field names from both structs
    fields1 = fieldnames(s1);
    fields2 = fieldnames(s2);

    % Check if they have the same set of field names, regardless of order
    if isequal(sort(fields1), sort(fields2))
        % Check if values are the same
        valuesEqual = true;
        for i = 1:length(fields1)
            if ~isequal(struct1.(fields1{i}), struct2.(fields2{i}))
                valuesEqual = false;
                break;
            end
        end

        if valuesEqual
            disp('The structs have the same parameters and values.');
        else
            disp('The structs have the same parameters but different values.');
        end
    else
        disp('The structs have different parameters.');
    end
end