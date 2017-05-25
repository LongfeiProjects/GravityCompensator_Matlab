%% border of each cell in imagesc plot
function cellborder()
    hold on;
    % plot horizontal border
    horizontal_x = [0.5, 15.5];
    for i = 0:8
        horizontal_y = [i+0.5, i+0.5];
        plot(horizontal_x, horizontal_y, 'k-');
    end
    % plot vertical border
    vertical_y = [0.5, 8.5];
    for j = 0:15
        vertical_x = [j+0.5, j+0.5];
        plot(vertical_x, vertical_y, 'k-');
    end
    hold off;
end