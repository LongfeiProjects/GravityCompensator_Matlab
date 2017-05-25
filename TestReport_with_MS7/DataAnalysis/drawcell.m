function h = drawcell(mat, kcaxis)
% kcaxis = [0, 2]*0.001;
% mat = JointWrench_Error_ForceX;

h = figure;
mat = flipud(mat); % base at the bottom and end effector at the top
[rows, cols] = size(mat);
imagesc(mat), 
colormap(flipud(gray));  %# Change the colormap to gray (so higher values are
                         %#   black and lower values are white)
caxis(kcaxis);

%% draw cell boarder
hold on;
% plot horizontal border
horizontal_x = [0.5, cols+0.5];
for i = 0:rows
    horizontal_y = [i+0.5, i+0.5];
    plot(horizontal_x, horizontal_y, 'k-');
end
% plot vertical border
vertical_y = [0.5, rows+0.5];
for j = 0:cols
    vertical_x = [j+0.5, j+0.5];
    plot(vertical_x, vertical_y, 'k-');
end
hold off;

% print values to the cell
textStrings = num2str(mat(:),'%0.4f');  %# Create strings from the matrix values
textStrings = strtrim(cellstr(textStrings));  %# Remove any space padding
[x,y] = meshgrid([1:cols], [1:rows]);   %# Create x and y coordinates for the strings
hStrings = text(x(:),y(:),textStrings(:), 'HorizontalAlignment','center', 'FontSize', 15); %# Plot the strings
midValue = mean(get(gca,'CLim'));  %# Get the middle value of the color range
textColors = repmat(mat(:) > midValue,1,3);  %# Choose white or black for the
                                             %#   text color of the strings so
                                             %#   they can be easily seen over
                                             %#   the background color
set(hStrings,{'Color'},num2cell(textColors,2));  %# Change the text colors

temp_cell = textscan(sprintf('%i\n', [1:cols]'),'%s');
xTick_cell = reshape(temp_cell{1}, [1 cols]);
set(gca,'XTick',1:cols, 'XTickLabel', xTick_cell, ...                          %# Change the axes tick marks and tick labels
        'YTick',1:rows, 'YTickLabel',{'Actuator7','Actuator6','Actuator5','Actuator4','Actuator3','Actuator2','Actuator1','Base'},...
        'TickLength',[0 0],'FontSize', 20);

xlabel('Index of Pre-defined Robot Configuration');
% title(title_str);

set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
end
