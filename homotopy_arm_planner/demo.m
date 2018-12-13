load 'a_star_-2.mat'
envmap = load('map4.txt');

close all;

%draw the environment
figure('units','normalized','outerposition',[0 0 1 1]);
imagesc(envmap'); axis square; colorbar; colormap jet; hold on;

%armplan should be a matrix of D by N 
%where D is the number of DOFs in the arm (length of armstart) and
%N is the number of steps in the plan 

fprintf(1, 'plan of length %d was found\n', size(armplan,1));

% plot beams
x1 = 41.5;
x2 = 19.5;
x3 = 6.5;
plot([x1; x1], [0; 16.5], 'y--', 'LineWidth',2);
plot([x2; x2], [0; 16.5], 'y--', 'LineWidth',2);
plot([x3; x3], [0; 16.5], 'y--', 'LineWidth',2);
% label them
text(x1-2,1.5,'+1','Color', 'y','FontWeight','bold')
text(x1+2,1.5,'-1','Color', 'y','FontWeight','bold')
text(x2-2,1.5,'+2','Color', 'y','FontWeight','bold')
text(x2+2,1.5,'-2','Color', 'y','FontWeight','bold')
text(x3-2,1.5,'+3','Color', 'y','FontWeight','bold')
text(x3+2,1.5,'-3','Color', 'y','FontWeight','bold')

%draw the plan
% plot(endeffector_armplan(:,1), endeffector_armplan(:,2), 'w-', 'LineWidth',2)

midx = size(envmap,2)/2;
x = zeros(length(armstart)+1,1);
x(1) = midx;
y = zeros(length(armstart)+1,1);
for i = 1:size(armplan)
    for j = 1:length(armstart)
        x(j+1) = x(j) + LINKLENGTH_CELLS*cos(armplan(i,j));
        y(j+1) = y(j) + LINKLENGTH_CELLS*sin(armplan(i,j));
    end
    if i==1
%         plot(x,y, 'r-','LineWidth',2);
        plot(x,y, 'c-');        
    else
        plot(x,y, 'c-');        
    end
    pause(0.1);
end
plot(x,y, 'g-','LineWidth',2);


%armplan