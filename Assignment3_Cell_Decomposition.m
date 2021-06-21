%Naive and Sweep Algorithm Improvisation
%Assignment 3

clc;clear all;
n = input('Input the number of vertices of the 2D configuration space: ');
config_sp = zeros(n,2);
axis([0,10,0,10]);
daspect([1 1 1]);
for i=1:n
    [x,y] = ginput(1);
    hold on;
    plot(x,y,'rx');
    hold off;
    config_sp(i,:) = [x,y];
end
hold on;
edges = polyshape(config_sp(:,1),config_sp(:,2))
plot(polyshape(config_sp(:,1),config_sp(:,2)));
hold off;

vertices = config_sp;

n_o = input('Input the number of obstacles: ');
obstacles={};
for i=1:n_o
    n_oi = input(strcat('Input the number of vertices in', ' obstacle no. ', num2str(i)));
    obstacle = zeros(n_oi,2);
    for j=1:n_oi
        [x,y] = ginput(1);
        hold on;
        plot(x,y,'bx');
        hold off;
        obstacle(j,:) = [x,y];
        vertices = [vertices;x,y];
    end
    hold on;
    plot(polyshape(obstacle(:,1),obstacle(:,2)));
    hold off;
    obstacles =[obstacles;obstacle]
end

svertices = sort(vertices,1);

%% Ensuring that all the X are unique
for i=1:size(svertices,1)
    vi = svertices(i,:);
    for j=1:size(svertices,1)
        vj = svertices(j,:);
        if i~=j
            
        end
    end
    v_line(i) = line([vi(1) vi(1)], [-10 10]);
    X(:,i) = vi(1);
end

if length(X) ~= length(unique(X))
    error('Please Give Obstacle Co-ordinate with Unique Xs !!!');
end

%% Storing Edges in all_edges Structure
L = config_sp;
S = diff([config_sp;config_sp(1,:)]);
for i = 1:length(obstacles)
    
    L = [L;obstacles{i}];
    S = [S;diff([obstacles{i};obstacles{i}(1,:)])];
end

all_edges.initial_point = L;
all_edges.diff = S;
all_edges.final_point = L + S;
all_edges.name = strcat('e',num2str([1:length(L)]'));
all_edges.label = [1:length(L)]';

%% Plotting the Edges
figure(2)
for i = 1:length(L)
    x1 = all_edges.initial_point(i,1);
    x2 = all_edges.initial_point(i,1) + all_edges.diff(i,1);
    
    y1 = all_edges.initial_point(i,2);
    y2 = all_edges.initial_point(i,2) + all_edges.diff(i,2);
    
    x_text = all_edges.initial_point(i,1) + 0.5*all_edges.diff(i,1);
    y_text = all_edges.initial_point(i,2) + 0.5*all_edges.diff(i,2) + 0.2;
    
    line([x1 x2],[y1 y2])
    text(x_text,y_text,all_edges.name(i,:))
end

%% Making the obstacles as polyshape
pygon = polyshape(config_sp(:,1),config_sp(:,2));
for o = 1:length(obstacles)
    pgon(o) = polyshape(obstacles{o}(:,1),obstacles{o}(:,2));
    pygon = subtract(pygon,polyshape(obstacles{o}(:,1),obstacles{o}(:,2)));
end
pgon(o+1) = polyshape(config_sp(:,1),config_sp(:,2));

%% Intersection points and cell decomposition Using naviie Approach
tic
syms t
k =0;
y_up = 0;
y_down = 0;
T_Manager = [];
count1 = 1;
cells = {};

%Loop For all x
for i = 1:length(vertices)
    
    x = X(i);
    y_check = vertices(find(vertices(:,1) == x),2);
    y_intersection = [];
    count = 0;
    %%
    %Finding all the Intersection points of the vertical lines at all the x
    %with the edges
    for j = 1:length(L)
        a = vpa(solve(all_edges.initial_point(j,1) + t*all_edges.diff(j,1) == x,t),3);
        k = double(a);
        if k >= 0 && k <= 1
            y = all_edges.initial_point(j,2) + k*all_edges.diff(j,2);
            count = count + 1;                %that are forward of current x
            y_intersection(count).x = x;
            y_intersection(count).y = y;
            y_intersection(count).c1 = all_edges.initial_point(j,:);
            y_intersection(count).c2 = all_edges.final_point(j,:);
            T = struct2table(y_intersection);
        end
    end
    %%
    %Keeping which line vertical Extension is inside the workspace and
    %outside the obstacles while eliminating others
    T = sortrows(T, 'y');
    upper_segment = T(min(find(y_check < T.y)),:);
    lower_segment = T(max(find(y_check > T.y)),:);
    Temp_T = [upper_segment;T(find(y_check == T.y),:);lower_segment];
    mid_up = (upper_segment.y + y_check)/2;
    mid_down = (lower_segment.y + y_check)/2;

    if ~isempty(mid_up)
        if ~isinterior(pygon,x,mid_up) 
            Temp_T(1,:) = [];
        end
    end

    if ~isempty(mid_down)
        if ~isinterior(pygon,x,mid_down)
            Temp_T(length(Temp_T.x),:) = [];
        end
    end
    
    T = Temp_T;
    %%
    %Plotting the Line extension inside the Configuration space
    figure(3)
    subplot(1,2,1)
    plot(pygon)
    hold on
    line(x.*ones(1,length(unique(T.y))),unique(T.y))
    hold off
    axis([0,10,0,10]);
    title('Configuration space')
    
    %%
    T_Manager = [T_Manager;T];
    Temp_T = table;
    C = unique(T_Manager.c1(:,1));

    while ~isempty(C)
        %Transferring the Common Edges to a temporary Table
        b = C(length(C));
        C(length(C)) = [];
        idx = find(T_Manager.c1(:,1) == b);
        if length(idx) > 1
            Temp_T = [Temp_T;T_Manager(idx,:)];
            Temp_T = unique(Temp_T);
            T_Manager(idx,:) = [];
        end
    end
    
    %Adding Up the Current Line intersection points back
    T_Manager = [T_Manager;T];
    T_Manager = unique(T_Manager);

    
    %Removing the row having the edges on the left side of the current
    %extension
    t1 = find(and((T_Manager.c1(:,1) < x),(T_Manager.c2(:,1) < x)));
    T_Manager(t1,:) = [];

    
    %Converting the Temp_T to Cells
    cell_vertices = [];
    if ~isempty(Temp_T)
        Temp_T = sortrows(Temp_T,'x');
        test = sortrows(Temp_T,'x');
        
        while ~isempty(unique(Temp_T.x))
            C = unique(Temp_T.x);
            idx = find(C(1) == Temp_T.x);
            
            while ~isempty(idx)
                l = 1;
                idx(1) = [];
                index = find(Temp_T.c1(l,1) == Temp_T.c1(:,1));
                cell_vertices = [cell_vertices;[Temp_T.x(index) Temp_T.y(index)]];
                Temp_T([l;index],:) = [];
            
                if length(cell_vertices(:,1)) == 4
                    %Storing in Cells
                    cell_vertices = unique(cell_vertices,'rows');
                    cells{count1} = cell_vertices;
                    count1 = count1 + 1;
                    cell_vertices = [];
                end
            end
        end
    end
%%
    %Plotting Cells After rearranging them for ployshape function
    subplot(1,2,2)
    for d = 1:length(cells)
        cell_vertices = cells{d};
        
        %Rearranging the cell vertices
        temp = [];
        c = unique(cell_vertices(:,1));
        idx_decreasing = find(cell_vertices(:,1) == c(1));
        idx_increasing = find(cell_vertices(:,1) == c(2));
        temp = sort(cell_vertices(idx_decreasing,:),'ascend');
        temp = [temp;sort(cell_vertices(idx_increasing,:),'descend')];
        cell_vertices = temp;
        naive_cells.cells{d} = cell_vertices;
        naive_cells.name{d} = num2str(d);
        [centroid_x,centroid_y] = centroid(polyshape(cell_vertices));
        hold on
        plot(polyshape(cell_vertices))
        text(centroid_x,centroid_y,num2str(d))
        hold off
    end
    axis([0,10,0,10]);
    title('Cell Decompostion Using Naive Method')
    
end
f1 = toc;
%% Improvisation with Sweep Strategy
tic
edge_table = struct2table(all_edges);
%edge_table = removevars(edge_table,{'diff'});
count = 0;
L = table;
T_Manager = table;
jumper = [];
syms t;
cells = {};
count1 = 1;

for i = 1:length(X)
    %%
    %Finding out the edges intersecting current vertical lines 
    jumper = [];
    x = X(i);
    %idx1 = find(edge_table.initial_point(:,1) == x)
    %idx2 = find(edge_table.final_point(:,1) == x)
    idx1 = find(round(edge_table.initial_point(:,1) - x,4) == 0);
    idx2 = find(round(edge_table.final_point(:,1) - x,4) == 0);
    temp = edge_table([idx1;idx2],:);
    list = unique([temp.initial_point(:,1);temp.final_point(:,1)]);
    idx = find(round(x-list,4) == 0);
    list(idx) = [];       %Removing Current Vertices
     
    %%
    %Operations on the List 
    if x < list(1) && x < list(2)
        count = count + 1; 
        length_of_list = 2*count; 
        if count == 1
            L(1,:) = temp(1,:);
            L(length_of_list,:) = temp(2,:);
        else
            L(length_of_list,:) = L(prev_length,:);
            L(2*(count-1),:) = temp(1,:);
            L(2*(count-1)+1,:) = temp(2,:);
        end
    end

    if x > list(1) && x < list(2) || x < list(1) && x > list(2)
        flag = 1;
        for j = 1:2
            idx = find(temp.label(j) == L.label);
            if flag == 1 && ~isempty(idx)
                if j == 1
                    L(idx,:) = temp(2,:);
                    jumper = temp(1,:);
                else
                    L(idx,:) = temp(1,:);
                    jumper = temp(2,:);
                end
                flag = 0;
            end
        end
    end

    if x > list(1) && x > list(2)
        count = count - 1; 
        for j = 1:2
            idx = find(temp.label(j) == L.label);
            jumper = [jumper;L(idx,:)];
            L(idx,:) = [];
        end
    end

    prev_length  = 2*count;
    %%
    %Adding the jumper
    if ~isempty(jumper)
        L_duplicate = [L;jumper];
    else
        L_duplicate = L;
    end
    
    %Making the Cells
    %Finding the intersection 
    counter = 0;
    T = table;
    for j = 1:length(L_duplicate.label)
        a = vpa(solve(L_duplicate.initial_point(j,1) + t*L_duplicate.diff(j,1) == x,t),3);
        k = double(a);
        y = L_duplicate.initial_point(j,2) + k*L_duplicate.diff(j,2);
        counter = counter + 1;                %that are forward of current x
        T.x(counter,:) = x;
        T.y(counter,:) = y;
        T.c1(counter,:) = L_duplicate.initial_point(j,:);
        T.c2(counter,:) = L_duplicate.final_point(j,:);
    end
    
    y_check = vertices(find(vertices(:,1) == x),2);
    
    %Keeping which line vertical Extension is inside the workspace and
    %outside the obstacles while eliminating others
    T = sortrows(T, 'y');
    upper_segment = T(min(find(y_check < T.y)),:);
    lower_segment = T(max(find(y_check > T.y)),:);
    Temp_T = [upper_segment;T(find(y_check == T.y),:);lower_segment];
    mid_up = (upper_segment.y + y_check)/2;
    mid_down = (lower_segment.y + y_check)/2;

    if ~isempty(mid_up)
        if ~isinterior(pygon,x,mid_up) 
            Temp_T(1,:) = [];
        end
    end

    if ~isempty(mid_down)
        if ~isinterior(pygon,x,mid_down)
            Temp_T(length(Temp_T.x),:) = [];
        end
    end
    
    T = Temp_T;
    %%
    %Plotting the verticle Line Extension inside the Configuration space
    figure(4)
    subplot(1,2,1)
    plot(pygon)
    hold on
    line(x.*ones(1,length(unique(T.y))),unique(T.y))
    hold off
    axis([0,10,0,10]);
    title('Configuration space');
    
    %%
    %Cells Making 
    T_Manager = [T_Manager;T];
    Temp_T = table;
    C = unique(T_Manager.c1(:,1));

    while ~isempty(C)
        %Transferring the Common Edges to a temporary Table
        b = C(length(C));
        C(length(C)) = [];
        idx = find(T_Manager.c1(:,1) == b);
        if length(idx) > 1
            Temp_T = [Temp_T;T_Manager(idx,:)];
            Temp_T = unique(Temp_T);
            T_Manager(idx,:) = [];
        end
    end
    
    %Adding Up the Current Line intersection
    T_Manager = [T_Manager;T];
    T_Manager = unique(T_Manager);

    
    %Removing the row having the edges on the left side of the current
    %extension
    t1 = find(and((T_Manager.c1(:,1) < x),(T_Manager.c2(:,1) < x)));
    T_Manager(t1,:) = [];

    
    %Converting the Temp_T to Cells
    cell_vertices = [];
    if ~isempty(Temp_T)
        Temp_T = sortrows(Temp_T,'x');
        test = sortrows(Temp_T,'x');
        
        while ~isempty(unique(Temp_T.x))
            C = unique(Temp_T.x);
            idx = find(C(1) == Temp_T.x);
            
            while ~isempty(idx)
                l = 1;
                idx(1) = [];
                index = find(Temp_T.c1(l,1) == Temp_T.c1(:,1));
                cell_vertices = [cell_vertices;[Temp_T.x(index) Temp_T.y(index)]];
                Temp_T([l;index],:) = [];
            
                if length(cell_vertices(:,1)) == 4
                    %Storing in Cells
                    cell_vertices = unique(cell_vertices,'rows');
                    cells{count1} = cell_vertices;
                    count1 = count1 + 1;
                    cell_vertices = [];
                end
            end
        end
    end
    
    %%
    %Plotting the Cells
    subplot(1,2,2)
    for d = 1:length(cells)
        cell_vertices = cells{d};
        
        %Rearranging the cell vertices
        temp = [];
        c = unique(cell_vertices(:,1));
        idx_decreasing = find(cell_vertices(:,1) == c(1));
        idx_increasing = find(cell_vertices(:,1) == c(2));
        temp = sort(cell_vertices(idx_decreasing,:),'ascend');
        temp = [temp;sort(cell_vertices(idx_increasing,:),'descend')];
        cell_vertices = temp;
        sweep_cells.cells{d} = cell_vertices;
        sweep_cells.name{d} = num2str(d);
        [centroid_x,centroid_y] = centroid(polyshape(cell_vertices));
        hold on
        plot(polyshape(cell_vertices))
        text(centroid_x,centroid_y,num2str(d))
        hold off
    end
    axis([0,10,0,10]);
    title('Cell Decomposition Using Sweep Strategy')
end
%%
f2 = toc;
fprintf('\n\n\nThe Naive method took %.2f sec\n',f1)
fprintf('The Sweep method took %.2f sec\n\n\n',f2)
