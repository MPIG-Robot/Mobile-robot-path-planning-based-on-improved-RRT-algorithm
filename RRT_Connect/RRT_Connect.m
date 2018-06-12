%%%%% parameters
map=im2bw(imread('map1.bmp')); % input map read from a bmp file. for new maps write the file name here
source=[10 10]; % source position in Y, X format
goal=[490 490]; % goal position in Y, X format
stepsize=20; 	% size of each step of the RRT
disTh=20; 		% nodes closer than this threshold are taken as almost the same
maxFailedAttempts = 10000;
display=true; 	% display of RRT
PATH=[];
tic;

if ~feasiblePoint(source,map), error('source lies on an obstacle or outside map'); end
if ~feasiblePoint(goal,map), error('goal lies on an obstacle or outside map'); end
if display, imshow(map);rectangle('position',[1 1 size(map)-1],'edgecolor','k'); end

RRTree1 = double([source -1]); % First RRT rooted at the source, representation node and parent index
RRTree2 = double([goal -1]);   % Second RRT rooted at the goal, representation node and parent index
counter=0;
tree1ExpansionFail = false; % sets to true if expansion after set number of attempts fails
tree2ExpansionFail = false; % sets to true if expansion after set number of attempts fails

while ~tree1ExpansionFail || ~tree2ExpansionFail  % loop to grow RRTs
    if ~tree1ExpansionFail 
        [RRTree1,pathFound,tree1ExpansionFail] = rrtExtend(RRTree1,RRTree2,goal,stepsize,maxFailedAttempts,disTh,map); % RRT 1 expands from source towards goal
        if ~tree1ExpansionFail && isempty(pathFound) && display
            line([RRTree1(end,2);RRTree1(RRTree1(end,3),2)],[RRTree1(end,1);RRTree1(RRTree1(end,3),1)],'color','b');
            counter=counter+1;M(counter)=getframe;
        end
    end
    if ~tree2ExpansionFail 
        [RRTree2,pathFound,tree2ExpansionFail] = rrtExtend(RRTree2,RRTree1,source,stepsize,maxFailedAttempts,disTh,map); % RRT 2 expands from goal towards source
        if ~isempty(pathFound), pathFound(3:4)=pathFound(4:-1:3); end % path found
        if ~tree2ExpansionFail && isempty(pathFound) && display
            line([RRTree2(end,2);RRTree2(RRTree2(end,3),2)],[RRTree2(end,1);RRTree2(RRTree2(end,3),1)],'color','r');
            counter=counter+1;M(counter)=getframe;
        end
    end
    if ~isempty(pathFound) % path found
         if display
            line([RRTree1(pathFound(1,3),2);pathFound(1,2);RRTree2(pathFound(1,4),2)],[RRTree1(pathFound(1,3),1);pathFound(1,1);RRTree2(pathFound(1,4),1)],'color','b');
            counter=counter+1;M(counter)=getframe;
        end
        path=[pathFound(1,1:2)]; % compute path
        prev=pathFound(1,3);     % add nodes from RRT 1 first
        while prev > 0
            path=[RRTree1(prev,1:2);path];
            prev=RRTree1(prev,3);
        end
        prev=pathFound(1,4); % then add nodes from RRT 2
        while prev > 0
            path=[path;RRTree2(prev,1:2)];
            prev=RRTree2(prev,3);
        end
        break;
    end
end

if display 
    disp('click/press any key');
    waitforbuttonpress; 
end

if size(pathFound,1)<=0, error('no path found. maximum attempts reached'); end


dis=zeros(size(path,1),size(path,1));%dijkstra .. the distance between two points that belong to path
for j=1:size(dis,1)
    for k=1:size(dis,2)
        dis(j,k)=distanceCost(path(j,:),path(k,:));
         if ~checkPath(path(j,:),path(k,:),map),dis(j,k)=inf;end
    end   
end
[distance shortestpath]=Dijkstra1(dis,1,size(dis,1));



pathLength=0;
for i=1:length(path)-1, pathLength=pathLength+distanceCost(path(i,1:2),path(i+1,1:2)); end
fprintf('processing time=%d \nPath Length=%d \n\n', toc,pathLength); 
imshow(map);
rectangle('position',[1 1 size(map)-1],'edgecolor','k');%k是黑色
line(path(:,2),path(:,1),'Color','m');%m是紫红色
waitforbuttonpress;
figure(2);
imshow(map);rectangle('position',[1 1 size(map)-1],'edgecolor','k');
line(path(:,2),path(:,1));
xlabel('RRT PATH');
waitforbuttonpress;
for p=1:size(shortestpath,2)
   PATH(p,:)=path(shortestpath(:,p),:);
   line(PATH(:,2),PATH(:,1),'Color','r');
   F=getframe;
end
xlabel('Dijkstra 优化');
fprintf('all processing time=%d\nDijkstra Path Length=%d \n',toc,distance); 