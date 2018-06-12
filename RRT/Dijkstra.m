function [distance shortestpath]= Dijkstra( W,start,goal)
n=length(W);
D=W(start,:);
visit=ones(n); visit(start)=0;
parent=zeros(1,n);
shortestpath=[];
for i=1:n-1
    temp=[];
    for j=1:n
        if visit(j)
            temp = [temp D(j)];
        else 
            temp = [temp inf];
        end
    end
    [value,index]=min(temp);
    visit(index)=0;
    for k=1:n
        if D(k)>D(index)+W(index,k)
           D(k)=D(index)+W(index,k);
           parent(k)=index;
        end
    end
end 
distance = D(goal);
t=goal;
while t~=start && t>0
    shortestpath=[t,shortestpath];
    p=parent(t); t=p;
end
shortestpath=[start,shortestpath];
end