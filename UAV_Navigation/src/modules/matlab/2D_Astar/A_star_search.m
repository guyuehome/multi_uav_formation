function path = A_star_search(map,MAX_X,MAX_Y)
%%
%This part is about map/obstacle/and other settings
    %pre-process the grid map, add offset
    size_map = size(map,1);
    Y_offset = 0;
    X_offset = 0;
    
    %Define the 2D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1)) + X_offset;
    yval=floor(map(size_map, 2)) + Y_offset;
    xTarget=xval;
    yTarget=yval;
    MAP(xval,yval)=0;
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1)) + X_offset;
        yval=floor(map(i, 2)) + Y_offset;
        MAP(xval,yval)=-1;
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1)) + X_offset;
    yval=floor(map(1, 2)) + Y_offset;
    xStart=xval;
    yStart=yval;
    MAP(xval,yval)=1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %LISTS USED FOR ALGORITHM
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %OPEN LIST STRUCTURE
    %--------------------------------------------------------------------------
    %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    %--------------------------------------------------------------------------
    OPEN=[];
    %CLOSED LIST STRUCTURE
    %--------------
    %X val | Y val |
    %--------------
    % CLOSED=zeros(MAX_VAL,2);
    CLOSED=[];

    %Put all obstacles on the Closed list
    k=1;%Dummy counter
    for i=1:MAX_X
        for j=1:MAX_Y
            if(MAP(i,j) == -1)
                CLOSED(k,1)=i;
                CLOSED(k,2)=j;
                k=k+1;
            end
        end
    end
    CLOSED_COUNT=size(CLOSED,1);
    %set the starting node as the first node
    xNode=xval;
    yNode=yval;
    OPEN_COUNT=1;
    goal_distance=distance(xNode,yNode,xTarget,yTarget);
    path_cost=0;
    OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,goal_distance,path_cost,goal_distance);
    OPEN(OPEN_COUNT,1)=0;
    CLOSED_COUNT=CLOSED_COUNT+1;
    CLOSED(CLOSED_COUNT,1)=xNode;
    CLOSED(CLOSED_COUNT,2)=yNode;
    NoPath=1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    while(1) %you have to dicide the Conditions for while loop exit 
     gn = distance(xStart,yStart,xNode,yNode);
     exp_array=expand_array(xNode,yNode,gn,xTarget,yTarget,CLOSED,MAX_X,MAX_Y);
     size_array = size(exp_array,1);
     for i=size_array:-1:1
         flag=0;
        for j=1:OPEN_COUNT
            if(OPEN(j,2) == exp_array(i,1) && OPEN(j,3) == exp_array(i,2))%判断
                if(exp_array(i,4)<OPEN(j,7))
                   OPEN(j,7) = exp_array(i,4);
                end
                flag=1;
            end
        end
        if(flag == 0)
           OPEN_COUNT=OPEN_COUNT+1;
           OPEN(OPEN_COUNT,:)=insert_open(exp_array(i,1),exp_array(i,2),xNode,yNode,exp_array(i,3),exp_array(i,4),exp_array(i,5));
           OPEN(OPEN_COUNT,1)=1;
        end
     end
     list_min=min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);
     if(OPEN(list_min,2) == xTarget && OPEN(list_min,3) == yTarget)   
         break;
     end
    xNode = OPEN(list_min,2);
    yNode = OPEN(list_min,3);
    OPEN(list_min,1)=0;
    CLOSED_COUNT=CLOSED_COUNT+1;
    CLOSED(CLOSED_COUNT,1)=xNode;
    CLOSED(CLOSED_COUNT,2)=yNode;

    end %End of While Loop
    
   path = [];
   path(1,1)=OPEN(list_min,2);
   path(1,2)=OPEN(list_min,3);
   i=2;
   n_index = node_index(OPEN,OPEN(list_min,4),OPEN(list_min,5));
   while(1)
       list_min = n_index;
       path(i,1)=OPEN(n_index,2);
       path(i,2)=OPEN(n_index,3);
       if(path(i,1) == 1 && path(i,2) == 1)
           break;
       end
       i=i+1;
       n_index = node_index(OPEN,OPEN(list_min,4),OPEN(list_min,5));
   end
end
