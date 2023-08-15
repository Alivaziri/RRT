function varargout = AliVaziri(varargin)
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @AliVaziri_OpeningFcn, ...
    'gui_OutputFcn',  @AliVaziri_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end

function AliVaziri_OpeningFcn(hObject, eventdata, handles, varargin)
handles.start=[];
handles.End=[];
handles.map=ones(1000);
handles.mapList{1}=handles.map;
axes(handles.axes1)
imshow(handles.map)
set(handles.Undo,'visible','off')
set(handles.uipanel3,'visible','off')
set(handles.uipanel4,'visible','off')
handles.StartProcess=0;
handles.EndProcess=0;
handles.output = hObject;
guidata(hObject, handles);

function varargout = AliVaziri_OutputFcn(hObject, eventdata, handles)
varargout{1} = handles.output;

function RRT_Callback(hObject, eventdata, handles)
map=handles.map;
start=handles.start;
End=handles.End;
source=[start(2) start(1)];
goal=[End(2) End(1)];
stepsize=20;
disTh=20;
maxFailedAttempts = 10000;
display=true; 
if ~feasiblePoint(source,map)
    error('source lies on an obstacle or outside map'); 
end
if ~feasiblePoint(goal,map)
    error('goal lies on an obstacle or outside map'); 
end
if display
    imshow(map);
    rectangle('position',[1 1 size(map)-1],'edgecolor','k'); 
end
RRTree=double([source -1]); 
failedAttempts=0;
counter=0;
pathFound=false;
while failedAttempts<=maxFailedAttempts  
    if rand < 0.5
        sample=rand(1,2) .* size(map); 
    else
        sample=goal; 
    end
    [A, I]=min( distanceCost(RRTree(:,1:2),sample) ,[],1); 
    closestNode = RRTree(I(1),1:2);
    theta=atan2(sample(1)-closestNode(1),sample(2)-closestNode(2));  
    newPoint=double(int32(closestNode(1:2)+stepsize*[sin(theta) cos(theta)]));
    if ~checkPath(closestNode(1:2), newPoint, map) 
        failedAttempts=failedAttempts+1;
        continue;
    end
    if distanceCost(newPoint,goal)<disTh
        pathFound=true;
        break; 
    end 
    [A, I2]=min( distanceCost(RRTree(:,1:2),newPoint) ,[],1); 
    if distanceCost(newPoint,RRTree(I2(1),1:2))<disTh
        failedAttempts=failedAttempts+1;
        continue;
    end 
    RRTree=[RRTree;newPoint I(1)]; 
    failedAttempts=0;
    if display 
        line([closestNode(2);newPoint(2)],[closestNode(1);newPoint(1)]);
        counter=counter+1;
        M(counter)=getframe;
    end
end
if display && pathFound 
    line([closestNode(2);goal(2)],[closestNode(1);goal(1)]);
    counter=counter+1;M(counter)=getframe;
    hold on
    plot(start(1),start(2),'*','MarkerSize',15)
    plot(End(1),End(2),'*','MarkerSize',15)
    text(start(1)-20,start(2)-30,'Start','fontweight','bold')
    text(End(1)-20,End(2)-30,'End','fontweight','bold')
end
if ~pathFound
    error('no path found. maximum attempts reached'); 
end
path=[goal];
prev=I(1);
while prev>0
    path=[RRTree(prev,1:2);path];
    prev=RRTree(prev,3);
end
pathLength=0;
for i=1:length(path)-1 
    pathLength=pathLength+distanceCost(path(i,1:2),path(i+1,1:2)); 
end
handles.path=path;
cla(handles.axes1)
axes(handles.axes1)
imshow(map);
rectangle('position',[1 1 size(map)-1],'edgecolor','k');
line(path(:,2),path(:,1));
hold on
plot(start(1),start(2),'b*','MarkerSize',15)
text(start(1)-20,start(2)-30,'Start','fontweight','bold')
plot(End(1),End(2),'r*','MarkerSize',15)
text(End(1)-20,End(2)-30,'End','fontweight','bold')
set(handles.uipanel4,'visible','on')
guidata(hObject,handles)

function Rectangle_Callback(hObject, eventdata, handles)
map=handles.map;
start=handles.start;
End=handles.End;
rect = getrect;
rect=floor(rect);
map= rgb2gray(insertShape(map,'FilledRectangle',rect,'Color','black'));
map=1*(map>=0.8);
imshow(map)
hold on
try
    plot(start(1),start(2),'b*','MarkerSize',15)
    text(start(1)-20,start(2)-30,'Start','fontweight','bold')
end
try
    plot(End(1),End(2),'r*','MarkerSize',15)
    text(End(1)-20,End(2)-30,'End','fontweight','bold')
end
handles.map=map;
handles.mapList{end+1}=handles.map;
set(handles.Undo,'visible','on')
guidata(hObject,handles)

function Circle_Callback(hObject, eventdata, handles)
map=handles.map;
start=handles.start;
End=handles.End;
rect = getrect;
rect=floor(rect);
R=sqrt(rect(3)^2+rect(4)^2);
map = insertShape(map,'FilledCircle',[rect(1) rect(2) R]);
map=rgb2gray(map);
map=1*(map>=0.95);
imshow(map)
hold on
try
    plot(start(1),start(2),'b*','MarkerSize',15)
    text(start(1)-20,start(2)-30,'Start','fontweight','bold')
end
try
    plot(End(1),End(2),'r*','MarkerSize',15)
    text(End(1)-20,End(2)-30,'End','fontweight','bold')
end
handles.map=map;
handles.mapList{end+1}=handles.map;
set(handles.Undo,'visible','on')
guidata(hObject,handles);

function Undo_Callback(hObject, eventdata, handles)
start=handles.start;
End=handles.End;
if length(handles.mapList)>=2
    handles.mapList(end)=[];   
end
if length(handles.mapList)==1
    set(handles.Undo,'visible','off')
end
map=handles.mapList{end};
imshow(map)
hold on
try
    plot(start(1),start(2),'b*','MarkerSize',15)
    text(start(1)-20,start(2)-30,'Start','fontweight','bold')
end
try
    plot(End(1),End(2),'r*','MarkerSize',15)
    text(End(1)-20,End(2)-30,'End','fontweight','bold')
end
handles.map=map;
guidata(hObject,handles)

function StartPoint_Callback(hObject, eventdata, handles)
map=handles.map;
start=floor(ginput(1));
End=handles.End;
cla(handles.axes1)
axes(handles.axes1)
imshow(map)
hold on
plot(start(1),start(2),'b*','MarkerSize',15)
text(start(1)-20,start(2)-30,'Start','fontweight','bold')
try
    plot(End(1),End(2),'r*','MarkerSize',15)
    text(End(1)-20,End(2)-30,'End','fontweight','bold')
end
handles.start=start;
handles.StartProcess=1;
if handles.StartProcess && handles.EndProcess
    set(handles.uipanel3,'visible','on')
end
guidata(hObject, handles);

function EndPoint_Callback(hObject, eventdata, handles)
map=handles.map;
start=handles.start;
End=floor(ginput(1));
cla(handles.axes1)
axes(handles.axes1)
imshow(map)
hold on
try
    plot(start(1),start(2),'b*','MarkerSize',15)
    text(start(1)-20,start(2)-30,'Start','fontweight','bold')
end
plot(End(1),End(2),'r*','MarkerSize',15)
text(End(1)-20,End(2)-30,'End','fontweight','bold')
handles.End=End;
handles.EndProcess=1;
if handles.StartProcess && handles.EndProcess
    set(handles.uipanel3,'visible','on')
end
guidata(hObject, handles);

function h=distanceCost(a,b)
h=sqrt((a(:,1)-b(:,1)).^2+(a(:,2)-b(:,2)).^2);

function feasible=feasiblePoint(point,map)
feasible=true;
if ~(point(1)>=1 && point(1)<=size(map,1) && ...
        point(2)>=1 && point(2)<=size(map,2) && map(point(1),point(2))==1)
    feasible=false;
end

function feasible=checkPath(n,newPos,map)
feasible=true;
dir=atan2(newPos(1)-n(1),newPos(2)-n(2));
for r=0:0.5:sqrt(sum((n-newPos).^2))
    posCheck=n+r.*[sin(dir) cos(dir)];
    if ~(feasiblePoint(ceil(posCheck),map) && feasiblePoint(floor(posCheck),map) ...
            && feasiblePoint([ceil(posCheck(1)) floor(posCheck(2))],map) ...
            && feasiblePoint([floor(posCheck(1)) ceil(posCheck(2))],map))
        feasible=false;
        break;
    end
    if ~feasiblePoint(newPos,map)
        feasible=false; 
    end
end

function [x,y]=carSim(P,phi_max,E)
theta=0;
V=0.1;
l=0.1;
n=1;
x(n)=P(1,2);
y(n)=P(1,1);
for i=2:length(P)
    xd=P(i,2);
    yd=P(i,1);
    e=sqrt((xd-x(n))^2+(yd-y(n))^2);
    if i==length(P)
        E=1;
    end
    while e>E
        phi=atan2d((yd-y(n)),(xd-x(n)))-theta(n);
        if phi>phi_max
            phi=phi_max;
        elseif phi<-phi_max
            phi=-phi_max;
        else
        end
        x(n+1)=V*cosd(theta(n))+x(n);
        y(n+1)=V*sind(theta(n))+y(n);
        theta(n+1)=((V)/l)*tand(phi)+theta(n);
        e=abs(sqrt((xd-x(n+1))^2+(yd-y(n+1))^2));
        n=n+1;
    end    
end

function Car_Callback(hObject, eventdata, handles)
path=handles.path;
map=handles.map;
start=handles.start;
End=handles.End;
[x,y]=carSim(path,60,20);
cla(handles.axes1)
axes(handles.axes1)
imshow(map);
rectangle('position',[1 1 size(map)-1],'edgecolor','k');
line(path(:,2),path(:,1));
hold on
plot(start(1),start(2),'b*','MarkerSize',15)
text(start(1)-20,start(2)-30,'Start','fontweight','bold')
plot(End(1),End(2),'r*','MarkerSize',15)
text(End(1)-20,End(2)-30,'End','fontweight','bold')
for i=1:length(x)
plot(x(i),y(i),'go','MarkerSize',5)
pause(0.0005);
end
guidata(hObject,handles)

function [x,y]=UniSim(P,phi_max,E)
theta=0;V=0.1;l=0.1;n=1;
x(n)=P(1,2);
y(n)=P(1,1);
for i=2:length(P)
    xd=P(i,2);
    yd=P(i,1);
    e=sqrt((xd-x(n))^2+(yd-y(n))^2);
    if i==length(P)
        E=1;
    end
    phi_p=0;
    while e>E
        phi=atan2d((yd-y(n)),(xd-x(n)))-theta(n);        
        if phi>phi_max
            phi=phi_max;
        elseif phi<-phi_max
            phi=-phi_max;
        else
        end
        w=phi-phi_p;
        phi_p=phi;
        x(n+1)=V*cos(theta(n))+x(n);   
        y(n+1)=V*sin(theta(n))+y(n);    
        theta(n+1)=w+theta(n);
        e=abs(sqrt((xd-x(n+1))^2+(yd-y(n+1))^2));
        n=n+1;
    end    
end

function Unicycle_Callback(hObject, eventdata, handles)
path=handles.path;
map=handles.map;
start=handles.start;
End=handles.End;
[x,y]=UniSim(path,90,20);
cla(handles.axes1)
axes(handles.axes1)
imshow(map);
rectangle('position',[1 1 size(map)-1],'edgecolor','k');
line(path(:,2),path(:,1));
hold on
plot(start(1),start(2),'b*','MarkerSize',15)
text(start(1)-20,start(2)-30,'Start','fontweight','bold')
plot(End(1),End(2),'r*','MarkerSize',15)
text(End(1)-20,End(2)-30,'End','fontweight','bold')
for i=1:length(x)
plot(x(i),y(i),'go','MarkerSize',5)
pause(0.0005);
end
guidata(hObject,handles)

function DiffrentialDrive_Callback(hObject, eventdata, handles)
path=handles.path;
map=handles.map;
start=handles.start;
End=handles.End;
[x,y]=DiffSim(path,1);
cla(handles.axes1)
axes(handles.axes1)
imshow(map);
rectangle('position',[1 1 size(map)-1],'edgecolor','k');
line(path(:,2),path(:,1));
hold on
plot(start(1),start(2),'b*','MarkerSize',15)
text(start(1)-20,start(2)-30,'Start','fontweight','bold')
plot(End(1),End(2),'r*','MarkerSize',15)
text(End(1)-20,End(2)-30,'End','fontweight','bold')
for i=1:length(x)
plot(x(i),y(i),'go','MarkerSize',5)
pause(0.0005);
end
guidata(hObject,handles)

function [x,y]=DiffSim(P,E)
r=0.1;L=0.2;n=1;
x(n)=P(1,2);
y(n)=P(1,1);
theta(n)=0;
for i=2:length(P)
    xd=P(i,2);
    yd=P(i,1);
    e=sqrt((xd-x(n))^2+(yd-y(n))^2);
    while e>E
        phi=atan2d((yd-y(n)),(xd-x(n)))-theta(n);
        if phi>0
            Ul=0.4;
            Ur=0.5;
        elseif phi<0
            Ul=0.5;
            Ur=0.4;
        else
            Ul=0.4;
            Ur=0.4;
        end
        x(n+1)=(r/2)*(Ul+Ur)*cos(theta(n))+x(n);                  
        y(n+1)=(r/2)*(Ul+Ur)*sin(theta(n))+y(n);                  
        theta(n+1)=(r/L)*(Ur-Ul)+theta(n);
        e=abs(sqrt((xd-x(n+1))^2+(yd-y(n+1))^2));
        n=n+1;
    end    
end
