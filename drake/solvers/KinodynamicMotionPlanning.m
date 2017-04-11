classdef KinodynamicMotionPlanning 
  properties (Constant)
    TRAPPED = 0;
    REACHED = 1;
    ADVANCED = 2;
  end
    
properties (Access = public)
    Tree
    parent
    xinit
    sampling_lb
    sampling_ub
    Control_lb
    Control_ub
    torque_limit=5;
    qlimit=2*pi;
    qdotlimit=10;
    Min_dt=0.01;
    Max_dt=0.1;
    x_start;
    x_goal;
    num_vars
    num_input
    num_ControlSamples=10;
    EdgeVariableNames
    Traj
    nq
    n
    children
    R_goal=0.4;
    fixedtime=false;
    timestep=0.1;
    Ndiscretizations=2;
    GoalBias=0.1;
    visualize=false;
end
  
 methods

    function obj = KinodynamicMotionPlanning(num_vars,num_input,xinit)
        
      if nargin < 2
          obj.num_input=0; 
      else
          obj.num_input=num_input;
      end 
      
      if nargin < 3
          xinit=zeros(num_vars,1);             
      end 
          
      obj.nq=num_vars/2;
      
      obj.num_vars=num_vars;      
      
      %Bounds on configuration sampling 
      qsampling_lb = ones(obj.nq, 1)*-obj.qlimit;
      qsampling_ub = ones(obj.nq, 1)*obj.qlimit; 
     
      %Bounds on qdot sampling 
      qdotsampling_lb = ones(obj.nq, 1)*-obj.qdotlimit;
      qdotsampling_ub = ones(obj.nq, 1)*obj.qdotlimit; 
      
      %Bounds on torque control sampling 
      obj.Control_lb = ones(obj.num_input, 1)*-obj.torque_limit;
      obj.Control_ub = ones(obj.num_input, 1)*obj.torque_limit; 
           
      
      % state sampling bound vector
      obj.sampling_lb=[qsampling_lb;qdotsampling_lb];
      obj.sampling_ub=[qsampling_ub;qdotsampling_ub];
      

      
      % Initialize Tree
      obj.Tree=digraph();  %create empty tree
      
      %Nodes
      obj.Tree=obj.Tree.addnode(1); %add root node 
      obj.n=1;
      
      for i=1:num_vars
      obj.Tree.Nodes.State{1,i}=xinit(i);  %Create node variable State and initialize it with qinit
      end 
      %Edges
      obj.EdgeVariableNames={'EndNodes','Distance','Control','Timestep'};
      EdgeTable=table([1 2],[0],[0],[0],'VariableNames',obj.EdgeVariableNames);
      obj.Tree=obj.Tree.addedge(EdgeTable);
      obj.Tree=obj.Tree.rmedge(1);
      obj.Tree=obj.Tree.rmnode(2);
      
      %Traj structre to store trajectories in the tree
      %obj.Traj = struct('ID',[],'xtraj',[],'utraj',[]);
      obj.Traj=struct('nodeStartID',[],'nodeEndID',[],'xtraj',[],'utraj',[],'dt',[]);
      obj.Traj(1)=[];

    end 
    
    
%       for i=1:num_input
%       Tree.Nodes.Control{1,i}=u0(i);  %Create node variable Control and initialize it with u0
%       end 
%     end
     
    function d=distanceMetric(obj,x1,x2)
     d = sqrt(sum(bsxfun(@minus, x1, x2).^2,1));
     end
     
     function [obj,path_ids,info,tElapsed]=RRT(obj,x_start,x_goal,options)
      tStart=tic;
      set(gcf,'currentchar',' ')         % set a dummy character
      defaultOptions.display_after_every = 50;
      defaultOptions.goal_bias = obj.GoalBias;
      defaultOptions.N = 1e4;
      defaultOptions.visualize = false;
      options = applyDefaults(options, defaultOptions);
      
      if nargin < 3
          x_start=obj.xinit;
      end 
      %Re-add root state alues
      for i=1:length(x_start)
      obj.Tree.Nodes.State{1,i}=x_start(i);
      end
      
      obj.x_start=x_start;
      obj.x_goal=x_goal;
      
       x=x_start(1);
       y=x_start(2);
      
      info=2;
      tloopStart = tic;
      while obj.n < options.N
          try_goal=rand <options.goal_bias;
          if try_goal
              x_sample=x_goal;
          else 
              x_sample=obj.randomSample(); 
          end  
        [obj, status,xnextbest,x_near] = extend(obj, x_sample);
                   
        
        
        fprintf('Nodes:%d\n',obj.n);
       
        Elapsedtime = toc(tloopStart);
        
        fprintf('Elapsed time:%8.2f\n',Elapsedtime);
        
        if options.visualize

           figure(5)
           plot(x_sample(1),x_sample(2),'co','MarkerSize',10,'LineWidth',3)
           hold on
           plot(x_start(1),x_start(2),'rx',x_goal(1),x_goal(2),'gx','MarkerSize',20,'LineWidth',3);
           obj.plotGoalTolerance(x_goal,obj.R_goal);
           plot(x_near(1),x_near(2),'ro','MarkerSize',10,'LineWidth',3)
           plot(xnextbest(1),xnextbest(2),'go','MarkerSize',10,'LineWidth',3)

            x=[x xnextbest(1)];
            y=[y xnextbest(2)];
           plot(obj.Tree,'XData',x,'YData',y); 
           axis square 
           drawnow
           hold off 
    
        end
        
        %if try_goal && status == obj.REACHED
        if status == obj.REACHED
          info = 1;
          path_ids = obj.getPathToNode(obj.n);
          tElapsed = toc(tStart);
          break;
        end
       path_ids = obj.getPathToNode(obj.n);
       
              
       if get(gcf,'currentchar')==' '  % which gets changed when key is pressed
           tElapsed = toc(tStart);
            break;
        end 
      end 
             
 end 
 
    function x = randomSample(obj)
      x = obj.sampling_lb + (obj.sampling_ub-obj.sampling_lb).*rand(obj.num_vars,1);
    end
    
    
    function [obj, id,T] = addNode(obj, x)
      obj.n = obj.n + 1;
      id = obj.n;
      T=obj.Tree;
      T=T.addnode(1);
      for i=1:obj.num_vars
      T.Nodes.State{id,i}=x(i);  %Create node variable State and add state value
      end
      obj.Tree=T;
    end
    
    function path_ids = getPathToNode(obj, leaf_id)
     path_ids = leaf_id;
      while path_ids(1) > 1
        path_ids = [obj.Tree.predecessors(path_ids(1)),path_ids];
      end
    end
    
    
     function [obj, status, xnextbest ,x_near,id_next] = extend(obj, x_sample)
      [x_near, id_near] = newPoint(obj, x_sample);
      
      [xnextbest, Ubest,dbest,dtbest,xtrajbest,utrajbest]=Propogate(obj,x_near,x_sample);
      
      [obj, id_next] = obj.addNode(xnextbest);
      
      EdgeTable=table([id_near id_next],dbest,Ubest,dtbest,'VariableNames',obj.EdgeVariableNames);
      obj.Tree=obj.Tree.addedge(EdgeTable);
      obj.Traj(obj.n)=struct('nodeStartID',id_near,'nodeEndID',id_next,'xtraj',xtrajbest,'utraj',utrajbest,'dt',dtbest);
      %obj.tree=obj.Tree.addedge(id_near,id_next); 
     
      %visualize state space plot
      if obj.visualize
      figure(10); 
      plot(obj.x_start(1),obj.x_start(2),'rx',obj.x_goal(1),obj.x_goal(2),'gx','MarkerSize',20,'LineWidth',3);
      hold on 
      plot(x_sample(1),x_sample(2),'co','MarkerSize',10,'LineWidth',3)
      plot(x_near(1),x_near(2),'ro','MarkerSize',10,'LineWidth',3)
      plot(xnextbest(1),xnextbest(2),'go','MarkerSize',10,'LineWidth',3)
      obj.plotGoalTolerance(obj.x_goal,obj.R_goal);
      axis square     
     
      for i=2:obj.n
      fnplt(obj.Traj(i).xtraj);
      end
      hold off
      end 
       
        d2goal=obj.distanceMetric(xnextbest,obj.x_goal); %distance to goal 
        fprintf('d2goal:%4.2f \n',d2goal);

        if d2goal < obj.R_goal     
          status = obj.REACHED;
        else
          status = obj.ADVANCED;
        end
        
      end
        
     
     function [x_new, id_near,d] = newPoint(obj, x)
      [d, id_near] = obj.nearestNeighbor(x);
        x_new = obj.getNode(id_near);
     end 
     
     function   x = getNode(obj, id)
         StateIndex=1;
        StateTable= obj.Tree.Nodes(id,StateIndex); % state in table format
        S=cell2mat(StateTable{1,1}); % state in horizontal array 
        x= reshape(S,[length(S),1]); % State in vertical array
     end 
     
    function [d, id_near] = nearestNeighbor(obj, q)
        d_all=[];
        for i=1:obj.n
          qtree=  obj.getNode(i);
       d_all = [d_all; obj.distanceMetric(q, qtree)];
        end 
      [d, id_near] = min(d_all);
    end
    
    function Urand =SampleControl(obj)
        Urand= obj.Control_lb + (obj.Control_ub-obj.Control_lb).*rand(obj.num_input,1);
    end 
    
    function dt =SampleTime(obj)
        dt= obj.Min_dt + (obj.Max_dt-obj.Min_dt)*rand(1);
    end 
    
    function [xnextbest, Ubest,dbest,dtbest,xtrajbest,utrajbest]=Propogate(obj,x_near,xdesired)

        Ubest=NaN;
        dbest=Inf; 
       if obj.fixedtime 
           dt=obj.timestep;
       else 
          dt=obj.SampleTime();
       end 
        
        for i=1:obj.num_ControlSamples 
            Urand=obj.SampleControl();
            [xnext,xtraj,utraj]=SimulateSystem(Urand,dt,obj.Ndiscretizations,x_near);
            d=obj.distanceMetric(xnext,xdesired);
            if d < dbest 
                dtbest=dt;
                dbest=d ;
                Ubest=Urand ;
                xnextbest=xnext;
                xtrajbest=xtraj;
                utrajbest=utraj;
            end 
        end 
        
    end 
    
    function[Finalxtraj,Finalutraj]=extractTrajectory(obj,path_ids)
        for i=1:(length(path_ids)-1)
            f=find([obj.Traj.nodeStartID]==path_ids(i) & [obj.Traj.nodeEndID]==path_ids(i+1));
            
            Finalxtraj{i}=obj.Traj(f+1).xtraj;
            Finalutraj{i}=obj.Traj(f+1).utraj;
            figure (3)
            fnplt(Finalxtraj{i})
            hold on 
            figure (4)
            fnplt(Finalutraj{i})
            hold on
           
        end 
        
        pv = PendulumVisualizer();
        for i=1:length(Finalxtraj)
            pv.playback(Finalxtraj{i});
        end 
            
    end 
    
    function Cplot=plotGoalTolerance(obj,center,radius)
        N=50;
       
        t = 2*pi/N*(1:N); 
        %theta=linspace(0,2*pi,numPoints);
        %rho=ones(1,numPoints)*radius;
        
        %[X,Y]=plo2cart(theta,rho);
        X=radius*cos(t)+center(1);
        Y=radius*sin(t)+center(2);
        
        Cplot=patch(X,Y,1);
        set(Cplot,'FaceColor',[1,0,0],'FaceAlpha',0.1)
        
    end 
        
        
    
    
    
 end
    
    
end 


%test
