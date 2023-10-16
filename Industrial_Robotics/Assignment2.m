classdef Assignment2 < handle
    properties (Access = public)
        GripperBase1;LeftHand;RightHand;
        robot;
        qMatrix = [];
        collision_signal;
        object_pos;object_index;
    end
    methods
        function self = Assignment2()
            close all
            warning('off')
            axis([-2 2 -2 2 -0.01 4])
            hold on
            self.robot = UR3(transl(0,0,1.5));
            baseTr = self.robot.model.fkine(self.robot.model.getpos).T*transl(0,0,-0.01)*troty(pi);
            self.GripperBase1 = GripperBase(baseTr);
            GripperHand1 = self.GripperBase1.model.fkine(self.GripperBase1.model.getpos).T*transl(0,0.015,-0.06)*troty(pi/2);
            GripperHand2 = self.GripperBase1.model.fkine(self.GripperBase1.model.getpos).T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2);
            self.LeftHand = GripperHand(GripperHand1);
            self.RightHand = GripperHand(GripperHand2);
            self.Add_models()
            pause(2)
            self.run()
        end
        %% Add table
        function Add_models(self)
            Table = PlaceObject('Table.ply');
            Table_vertices = get(Table,'Vertices');
            transformedVerticesT = [Table_vertices,ones(size(Table_vertices,1),1)]*troty(-pi/2)'*transl(0,0,1.4)';
            set(Table,'Vertices',transformedVerticesT(:,1:3));
            % Add cans

            Can1 = PlaceObject('Canbody.ply');
            Can1_vertices = get(Can1,'Vertices');
            Can1_transformedVerticesT = [Can1_vertices,ones(size(Can1_vertices,1),1)]*trotx(-pi/2)*transl(0.7,0,1.5)';
            set(Can1,'Vertices',Can1_transformedVerticesT(:,1:3));
            self.object_pos{1} = [0.6,0,1.5];

            surf([-2,-2;2,2],[-2,2;-2,2],[0,0;0,0],'CData',imread('concrete.jpg'),'FaceColor','texturemap','FaceLighting','none');
            surf([2,2;2,2],[-2,2;-2,2],[0,0;3,3],'CData',imread('Wall.jpg'),'FaceColor','texturemap');
            surf([-2,2;-2,2],[2,2;2,2],[0,0;3,3],'CData',imread('Wall.jpg'),'FaceColor','texturemap');

        end
        %% Test Movement
        function self = run(self)
            q_open =  [1.1345,0,0.6213];
            q_close = [0.6319, 0,1.1240];
            q_gripper = jtraj(q_open,q_close,200);
            self.object_index=1;
            self.FindqMatrix;

            self.LeftHand.model.delay = 0;
            self.RightHand.model.delay = 0;
            self.robot.model.delay = 0;
            self.GripperBase1.model.delay = 0;
            for i=1:200
                self.GripperBase1.model.base = self.robot.model.fkine(self.robot.model.getpos).T*transl(0,0,-0.01)*troty(pi);
                self.LeftHand.model.base = self.GripperBase1.model.base.T*transl(0,0.015,-0.06)*troty(pi/2);
                self.RightHand.model.base = self.GripperBase1.model.base.T*trotz(pi)*transl(0,0.015,-0.06)*troty(pi/2);
                self.robot.model.animate(self.qMatrix(i,:));

                self.GripperBase1.model.animate(0);
                self.LeftHand.model.animate(self.LeftHand.model.getpos);
                self.RightHand.model.animate(self.RightHand.model.getpos);

                drawnow
            end
            self.robot.model.fkine(self.robot.model.getpos)
            for i = 1:200
                self.LeftHand.model.animate(q_gripper(i,:));
                self.RightHand.model.animate(q_gripper(i,:));
                drawnow()
            end

        end
        %%
        function FindqMatrix(self)
            cur_object = self.object_pos{self.object_index};
            Q_destination{1} = transl(cur_object(1),cur_object(2),cur_object(3)+0.2)*trotx(-pi/2)*troty(pi/2)*trotz(-pi/2);
            Q_destination{2} = transl(cur_object(1),cur_object(2),cur_object(3)+0.06)*trotx(-pi/2)*troty(pi/2)*trotz(-pi/2);
            Q_iniguest = [2.8695,-0.5221,0.3709,0.1511,1.2987,-1.5708];
            Q{1} = self.robot.model.ikcon(Q_destination{1},Q_iniguest);
            Q{2} = self.robot.model.ikcon(Q_destination{2},Q{1});
            for index = 1:2
                for i= 1:6
                    a = fix(Q{index}(i)/(pi));
                    if (a<-1 || a>1)
                        Q{index}(i) = Q{index}(i) - a*2*pi;
                    end
                end
            end
            self.qMatrix = [jtraj(self.robot.model.getpos,Q{1},100); jtraj(Q{1},Q{2},100)];
            self.collision_signal = self.CheckCollision;
            if self.collision_signal
                self.qMatrix = [];
                disp('Collision Detected!!!!!!')
            end
        end
        %%
        function result = CheckCollision(self)
            result = 0;
            for qIndex = 1:size(self.qMatrix,1)

                % Get the transform of every joint (i.e. start and end of every link)
                tr = GetLinkPoses(self.qMatrix(qIndex,:), self);
                % Go through each link and also each triangle face
                for i = 1 : size(tr,3)-1
                    vertOnPlane = [0,0,0];
                    faceNormals = [0,0,1];
                    [~,check] = LinePlaneIntersection(faceNormals,vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)');
                    if check ==1
                        result = result + 1;
                    end
                end
            end
        end
        %%
        function [ transforms ] = GetLinkPoses(q,self)
            links = self.robot.model.links;
            transforms = zeros(4, 4, length(links) + 1);
            transforms(:,:,1) = self.robot.model.base;

            for i = 1:length(links)
                L = links(1,i);

                current_transform = transforms(:,:, i);

                current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
                    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
                transforms(:,:,i + 1) = current_transform;
            end
        end

    end
end
