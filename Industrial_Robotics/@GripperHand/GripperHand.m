classdef GripperHand < RobotBaseClass

    properties(Access = public)
        plyFileNameStem = 'GripperHand';
    end

    methods
        % Define robot Function
        function self = GripperHand(baseTr)
%              axis([-1 1 -1 1 -1 1])
%              hold on
            self.CreateModel();
            if nargin < 1
                baseTr = eye(4);
            end
            self.model.base = self.model.base.T * baseTr;
            self.homeQ = [1.1345,0,0.6213];
            self.PlotAndColourRobot();
%             self.model.teach([0.8832,0,0.8727])
        end

        % Create the robot model
        function CreateModel(self)

            link(1) = Link('d',0,'a',0,'alpha',0);
            link(2) = Link('d',0,'a',0.075,'alpha',0,'offset',-10.6*pi/180,'qlim',0);
            link(3) = Link('d',0,'a',0,'alpha',pi,'qlim',[pi/90 pi]);

            self.model = SerialLink(link,'name',self.name);
        end

    end
end
