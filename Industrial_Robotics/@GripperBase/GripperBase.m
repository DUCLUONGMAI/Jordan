classdef GripperBase < RobotBaseClass
    % LinearUR5 UR5 on a non-standard linear rail created by a student

    properties(Access = public)
        plyFileNameStem = 'GripperBase';

    end

    methods
        % Define robot Function
        function self = GripperBase(baseTr)
%             axis([-1 1 -1 1 -1 1])
%             hold on
            self.CreateModel();
            if nargin < 1
                baseTr = eye(4);
            end
            self.model.base = self.model.base.T * baseTr;
            self.model.plotopt = {'nojoints', 'noname', 'noshadow', 'nowrist','nobase','notiles','nojaxes'};
            self.PlotAndColourRobot();

        end

        % Create the robot model
        function CreateModel(self)
            link(1) = Link('d',-0.0145,'a',0,'alpha',0);
            self.model = SerialLink(link,'name',self.name);

        end

    end
end
