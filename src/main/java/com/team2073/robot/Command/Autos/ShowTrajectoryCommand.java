package com.team2073.robot.Command.Autos;

import com.team2073.common.command.AbstractLoggingCommand;
import com.team2073.robot.ApplicationContext;
import edu.wpi.first.math.trajectory.Trajectory;

public class ShowTrajectoryCommand extends AbstractLoggingCommand {
    private Trajectory trajectory;

    public ShowTrajectoryCommand(Trajectory trajectory) {
        this.trajectory = trajectory;
    }
    @Override
    protected boolean isFinishedDelegate() {
        return true;
    }

    @Override
    protected void initializeDelegate() {
        ApplicationContext.getInstance().getDrivetrainSubsystem().showCurrentTrajectory(trajectory);
    }
}
