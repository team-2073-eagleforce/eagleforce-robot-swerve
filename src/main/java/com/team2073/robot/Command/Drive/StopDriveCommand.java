package com.team2073.robot.Command.Drive;

import com.team2073.common.command.AbstractLoggingCommand;
import com.team2073.robot.ApplicationContext;
import com.team2073.robot.Subsystems.Drive.DrivetrainSubsystem;

public class StopDriveCommand extends AbstractLoggingCommand {
    private ApplicationContext appCtx = ApplicationContext.getInstance();
    private DrivetrainSubsystem driveSubsystem = appCtx.getDrivetrainSubsystem();

    @Override
    protected void initializeDelegate() {
        driveSubsystem.drive(0,0,0,false,false);
    }

    @Override
    protected boolean isFinishedDelegate() {
        return true;
    }
}
