package com.team2073.robot.Command.Drive;

import com.team2073.common.command.AbstractLoggingCommand;
import com.team2073.robot.ApplicationContext;
import com.team2073.robot.Subsystems.Drive.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Joystick;

public class angleLockCommand extends AbstractLoggingCommand {
    private ApplicationContext appCTX = ApplicationContext.getInstance();
    private DrivetrainSubsystem drivetrainSubsystem = appCTX.getDrivetrainSubsystem();

    @Override
    protected void initializeDelegate() {
        drivetrainSubsystem.setIsAngleLocked(true);
    }

    @Override
    protected void endDelegate() {
        drivetrainSubsystem.setIsAngleLocked(false);
    }

    @Override
    protected boolean isFinishedDelegate() {
        return false;
    }
}
