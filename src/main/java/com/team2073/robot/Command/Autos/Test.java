package com.team2073.robot.Command.Autos;

import com.team2073.robot.ApplicationContext;
import com.team2073.robot.Command.Drive.StopDriveCommand;
import com.team2073.robot.Command.Drive.SwerveAutoCommand;
import com.team2073.robot.Subsystems.Drive.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class Test extends CommandGroup {
    private ApplicationContext appCTX = ApplicationContext.getInstance();
    private DrivetrainSubsystem driveSubsystem = appCTX.getDrivetrainSubsystem();

    public Test() {
        addSequential(new ShowTrajectoryCommand(DrivetrainSubsystem.AUTO_PATHS_SWERVE.TEST_PATH.getTraj()));
        addSequential(new SwerveAutoCommand(DrivetrainSubsystem.AUTO_PATHS_SWERVE.TEST_PATH.getTraj(), driveSubsystem));
        addSequential(new StopDriveCommand());
    }
}
