package com.team2073.robot.Command.Autos;

import com.team2073.robot.ApplicationContext;
import com.team2073.robot.Command.Drive.StopDriveCommand;
import com.team2073.robot.Command.Drive.SwerveAutoCommand;
import com.team2073.robot.Command.Drive.SwerveAutoCommand.*;
import com.team2073.robot.Subsystems.Drive.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Test extends CommandGroup {
    private ApplicationContext appCTX = ApplicationContext.getInstance();
    private DrivetrainSubsystem driveSubsystem = appCTX.getDrivetrainSubsystem();

    public Test() {
//        addSequential(new SwerveAutoCommand(DrivetrainSubsystem.AUTO_PATHS_SWERVE.TEST_PATH_2.getTraj(), driveSubsystem));
        addSequential(new StopDriveCommand());
    }
}
