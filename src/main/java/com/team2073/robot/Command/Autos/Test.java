package com.team2073.robot.Command.Autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.team2073.robot.AppConstants;
import com.team2073.robot.ApplicationContext;
import com.team2073.robot.Command.Drive.StopDriveCommand;
import com.team2073.robot.Command.Drive.SwerveAutoCommand;
import com.team2073.robot.Command.Drive.SwerveAutoCommand.*;
import com.team2073.robot.Subsystems.Drive.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import static com.team2073.robot.AppConstants.AutoConstants.*;
import static com.team2073.robot.AppConstants.AutoConstants.kThetaControllerConstraints;


public class Test extends CommandGroup {
    private ApplicationContext appCTX = ApplicationContext.getInstance();
    private DrivetrainSubsystem driveSubsystem = appCTX.getDrivetrainSubsystem();
    private double maxSpeed = AppConstants.AutoConstants.kMaxSpeedMetersPerSecond;
    private double maxAcceleration = AppConstants.AutoConstants.kMaxAccelerationMetersPerSecondSquared;

    PathPlannerTrajectory testPath = PathPlanner.loadPath("Test", maxSpeed, maxAcceleration);

    public Test() {
        addSequential(new SwerveAutoCommand(testPath, driveSubsystem));
//        addSequential(new SwerveAutoCommand(testPath, driveSubsystem::getPose, AppConstants.DriveConstants.kinematics,new PIDController(kPXController,0,0),new PIDController(kPYController,0,0),new ProfiledPIDController(kPThetaController,0,0,kThetaControllerConstraints),driveSubsystem::setModuleStates, driveSubsystem));
        addSequential(new StopDriveCommand());
    }
}
