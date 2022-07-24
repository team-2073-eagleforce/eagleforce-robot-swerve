package com.team2073.robot.Command.Autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.team2073.robot.AppConstants;
import com.team2073.robot.ApplicationContext;
import com.team2073.robot.Command.Drive.StopDriveCommand;
import com.team2073.robot.Command.Drive.SwerveAutoCommand;
import com.team2073.robot.Subsystems.Drive.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class FiveBallAuto extends CommandGroup {
    private ApplicationContext appCTX = ApplicationContext.getInstance();
    private DrivetrainSubsystem driveSubsystem = appCTX.getDrivetrainSubsystem();
    private double maxSpeed = AppConstants.AutoConstants.kMaxSpeedMetersPerSecond;
    private double maxAcceleration = AppConstants.AutoConstants.kMaxAccelerationMetersPerSecondSquared;

    PathPlannerTrajectory fiveBallPt1 = PathPlanner.loadPath("5 Ball Pt 1", maxSpeed, maxAcceleration);
    PathPlannerTrajectory fiveBallPt2 = PathPlanner.loadPath("5 Ball Pt 2", maxSpeed, maxAcceleration);
    PathPlannerTrajectory fiveBallPt3 = PathPlanner.loadPath("5 Ball Pt 3", maxSpeed, maxAcceleration);
    PathPlannerTrajectory fiveBallPt4 = PathPlanner.loadPath("5 Ball Pt 4", maxSpeed, maxAcceleration);

    public FiveBallAuto() {
        addSequential(new SwerveAutoCommand(fiveBallPt1, driveSubsystem));
        addSequential(new StopDriveCommand());
        addSequential(new WaitCommand(.2));
        addSequential(new SwerveAutoCommand(fiveBallPt2, driveSubsystem));
        addSequential(new StopDriveCommand());
        addSequential(new WaitCommand(.2));
        addSequential(new SwerveAutoCommand(fiveBallPt3, driveSubsystem));
        addSequential(new StopDriveCommand());
        addSequential(new WaitCommand(.2));
        addSequential(new SwerveAutoCommand(fiveBallPt4, driveSubsystem));
        addSequential(new StopDriveCommand());
        addSequential(new WaitCommand(.2));
    }
}
