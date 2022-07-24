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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FourBallAuto extends CommandGroup {
    private ApplicationContext appCTX = ApplicationContext.getInstance();
    private DrivetrainSubsystem driveSubsystem = appCTX.getDrivetrainSubsystem();
    private double maxSpeed = AppConstants.AutoConstants.kMaxSpeedMetersPerSecond;
    private double maxAcceleration = AppConstants.AutoConstants.kMaxAccelerationMetersPerSecondSquared;

    PathPlannerTrajectory fourBallPt1 = PathPlanner.loadPath("4 Ball Pt 1", maxSpeed, maxAcceleration);
    PathPlannerTrajectory fourBallPt2 = PathPlanner.loadPath("4 Ball Pt 2", maxSpeed, maxAcceleration);
    PathPlannerTrajectory fourBallPt3 = PathPlanner.loadPath("4 Ball Pt 3", maxSpeed, maxAcceleration);

    public FourBallAuto() {
        addSequential(new SwerveAutoCommand(fourBallPt1, driveSubsystem));
        addSequential(new StopDriveCommand());
        addSequential(new WaitCommand(.2));
        addSequential(new SwerveAutoCommand(fourBallPt2, driveSubsystem));
        addSequential(new StopDriveCommand());
        addSequential(new WaitCommand(.2));
        addSequential(new SwerveAutoCommand(fourBallPt3, driveSubsystem));
        addSequential(new StopDriveCommand());
    }
}
