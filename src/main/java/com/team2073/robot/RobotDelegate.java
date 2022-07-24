package com.team2073.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.team2073.common.robot.AbstractRobotDelegate;
import com.team2073.robot.Command.Autos.FiveBallAuto;
import com.team2073.robot.Command.Autos.FourBallAuto;
import com.team2073.robot.Command.Autos.Test;
import com.team2073.robot.Command.Drive.DriveCommand;
import com.team2073.robot.Subsystems.Drive.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotDelegate extends AbstractRobotDelegate {
    private OperatorInterface oi = new OperatorInterface();
    private ApplicationContext appCTX = ApplicationContext.getInstance();
    private Joystick controller = appCTX.getController();
//    private FiveBallAuto fiveBallAuto = new FiveBallAuto();
    private DrivetrainSubsystem drivetrain;
    private AutoRun autonomous;
    private SendableChooser<AutoRun> autonRun;
    boolean started = false;
    boolean hasEnabled = false;

    private double maxSpeed = AppConstants.AutoConstants.kMaxSpeedMetersPerSecond;
    private double maxAcceleration = AppConstants.AutoConstants.kMaxAccelerationMetersPerSecondSquared;

    PathPlannerTrajectory fourBallPt1 = PathPlanner.loadPath("4 Ball Pt 1", maxSpeed, maxAcceleration);
    PathPlannerTrajectory fiveBallPt1 = PathPlanner.loadPath("5 Ball Pt 1", maxSpeed, maxAcceleration);


    public RobotDelegate(double period) {
        super(period);
    }

    @Override
    public void robotInit() {
        drivetrain = appCTX.getDrivetrainSubsystem();
        oi.init();
        autonRun = new SendableChooser<>();
        autonRun.addOption("Test", AutoRun.Test);
        autonRun.addOption("Four Ball", AutoRun.FOUR_BALL);
        autonRun.addOption("Five Ball", AutoRun.FIVE_BALL);
        SmartDashboard.putData("Auto Chooser", autonRun);
    }

    @Override
    public void teleopPeriodic() {
    }


    @Override
    public void teleopInit() {
        teleopDrive();
    }

    @Override
    public void robotPeriodic() {
        if (DriverStation.isDisabled() && !hasEnabled) {
            autonomous = autonRun.getSelected();
            if (autonomous == AutoRun.FIVE_BALL) {
                drivetrain.resetOdometry(fiveBallPt1.getInitialPose(), fiveBallPt1);
            }else if (autonomous == AutoRun.FOUR_BALL){
                drivetrain.resetOdometry(fourBallPt1.getInitialPose(), fourBallPt1);
            }
        }

        if (DriverStation.isAutonomous() && DriverStation.isEnabled()) {
            hasEnabled = true;
            if (!started) {
                if (autonomous == AutoRun.Test) {
                    new Test().start();
                    started = true;
                }else if (autonomous == AutoRun.FOUR_BALL) {
                    new FourBallAuto().start();
                    started = true;
                }else if (autonomous == AutoRun.FIVE_BALL) {
                    new FiveBallAuto().start();
                    started = true;
                }
            }
        }
    }

    @Override
    public void autonomousInit() {
    }

    private void teleopDrive() {
        new DriveCommand(drivetrain, controller).start();
    }

    public enum AutoRun {
        Test,
        FOUR_BALL,
        FIVE_BALL;
    }
}
