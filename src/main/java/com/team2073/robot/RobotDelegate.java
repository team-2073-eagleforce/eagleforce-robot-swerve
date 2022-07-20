package com.team2073.robot;

import com.team2073.common.robot.AbstractRobotDelegate;
import com.team2073.robot.Command.Autos.Test;
import com.team2073.robot.Command.Drive.DriveCommand;
import com.team2073.robot.Subsystems.Drive.DrivetrainSubsystem;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class RobotDelegate extends AbstractRobotDelegate {
    private OperatorInterface oi = new OperatorInterface();
    private ApplicationContext appCTX = ApplicationContext.getInstance();
    private Joystick controller = appCTX.getController();
    private DrivetrainSubsystem drivetrain = appCTX.getDrivetrainSubsystem();
    private Test test = new Test();
    private AutoRun autonomous;
    private SendableChooser<AutoRun> autonRun;
    boolean started = false;

    public RobotDelegate(double period) {
        super(period);
    }

    @Override
    public void robotInit() {
        oi.init();
        autonRun = new SendableChooser<>();
        autonomous = AutoRun.Test;
        autonRun.addOption("Test", AutoRun.Test);
        SmartDashboard.putData(autonRun);
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
        if (DriverStation.isDisabled()) {
            autonomous = autonRun.getSelected();
        }

        if (DriverStation.isAutonomous() && DriverStation.isEnabled()) {
            if (started == false) {
                if (autonomous == AutoRun.Test){
                    System.out.println("sahdhkasdjkashkjdasjkdjaks");
                    test.start();
//                    test.schedule();
//                    CommandScheduler.getInstance().run();
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
        Test;
    }
}
