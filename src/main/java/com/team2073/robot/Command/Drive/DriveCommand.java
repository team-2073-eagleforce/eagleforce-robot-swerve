package com.team2073.robot.Command.Drive;

import com.team2073.common.command.AbstractLoggingCommand;
import com.team2073.robot.ApplicationContext;
import com.team2073.robot.Subsystems.Drive.DrivetrainSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;

public class DriveCommand extends AbstractLoggingCommand {
    private ApplicationContext appCTX = ApplicationContext.getInstance();

    private Joystick controller;
    private DrivetrainSubsystem drivetrain;
    private SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(.5);
    private SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(.5);


    public DriveCommand(DrivetrainSubsystem drivetrain, Joystick controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
    }

    @Override
    public void executeDelegate() {
        final double xSpeed = -MathUtil.applyDeadband(controller.getRawAxis(1), .1) * DrivetrainSubsystem.maxSpeedMeters;
//        final double xSpeed = .1 * DrivetrainSubsystem.maxSpeedMeters;
        final double ySpeed = -MathUtil.applyDeadband(controller.getRawAxis(0),.1) * DrivetrainSubsystem.maxSpeedMeters;
        final double rot = -MathUtil.applyDeadband(controller.getRawAxis(4), .1) * DrivetrainSubsystem.maxAngularSpeed;

        boolean calibrate = controller.getRawButton(1);

        drivetrain.drive(xSpeed, ySpeed, rot, true, calibrate);
    }

    @Override
    protected boolean isFinishedDelegate() {
        return false;
    }
}
