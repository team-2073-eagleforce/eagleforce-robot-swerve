package com.team2073.robot.Command.Drive;

import com.team2073.common.command.AbstractLoggingCommand;
import com.team2073.common.util.ConversionUtil;
import com.team2073.robot.ApplicationContext;
import com.team2073.robot.Subsystems.Drive.DrivetrainSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import org.opencv.core.Mat;

import java.security.PrivateKey;

import static com.team2073.robot.AppConstants.AutoConstants.kPThetaController;
import static com.team2073.robot.AppConstants.AutoConstants.kThetaControllerConstraints;
import static com.team2073.robot.AppConstants.DriveConstants.kAngleControllerConstraints;

public class DriveCommand extends AbstractLoggingCommand {
    private ApplicationContext appCTX = ApplicationContext.getInstance();

    private Joystick controller;
    private DrivetrainSubsystem drivetrain;
    private SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(.5);
    private SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(.5);
    private PIDController anglecontroller = new PIDController(3.5, 0, .09);
    private double angle = 0;


    public DriveCommand(DrivetrainSubsystem drivetrain, Joystick controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        anglecontroller.enableContinuousInput(0, 2* Math.PI);
    }

    @Override
    public void executeDelegate() {
        final double xSpeed = xSlewRateLimiter.calculate(-MathUtil.applyDeadband(controller.getRawAxis(1), .1)) * DrivetrainSubsystem.maxSpeedMeters;
        final double ySpeed = ySlewRateLimiter.calculate(-MathUtil.applyDeadband(controller.getRawAxis(0),.1)) * DrivetrainSubsystem.maxSpeedMeters;
        calculateDesiredAngle();

        double rot = anglecontroller.calculate(drivetrain.getPose().getRotation().getRadians(), angle);
        boolean calibrate = controller.getRawButton(1);

        System.out.println("desired angle: " + ConversionUtil.radiansToDegrees(angle));
        System.out.println("current angle: " + drivetrain.getPose().getRotation().getDegrees());

        drivetrain.drive(-xSpeed, -ySpeed, rot, true, calibrate);
    }

    private void calculateDesiredAngle() {
        final double oppLeg = MathUtil.applyDeadband(controller.getRawAxis(4), .1);
        final double adjLeg = -MathUtil.applyDeadband(controller.getRawAxis(5), .1);
        System.out.println("oppLeg: " + oppLeg);
        System.out.println("adjLeg: " + adjLeg);
        if (drivetrain.getIsAngleLocked()) {
            angle = drivetrain.getPose().getRotation().getRadians();
        }else {
            if (oppLeg != 0) {
                if (adjLeg > 0) {
                    angle = -Math.atan(oppLeg/adjLeg);
                } else if (adjLeg < 0) {
                    angle = -Math.atan(oppLeg/adjLeg) - Math.PI;
                } else {
                    if (oppLeg > 0) {
                        angle = 3*Math.PI/2;
                    }else {
                        angle = Math.PI/2;
                    }
                }
            }else if (adjLeg > 0) {
                angle =  Math.atan(0);
            }else if (adjLeg < 0){
                angle =  Math.PI;
            }
        }
    }

    @Override
    protected boolean isFinishedDelegate() {
        return false;
    }
}
