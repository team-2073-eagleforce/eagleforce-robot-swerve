package com.team2073.robot.Command.Drive;

import com.team2073.common.command.AbstractLoggingCommand;
import com.team2073.robot.ApplicationContext;
import com.team2073.robot.Subsystems.Drive.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {
    private ApplicationContext appCTX = ApplicationContext.getInstance();

    private Joystick controller;
    private DrivetrainSubsystem drivetrain;

    public DriveCommand(DrivetrainSubsystem drivetrain, Joystick controller) {
        this.drivetrain = drivetrain;

        this.controller = controller;
    }

    @Override
    public void execute() {
        final double xSpeed = -controller.getRawAxis(0) * DrivetrainSubsystem.maxSpeed;
        final double ySpeed = -controller.getRawAxis(1) * DrivetrainSubsystem.maxSpeed;
        final double rot = -controller.getRawAxis(3) * DrivetrainSubsystem.maxAngularSpeed;

        boolean calibrate = controller.getRawButton(0);

        drivetrain.drive(xSpeed, ySpeed, rot, true, calibrate);
    }

}
