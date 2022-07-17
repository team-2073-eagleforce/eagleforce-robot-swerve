package com.team2073.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.team2073.common.CommonConstants;
import com.team2073.robot.Subsystems.Drive.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class ApplicationContext {
    private static ApplicationContext instance;

    //Front Left Module
    private TalonFX frontLeftSteerMotor;
    private TalonFX frontLeftDriveMotor;
    private CANCoder frontLeftEncoder;

    //Front Rigth Module
    private TalonFX frontRightSteerMotor;
    private TalonFX frontRightDriveMotor;
    private CANCoder frontRightEncoder;

    //Back Left Module
    private TalonFX backLeftSteerMotor;
    private TalonFX backLeftDriveMotor;
    private CANCoder backLeftEncoder;

    //Back Right Module
    private TalonFX backRightSteerMotor;
    private TalonFX backRightDriveMotor;
    private CANCoder backRightEncoder;

    private PigeonIMU gyro;

    private DrivetrainSubsystem drivetrainSubsystem;

    private Joystick controller;

    private AppConstants constants = AppConstants.getInstance();

    public static ApplicationContext getInstance(){
        if (instance == null){
            instance = new ApplicationContext();
        }
        return instance;
    }

    public TalonFX getFrontLeftSteerMotor() {
        if(frontLeftSteerMotor == null) {
            frontLeftSteerMotor = new TalonFX(constants.FRONT_LEFT_STEER_PORT);
        }
        return frontLeftSteerMotor;
    }

    public TalonFX getFrontRightSteerMotor() {
        if(frontRightSteerMotor == null) {
            frontRightSteerMotor = new TalonFX(constants.FRONT_RIGHT_STEER_PORT);
        }
        return frontRightSteerMotor;
    }

    public TalonFX getBackLeftSteerMotor() {
        if(backLeftSteerMotor == null) {
            backLeftSteerMotor = new TalonFX(constants.BACK_LEFT_STEER_PORT);
        }
        return backLeftSteerMotor;
    }

    public TalonFX getBackRightSteerMotor() {
        if(backRightSteerMotor == null) {
            backRightSteerMotor = new TalonFX(constants.BACK_RIGHT_STEER_PORT);
        }
        return backRightSteerMotor;
    }

    public TalonFX getFrontLeftDriveMotor() {
        if(frontLeftDriveMotor == null) {
            frontLeftDriveMotor = new TalonFX(constants.FRONT_LEFT_DRIVE_PORT);
        }
        return frontLeftDriveMotor;
    }

    public TalonFX getFrontRightDriveMotor() {
        if(frontRightDriveMotor == null) {
            frontRightDriveMotor = new TalonFX(constants.FRONT_RIGHT_DRIVE_PORT);
        }
        return frontRightDriveMotor;
    }

    public TalonFX getBackLeftDriveMotor() {
        if(backLeftDriveMotor == null) {
            backLeftDriveMotor = new TalonFX(constants.BACK_LEFT_DRIVE_PORT);
        }
        return backLeftDriveMotor;
    }

    public TalonFX getBackRightDriveMotor() {
        if(backRightDriveMotor == null) {
            backRightDriveMotor = new TalonFX(constants.BACK_RIGHT_DRIVE_PORT);
        }
        return backRightDriveMotor;
    }

    public CANCoder getFrontLeftEncoder() {
        if(frontLeftEncoder == null) {
            frontLeftEncoder = new CANCoder(constants.FRONT_LEFT_ENCODER_PORT);
        }
        return frontLeftEncoder;
    }

    public CANCoder getFrontRightEncoder() {
        if (frontRightEncoder == null) {
            frontRightEncoder = new CANCoder(constants.FRONT_RIGHT_ENCODER_PORT);
        }
        return frontRightEncoder;
    }

    public CANCoder getBackLeftEncoder() {
        if (backLeftEncoder == null) {
            backLeftEncoder = new CANCoder(constants.BACK_LEFT_ENCODER_PORT);
        }
        return backLeftEncoder;
    }

    public CANCoder getBackRightEncoder() {
        if (backRightEncoder ==  null) {
            backRightEncoder = new CANCoder(constants.BACK_RIGHT_ENCODER_PORT);
        }
        return backRightEncoder;
    }

    public DrivetrainSubsystem getDrivetrainSubsystem() {
        if (drivetrainSubsystem == null) {
            drivetrainSubsystem = new DrivetrainSubsystem();
        }
        return drivetrainSubsystem;
    }

    public Joystick getController(){
        if (controller == null) {
            controller = new Joystick(0);
        }
        return controller;
    }

    public PigeonIMU getGyro() {
        if (gyro == null){
            gyro = new PigeonIMU(17);
        }
        return gyro;
    }
}
