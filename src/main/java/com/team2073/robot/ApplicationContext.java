package com.team2073.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.team2073.common.CommonConstants;
import com.team2073.robot.Subsystems.Drive.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class ApplicationContext {
    private static ApplicationContext instance;

    //Front Left Module
    private WPI_TalonFX frontLeftSteerMotor;
    private WPI_TalonFX frontLeftDriveMotor;
    private CANCoder frontLeftEncoder;

    //Front Rigth Module
    private WPI_TalonFX frontRightSteerMotor;
    private WPI_TalonFX frontRightDriveMotor;
    private CANCoder frontRightEncoder;

    //Back Left Module
    private WPI_TalonFX backLeftSteerMotor;
    private WPI_TalonFX backLeftDriveMotor;
    private CANCoder backLeftEncoder;

    //Back Right Module
    private WPI_TalonFX backRightSteerMotor;
    private WPI_TalonFX backRightDriveMotor;
    private CANCoder backRightEncoder;

    private PigeonIMU gyro;

    private DrivetrainSubsystem drivetrainSubsystem;

    Field2d m_fieldSim;

    private Joystick controller;

    private AppConstants constants = AppConstants.getInstance();

    public static ApplicationContext getInstance(){
        if (instance == null){
            instance = new ApplicationContext();
        }
        return instance;
    }

    public WPI_TalonFX getFrontLeftSteerMotor() {
        if(frontLeftSteerMotor == null) {
            frontLeftSteerMotor = new WPI_TalonFX(constants.FRONT_LEFT_STEER_PORT);
        }
        return frontLeftSteerMotor;
    }

    public WPI_TalonFX getFrontRightSteerMotor() {
        if(frontRightSteerMotor == null) {
            frontRightSteerMotor = new WPI_TalonFX(constants.FRONT_RIGHT_STEER_PORT);
        }
        return frontRightSteerMotor;
    }

    public WPI_TalonFX getBackLeftSteerMotor() {
        if(backLeftSteerMotor == null) {
            backLeftSteerMotor = new WPI_TalonFX(constants.BACK_LEFT_STEER_PORT);
        }
        return backLeftSteerMotor;
    }

    public WPI_TalonFX getBackRightSteerMotor() {
        if(backRightSteerMotor == null) {
            backRightSteerMotor = new WPI_TalonFX(constants.BACK_RIGHT_STEER_PORT);
        }
        return backRightSteerMotor;
    }

    public WPI_TalonFX getFrontLeftDriveMotor() {
        if(frontLeftDriveMotor == null) {
            frontLeftDriveMotor = new WPI_TalonFX(constants.FRONT_LEFT_DRIVE_PORT);
        }
        return frontLeftDriveMotor;
    }

    public WPI_TalonFX getFrontRightDriveMotor() {
        if(frontRightDriveMotor == null) {
            frontRightDriveMotor = new WPI_TalonFX(constants.FRONT_RIGHT_DRIVE_PORT);
        }
        return frontRightDriveMotor;
    }

    public WPI_TalonFX getBackLeftDriveMotor() {
        if(backLeftDriveMotor == null) {
            backLeftDriveMotor = new WPI_TalonFX(constants.BACK_LEFT_DRIVE_PORT);
        }
        return backLeftDriveMotor;
    }

    public WPI_TalonFX getBackRightDriveMotor() {
        if(backRightDriveMotor == null) {
            backRightDriveMotor = new WPI_TalonFX(constants.BACK_RIGHT_DRIVE_PORT);
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

    public Field2d getM_fieldSim() {
        if (m_fieldSim == null) {
            m_fieldSim = new Field2d();
        }
        return m_fieldSim;
    }
}
