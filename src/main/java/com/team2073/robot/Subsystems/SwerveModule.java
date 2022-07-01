package com.team2073.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.team2073.common.util.ConversionUtil;
import com.team2073.robot.AppConstants;
import com.team2073.robot.MotorSimProfiles.PhysicsSim;
import com.team2073.robot.Subsystems.Drive.DrivetrainSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import static com.team2073.robot.AppConstants.DriveConstants.*;

public class SwerveModule {

    private static final double kDriveP = 1;
    private static final double kDriveI = 0.01;
    private static final double kDriveD = 0.1;
    private static final double kDriveF = 0.2;

    private static final double kAngleP = 1;
    private static final double kAngleI = 0.0;
    private static final double kAngleD = 0.0;

//    private static double encoderTicksPerRotation = 26214.4;
    private static double encoderTicksPerRotation = 4096;

    private TalonFX driveMotor;
    private TalonFX steerMotor;
    private CANCoder canCoder;
    private EncoderSim m_driveEncoderSim;
    private TalonFXSensorCollection m_turningEncoderSim;

    private Rotation2d offset;

    private double m_simTurnEncoderDistance;
    private double m_simDriveEncoderDistance;

    public SwerveModule(TalonFX driveMotor, TalonFX steerMotor, CANCoder canCoder, Rotation2d offset){
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.canCoder = canCoder;
        this.offset = offset;

        PhysicsSim.getInstance().addTalonFX(driveMotor, 0.5, 5100);
        PhysicsSim.getInstance().addTalonFX(steerMotor, 0.5, 5100);

        this.m_turningEncoderSim = steerMotor.getSensorCollection();

        TalonFXConfiguration angleTalonFXConfiguration = new TalonFXConfiguration();

        angleTalonFXConfiguration.slot0.kP = kAngleP;
        angleTalonFXConfiguration.slot0.kI = kAngleI;
        angleTalonFXConfiguration.slot0.kD = kAngleD;

        angleTalonFXConfiguration.remoteFilter0.remoteSensorDeviceID = canCoder.getDeviceID();
        angleTalonFXConfiguration.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;

        angleTalonFXConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        steerMotor.configAllSettings(angleTalonFXConfiguration);
        steerMotor.setNeutralMode(NeutralMode.Brake);

        TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();

        driveTalonFXConfiguration.slot0.kP = kDriveP;
        driveTalonFXConfiguration.slot0.kI = kDriveI;
        driveTalonFXConfiguration.slot0.kD = kDriveD;
        driveTalonFXConfiguration.slot0.kF = kDriveF;

        driveMotor.configAllSettings(driveTalonFXConfiguration);
        driveMotor.setNeutralMode(NeutralMode.Brake);

        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.magnetOffsetDegrees = offset.getDegrees();
        canCoder.configAllSettings(canCoderConfiguration);
    }

    private final FlywheelSim m_turnMotorSim = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian),
            kTurnMotorGearbox,
            kTurnGearRatio
    );

    private final FlywheelSim m_driveMotorSim = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(AppConstants.DriveConstants.kvVoltSecondsPerMeter, AppConstants.DriveConstants.kaVoltSecondsSquaredPerMeter),
            kDriveMotorGearbox,
            kDriveGearRatio
    );

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(canCoder.getAbsolutePosition());
    }

    public double getRawAngle() {
        return canCoder.getAbsolutePosition();
    }

    public static SwerveModuleState optimize(
            SwerveModuleState desiredState, Rotation2d currentAngle) {
        Rotation2d delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > 90.0) {
            return new SwerveModuleState(
                    -desiredState.speedMetersPerSecond,
                    desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
        } else {
            return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
    }

    public void setState(SwerveModuleState desiredState) {
        Rotation2d currentAngle = getAngle();
        SwerveModuleState state = optimize(desiredState, currentAngle);

        Rotation2d rotationDelta = state.angle.minus(currentAngle);

        double positionDelta = (rotationDelta.getDegrees()/360) * encoderTicksPerRotation;
        double currentPosition = canCoder.getPosition() / canCoder.configGetFeedbackCoefficient();
        double desiredPosition = currentPosition + positionDelta;

        steerMotor.set(TalonFXControlMode.Position, desiredPosition);

        double feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond);

        driveMotor.set(TalonFXControlMode.PercentOutput, feetPerSecond/ DrivetrainSubsystem.maxSpeed);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(((driveMotor.getSensorCollection().getIntegratedSensorVelocity() * GEAR_RATIO * 10)/ 2048 * ConversionUtil.inchesToMeters(4 * Math.PI)), getAngle());
//        return new SwerveModuleState(((driveMotor.getSelectedSensorVelocity() * kDriveGearRatio * 10)/ 2048 * ConversionUtil.inchesToMeters(4 * Math.PI)), getAngle());
    }

    /**
     *   REV     | 1 MIN  |  | 1 Sec    |    10     |  2048 ticks
     * ( --- )   ( ------ )  (  ------  )  ( ---- )  ( --------- )
     *   MIN    | 60 Sec |  | 1000 ms  |      1     |     1 REV
     * @param RPM
     * @return Native Falcon Units for Velocity from RPM
     */
    public double convertRPMtoFalconUnitsVelocity(double RPM) {
        return RPM / 60 / 1000 * 10 * 2048;
    }

    /** Simulate the SwerveModule */
    public void simulationPeriodic(double dt) {
        m_turnMotorSim.setInputVoltage(steerMotor.getMotorOutputPercent() / AppConstants.AutoConstants.kMaxAngularSpeedRadiansPerSecond * RobotController.getBatteryVoltage());
        m_driveMotorSim.setInputVoltage(driveMotor.getMotorOutputPercent() / AppConstants.AutoConstants.kMaxSpeedMetersPerSecond * RobotController.getBatteryVoltage());
        System.out.println(steerMotor.getMotorOutputPercent());
        m_turnMotorSim.update(dt);
        m_driveMotorSim.update(dt);

        // Calculate distance traveled using RPM * dt
        m_simTurnEncoderDistance += m_turnMotorSim.getAngularVelocityRPM() * dt;
//        m_simTurnEncoderDistance *= 360;
//        m_simTurnEncoderDistance = m_simTurnEncoderDistance > 360 ? 0 : m_simTurnEncoderDistance;

        canCoder.getSimCollection().setRawPosition((int) m_simTurnEncoderDistance * 5);
//        System.out.println(m_turnMotorSim.getAngularVelocityRPM());
        canCoder.setPositionToAbsolute();
        canCoder.getSimCollection().setVelocity((int) m_turnMotorSim.getAngularVelocityRPM() / 60  / 1000 * 10);
//
//        steerMotor.getSimCollection().setIntegratedSensorRawPosition((int) m_simTurnEncoderDistance);
//        steerMotor.getSimCollection().setIntegratedSensorVelocity((int) m_turnMotorSim.getAngularVelocityRadPerSec());

//        m_driveEncoderSim.setRate(m_driveMotorSim.getAngularVelocityRadPerSec());

        m_simDriveEncoderDistance += convertRPMtoFalconUnitsVelocity(m_driveMotorSim.getAngularVelocityRPM()) * dt;
        steerMotor.getSimCollection().setIntegratedSensorVelocity((int) convertRPMtoFalconUnitsVelocity( m_driveMotorSim.getAngularVelocityRPM()));
        steerMotor.getSimCollection().setIntegratedSensorRawPosition((int) m_simDriveEncoderDistance);

//        m_simTurnEncoderDistance += m_turnMotorSim.getAngularVelocityRadPerSec() * 0.02;
//        m_simDriveEncoderDistance += m_driveMotorSim.getAngularVelocityRadPerSec() * 0.02;
//
//        steerMotor.getSimCollection().setIntegratedSensorRawPosition((int) (m_simTurnEncoderDistance / kTurningEncoderDistancePerPulse));
//        steerMotor.getSimCollection().setIntegratedSensorVelocity( (int) (m_turnMotorSim.getAngularVelocityRadPerSec() / kTurningEncoderDistancePerPulse / 10));
//        driveMotor.getSimCollection().setIntegratedSensorRawPosition( (int) (m_simDriveEncoderDistance / kDriveSimEncoderDistancePerPulse));
//        driveMotor.getSimCollection().setIntegratedSensorVelocity( (int) (m_driveMotorSim.getAngularVelocityRadPerSec() / kDriveSimEncoderDistancePerPulse / 10));
    }
}
