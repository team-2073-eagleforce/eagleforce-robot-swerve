package com.team2073.robot.Subsystems.Drive;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.team2073.common.periodic.AsyncPeriodicRunnable;
import com.team2073.robot.ApplicationContext;
import com.team2073.robot.Subsystems.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import static com.team2073.robot.AppConstants.DriveConstants.*;
import static com.team2073.robot.AppConstants.AutoConstants.*;

public class DrivetrainSubsystem extends SubsystemBase implements AsyncPeriodicRunnable {
    ApplicationContext appCTX = ApplicationContext.getInstance();

    public static double maxSpeedMeters = Units.feetToMeters(13.6);
    public static double maxSpeedFeet = 13.6;
    public static double maxAngularSpeed = 2*Math.PI;

    private ShuffleboardTab tuning = Shuffleboard.getTab("tuning");

    public static double frontLeftOffset = 269.25;
    public static double frontRightOffset = 310;
    public static double backLeftOffset = 260;
    public static double backRightOffset = 319;

    public PigeonIMU gyro = appCTX.getGyro();




    private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(gyro.getFusedHeading()));

    private SwerveModule[] modules = new SwerveModule[] {
            new SwerveModule(appCTX.getFrontLeftDriveMotor(), appCTX.getFrontLeftSteerMotor(), appCTX.getFrontLeftEncoder(), frontLeftOffset),
            new SwerveModule(appCTX.getFrontRightDriveMotor(), appCTX.getFrontRightSteerMotor(), appCTX.getFrontRightEncoder(), frontRightOffset),
            new SwerveModule(appCTX.getBackLeftDriveMotor(), appCTX.getBackLeftSteerMotor(), appCTX.getBackLeftEncoder(), backLeftOffset),
            new SwerveModule(appCTX.getBackRightDriveMotor(), appCTX.getBackRightSteerMotor(), appCTX.getBackRightEncoder(), backRightOffset)
    };

    public DrivetrainSubsystem(){
//        frontLeft = tuning.addNumber("front left", modules[0]::getDegreeHeading);
//        frontRight = tuning.addNumber("front right", modules[1]::getDegreeHeading);
//        backLeft = tuning.addNumber("back left", modules[2]::getDegreeHeading);
//        backRight = tuning.addNumber("back right", modules[3]::getDegreeHeading);
    }


    @Override
    public void onPeriodicAsync() {
        m_odometry.update(Rotation2d.fromDegrees(gyro.getFusedHeading()),modules[0].getState(),modules[1].getState(),modules[2].getState(),modules[3].getState());
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean calibrateGyro) {
        if (calibrateGyro) {
            gyro.setFusedHeading(0);
        }

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(gyro.getFusedHeading()))
                    : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeedMeters);
        for (int i = 0; i < modules.length; i++) {
            SwerveModule module = modules[i];
            SwerveModuleState state = states[i];

            SmartDashboard.putNumber(String.valueOf(i), modules[i].getRawAngle());
            SmartDashboard.putNumber("state: " + String.valueOf(i), states[i].angle.getDegrees());
            SmartDashboard.putNumber("gyro Angle", gyro.getFusedHeading());

            module.setState(state);
        }
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(gyro.getFusedHeading()));
    }

    public void zeroHeading() {
        gyro.setFusedHeading(0);
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getFusedHeading(),360);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, kMaxSpeedMetersPerSecond);
        for (int i = 0; i < modules.length; i++) {
            SwerveModule module = modules[i];
            module.setState(desiredStates[i]);
        }
    }

    public enum AUTO_PATHS_SWERVE {
        TEST_PATH(TrajectoryGenerator.generateTrajectory(
                List.of(
                        new Pose2d(0,0, Rotation2d.fromDegrees(0)),
                        new Pose2d(1,1, Rotation2d.fromDegrees(30)),
                        new Pose2d(2,-1, Rotation2d.fromDegrees(60)),
                        new Pose2d(3,0, Rotation2d.fromDegrees(90))),
                config.setReversed(false))),
        TEST_PATH_2(TrajectoryGenerator.generateTrajectory(
                List.of(
                        new Pose2d(0,0, Rotation2d.fromDegrees(0d)),
                        new Pose2d(Units.inchesToMeters(36d),0, Rotation2d.fromDegrees(0))),
                config.setReversed(false)));

        private Trajectory traj;

        AUTO_PATHS_SWERVE(Trajectory traj) {
            this.traj = traj;
        }

        public Trajectory getTraj() {
            return traj;
        }
    }
}
