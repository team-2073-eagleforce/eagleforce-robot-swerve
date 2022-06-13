package com.team2073.robot.Subsystems.Drive;

import com.team2073.robot.AppConstants;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import static com.team2073.robot.AppConstants.DriveConstants.*;
import static com.team2073.robot.AppConstants.AutoConstants.*;

public class DrivetrainSubsystem extends SubsystemBase {
    ApplicationContext appCTX = ApplicationContext.getInstance();

    public static double maxSpeed = Units.feetToMeters(13.6);
    public static double maxAngularSpeed = Math.PI;

    public static double frontLeftOffset = 10;
    public static double frontRightOffset = 10;
    public static double backLeftOffset = 10;
    public static double backRightOffset = 10;

    public static PigeonIMU gyro = new PigeonIMU(51);




    private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(getHeading()));

    private SwerveModule[] modules = new SwerveModule[] {
            new SwerveModule(appCTX.getFrontLeftDriveMotor(), appCTX.getFrontLeftSteerMotor(), appCTX.getFrontLeftEncoder(), Rotation2d.fromDegrees(frontLeftOffset)),
            new SwerveModule(appCTX.getFrontRightDriveMotor(), appCTX.getFrontRightSteerMotor(), appCTX.getFrontRightEncoder(), Rotation2d.fromDegrees(frontRightOffset)),
            new SwerveModule(appCTX.getBackLeftDriveMotor(), appCTX.getBackLeftSteerMotor(), appCTX.getBackLeftEncoder(), Rotation2d.fromDegrees(backLeftOffset)),
            new SwerveModule(appCTX.getBackRightDriveMotor(), appCTX.getBackRightSteerMotor(), appCTX.getBackRightEncoder(), Rotation2d.fromDegrees(backRightOffset))
    };

    public DrivetrainSubsystem(){
    }

    @Override
    public void periodic() {
        m_odometry.update(
                Rotation2d.fromDegrees(getHeading()),modules[0].getState(),modules[1].getState(),modules[2].getState(),modules[3].getState());
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean calibrateGyro) {
        if (calibrateGyro) {
            gyro.setFusedHeading(0);
        }

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(-gyro.getFusedHeading()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeed);
        for (int i = 0; i < modules.length; i++) {
            SwerveModule module = modules[i];
            SwerveModuleState state = states[i];

            module.setState(state);
        }
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    public void zeroHeading() {
        gyro.setFusedHeading(0);
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getFusedHeading(),360) * (kGyroReversed ? -1.0 : 1.0);
    }

//    public double getTurnRate() {
//        return gyro * (kGyroReversed ? -1.0 : 1.0);
//    }

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
                new Pose2d(0,0,new Rotation2d(0)),
                List.of(new Translation2d(1,1), new Translation2d(2,-1)),
                new Pose2d(3,0,new Rotation2d(0)),
                config.setReversed(false))),
        TEST_PATH_2(TrajectoryGenerator.generateTrajectory(
                List.of(
                        new Pose2d(0,0, new Rotation2d(0)),
                        new Pose2d(1,0, new Rotation2d(0))),
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