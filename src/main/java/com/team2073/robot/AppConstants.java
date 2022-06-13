package com.team2073.robot;

import com.team2073.robot.Subsystems.Drive.DrivetrainSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class AppConstants {
    private static AppConstants instance;

    //Front Left Module
    public int FRONT_LEFT_DRIVE_PORT = 11;
    public int FRONT_LEFT_STEER_PORT = 12;
    public int FRONT_LEFT_ENCODER_PORT = 1;

    //Front Right Module
    public int FRONT_RIGHT_DRIVE_PORT = 41;
    public int FRONT_RIGHT_STEER_PORT = 42;
    public int FRONT_RIGHT_ENCODER_PORT = 4;

    //Back Left Module
    public int BACK_LEFT_DRIVE_PORT = 21;
    public int BACK_LEFT_STEER_PORT = 22;
    public int BACK_LEFT_ENCODER_PORT = 2;

    //Back Right Module
    public int BACK_RIGHT_DRIVE_PORT = 31;
    public int BACK_RIGHT_STEER_PORT = 32;
    public int BACK_RIGHT_ENCODER_PORT = 3;


    private AppConstants(){}

    public static AppConstants getInstance(){
        if (instance == null){
            instance = new AppConstants();
        }
        return instance;
    }

    public static final class DriveConstants {
        public static final boolean kGyroReversed = true;
        //Change later
        public static final double GEAR_RATIO = 1d;

        //Change later
        public static final double ksVolts = 1;
        //Change later
        public static final double kvVoltSecondsPerMeter = 0.8;
        //Change later
        public static final double kaVoltSecondsSquaredPerMeter = 0.15;

        public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(
                        Units.inchesToMeters(10),
                        Units.inchesToMeters(10)
                ),
                new Translation2d(
                        Units.inchesToMeters(10),
                        Units.inchesToMeters(-10)
                ),
                new Translation2d(
                        Units.inchesToMeters(-10),
                        Units.inchesToMeters(10)
                ),
                new Translation2d(
                        Units.inchesToMeters(-10),
                        Units.inchesToMeters(-10)
                )
        );

        public static TrajectoryConfig config =
                new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(kinematics);
    }

    public static final class AutoConstants {
        //Change later
        public static final double kMaxSpeedMetersPerSecond = 3;
        //Change later
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        //Change later
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        //Change later
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        //Change later
        public static final double kPXController = 1;
        //Change later
        public static final double kPYController = 1;
        //Change later
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
