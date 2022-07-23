package com.team2073.robot;

import com.team2073.robot.Subsystems.Drive.DrivetrainSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class AppConstants {
    private static AppConstants instance;

    //Front Left Module
    public int FRONT_LEFT_DRIVE_PORT = 22;
    public int FRONT_LEFT_STEER_PORT = 21;
    public int FRONT_LEFT_ENCODER_PORT = 2;

    //Front Right Module
    public int FRONT_RIGHT_DRIVE_PORT = 32;
    public int FRONT_RIGHT_STEER_PORT = 31;
    public int FRONT_RIGHT_ENCODER_PORT = 3;

    //Back Left Module
    public int BACK_LEFT_DRIVE_PORT = 12;
    public int BACK_LEFT_STEER_PORT = 11;
    public int BACK_LEFT_ENCODER_PORT = 1;

    //Back Right Module
    public int BACK_RIGHT_DRIVE_PORT = 42;
    public int BACK_RIGHT_STEER_PORT = 41;
    public int BACK_RIGHT_ENCODER_PORT = 4;


    private AppConstants(){}

    public static AppConstants getInstance(){
        if (instance == null){
            instance = new AppConstants();
        }
        return instance;
    }

    public static final class DriveConstants {
        public static final boolean kGyroReversed = false;
        //@TODO Change later
        public static final double GEAR_RATIO = 6.83d;

        //@TODO Change later
        public static final double ksVolts = 0.10726;
        //@TODO Change later
        public static final double kvVoltSecondsPerMeter = 2.1247;
        //@TODO Change later
        public static final double kaVoltSecondsSquaredPerMeter = 0.19858;

        public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(Units.inchesToMeters(12), Units.inchesToMeters(12)),
                new Translation2d(Units.inchesToMeters(12), Units.inchesToMeters(-12)),
                new Translation2d(Units.inchesToMeters(-12), Units.inchesToMeters(12)),
                new Translation2d(Units.inchesToMeters(-12), Units.inchesToMeters(-12)));

        public static TrajectoryConfig config =
                new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(kinematics);
    }

    public static final class AutoConstants {

        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2.5;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        //@TODO Change later
        public static final double kPXController = 0.1;
        //@TODO Change later
        public static final double kPYController = 0.1;
        //@TODO Change later
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
