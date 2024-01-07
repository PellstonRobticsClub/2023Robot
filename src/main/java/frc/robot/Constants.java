// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kRearLeftDriveMotorPort = 3;
    public static final int kFrontRightDriveMotorPort = 2;
    public static final int kRearRightDriveMotorPort = 4;

    public static final int kFrontLeftTurningMotorPort = 11;
    public static final int kRearLeftTurningMotorPort = 13;
    public static final int kFrontRightTurningMotorPort = 12;
    public static final int kRearRightTurningMotorPort = 14;

    public static final int kRearLeftTurningEncoderPort = 2;
    public static final int kFrontRightTurningEncoderPort = 1;
    public static final int kRearRightTurningEncoderPort = 3;
    public static final int kFrontLeftTurningEncoderPort = 0;

    public static final boolean kFrontLeftTurningMotorReversed = true;
    public static final boolean kRearLeftTurningMotorReversed = true;
    public static final boolean kFrontRightTurningMotorReversed = true;
    public static final boolean kRearRightTurningMotorReversed = true;

    public static final boolean kFrontLeftDriveMotorReversed = false;
    public static final boolean kRearLeftDriveMotorReversed = false;
    public static final boolean kFrontRightDriveMotorReversed = false;
    public static final boolean kRearRightDriveMotorReversed = false;

    public static final double kFrontLeftAnalogEncoderOffset = 4.0455;
    public static final double kRearLeftAnalogEncoderOffset = 2.8235;
    public static final double kFrontRightAnalogEncoderOffset = 3.7955;
    public static final double kRearRightAnalogEncoderOffset = .640;

    // Distance between centers of right and left wheels on robot in meters
    public static final double kTrackWidth = 0.42545;
    
    // Distance between front and back wheels on robot in meters
    public static final double kWheelBase = 0.62865;
    
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 5;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedDegreesPerSecond = 360;
    public static final double kMaxModuleAngularAccelerationDegreesPerSecondSquared = 360;

    public static final int kEncoderCPR = 1;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveGearRatio = 1/6.75;
    public static final double kDriveEncoderDistancePerPulse = 
        // Assumes the encoders are directly mounted on the wheel shafts
       (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR* kDriveGearRatio * 1.1875;

    private static final double kturningEncoderCountsPerRevolution = 1;
    private static final double kTurningEncoderGearRatio = .04667;
    public static final double kTurningEncoderDistancePerPulse =
        kturningEncoderCountsPerRevolution * kTurningEncoderGearRatio * (2 * Math.PI);

    public static final double kPModuleTurningController = .5;

    public static final double kPModuleDriveController = .75;
    public static double kPModuleDriveSlewRate = 1;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = .7;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
  public static final class ElevatorConstants{

    public static final int potentiometerAnologID = 6;
    public static final int MotorID = 3;
    public static final double Speed = 1;

  }

  public static final class ExtenderConstants{

    public static final int absoluteEncoderPort = 0;
    public static final int[] encoderPorts = {1,2};
    public static final int MotorID = 2;
    public static final double Speed = .3;

  }

  public static final class IntakeConstants{
    public static final int motorID = 1;

  }
  public static final class AutonConfig{
    public static final TrajectoryConfig trajectoryConfig =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    
  }
}
