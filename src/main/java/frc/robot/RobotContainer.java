// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorToX;
import frc.robot.commands.ElevatorUp;
import frc.robot.commands.ExtenderIn;
import frc.robot.commands.ExtenderOut;
import frc.robot.commands.HighGoal;
import frc.robot.commands.Pickup;
import frc.robot.commands.drivePostion;
import frc.robot.commands.HighGoalSeq;
import frc.robot.commands.MidGoal;
import frc.robot.commands.extenderToX;
import frc.robot.commands.intakeWithJoystick;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private static final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private static final ExtenderSubsystem m_extender = new ExtenderSubsystem();
  private static final IntakeSubsystem m_intake = new IntakeSubsystem();
  TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);
  private Trajectory exampleTrajectory;


  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  XboxController m_manipulator = new XboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 0) ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(2, 0, new Rotation2d(0)),
            config);
    // Configure the button bindings
    configureButtonBindings();
    CameraServer.startAutomaticCapture();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new DriveWithJoystick(m_driverController, m_robotDrive));
    //m_elevator.setDefaultCommand(new ElevatorStop(m_elevator));
    m_intake.setDefaultCommand(new intakeWithJoystick(m_intake, m_manipulator));

  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, 3).onTrue(
        new InstantCommand(
            () ->
                m_robotDrive.resetEncoders())
    );
    new JoystickButton(m_driverController, 7).onTrue(
        new InstantCommand(
            () ->
                m_robotDrive.switchDrive())
    );
    new JoystickButton(m_driverController, 11).onTrue(
      new InstantCommand(
        () ->
        m_robotDrive.switchBrake()
      )
    );
    new JoystickButton(m_manipulator, 7).whileTrue(new ExtenderIn(m_extender));
    new JoystickButton(m_manipulator, 8).whileTrue(new ExtenderOut(m_extender));
    new JoystickButton(m_manipulator, 6).whileTrue(new ElevatorUp(m_elevator));
    new JoystickButton(m_manipulator, 5).whileTrue(new ElevatorDown(m_elevator));
    new JoystickButton(m_manipulator, 3).onTrue(new drivePostion(m_elevator, m_extender));
    new JoystickButton(m_manipulator, 1).onTrue(new Pickup(m_elevator, m_extender));
    new JoystickButton(m_manipulator, 4).onTrue(new HighGoal(m_elevator, m_extender));
    new JoystickButton(m_manipulator, 2).onTrue(new MidGoal(m_elevator, m_extender));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    

    // An example trajectory to follow.  All units in meters.
    

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(1,0, 0, 0));
  }
}
