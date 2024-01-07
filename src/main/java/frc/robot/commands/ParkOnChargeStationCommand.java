// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.Trajectories.ParkOnChargeStaionTrajectory;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ParkOnChargeStationCommand extends SequentialCommandGroup {
  /** Creates a new ParkOnChargeStationCommand. */
  public ParkOnChargeStationCommand(DriveSubsystem m_robotDrive, ProfiledPIDController thetaController) {

    m_robotDrive.resetOdometry(ParkOnChargeStaionTrajectory.trajectory.getInitialPose());
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(
        () -> m_robotDrive.resetEncoders()
      ),
      new InstantCommand(
        () ->  m_robotDrive.switchBrake()
      ),
      new SwerveControllerCommand(
        ParkOnChargeStaionTrajectory.trajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive
      ), 
      new InstantCommand(
        () -> m_robotDrive.drive(1,0, 0, 0)
      )
    );
  }
}
