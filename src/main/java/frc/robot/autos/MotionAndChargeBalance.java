// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.SelfBalanceCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MotionAndChargeBalance extends SequentialCommandGroup {

  
  /** Creates a new MotionAndChargeBalance. */
  public MotionAndChargeBalance(RobotContainer robot) {
    PathPlannerTrajectory trajectory1 =
    PathPlanner.generatePath(
      new PathConstraints(2, 1.5), 
      new PathPoint(
        new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)),
        Rotation2d.fromDegrees(0),
        Rotation2d.fromDegrees(0)), 
        new PathPoint(
          new Translation2d(Units.inchesToMeters(-195), Units.inchesToMeters(0)),
          Rotation2d.fromDegrees(0),
          Rotation2d.fromDegrees(0)));

          PathPlannerTrajectory trajectory2 =
    PathPlanner.generatePath(
      new PathConstraints(1.25, 1), 
      new PathPoint(
        new Translation2d(Units.inchesToMeters(-195), Units.inchesToMeters(0)),
        Rotation2d.fromDegrees(0),
        Rotation2d.fromDegrees(0)), 
        new PathPoint(
          new Translation2d(Units.inchesToMeters(-120), Units.inchesToMeters(0)),
          Rotation2d.fromDegrees(0),
          Rotation2d.fromDegrees(0)));

      PathPlannerTrajectory trajectory3 = PathPlanner.loadPath("New Path", new PathConstraints(.5, .5));
      SmartDashboard.putNumber("traj time", trajectory1.getTotalTimeSeconds());

      PPSwerveControllerCommand swerveControllerCommand1 =
        new PPSwerveControllerCommand(
          trajectory1,
          robot.m_robotDrive::getPose,
          Constants.DriveConstants.kDriveKinematics,
          new PIDController(Constants.AutoConstants.kPXController, 0, 0),
          new PIDController(Constants.AutoConstants.kPYController, 0, 0),
          new PIDController(Constants.AutoConstants.kPThetaController, 0, 0),
          robot.m_robotDrive::setModuleStates,
          robot.m_robotDrive);

          PPSwerveControllerCommand swerveControllerCommand2 =
        new PPSwerveControllerCommand(
          trajectory2,
          robot.m_robotDrive::getPose,
          Constants.DriveConstants.kDriveKinematics,
          new PIDController(Constants.AutoConstants.kPXController, 0, 0),
          new PIDController(Constants.AutoConstants.kPYController, 0, 0),
          new PIDController(Constants.AutoConstants.kPThetaController, 0, 0),
          robot.m_robotDrive::setModuleStates,
          robot.m_robotDrive);
             // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(
        ()->
          robot.m_robotDrive.resetOdometry(
            new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0))
          )
      
      ),
      swerveControllerCommand1,
      
      
      new WaitCommand(0.5),
      swerveControllerCommand2,
      new SelfBalanceCommand(robot.m_robotDrive),
      new InstantCommand(
        ()-> robot.m_robotDrive.stop(),
        robot.m_robotDrive
      
      ),
      new InstantCommand(
       ()-> robot.m_robotDrive.setX(),
       robot.m_robotDrive
      )
    );
  }
}

