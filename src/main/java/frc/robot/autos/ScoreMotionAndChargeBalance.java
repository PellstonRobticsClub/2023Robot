// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import javax.swing.text.ParagraphView;

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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ElevatorToX;
import frc.robot.commands.HighGoal;
import frc.robot.commands.HumanStation2;
import frc.robot.commands.HumanStationPos;
import frc.robot.commands.Pickup;
import frc.robot.commands.SelfBalanceCommand;
import frc.robot.commands.drivePostion;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreMotionAndChargeBalance extends SequentialCommandGroup {

  
  /** Creates a new MotionAndChargeBalance. */
  public ScoreMotionAndChargeBalance(RobotContainer robot) {
    PathPlannerTrajectory trajectory1 =
    PathPlanner.generatePath(
      new PathConstraints(2, 1.5), 
      new PathPoint(
        new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)),
        Rotation2d.fromDegrees(0),
        Rotation2d.fromDegrees(0)), 
        new PathPoint(
          new Translation2d(Units.inchesToMeters(-170), Units.inchesToMeters(0)),
          Rotation2d.fromDegrees(0),
          Rotation2d.fromDegrees(0)),
          new PathPoint(
          new Translation2d(Units.inchesToMeters(-196), Units.inchesToMeters(0)),
          Rotation2d.fromDegrees(0),
          Rotation2d.fromDegrees(180)));

          PathPlannerTrajectory trajectory2 =
    PathPlanner.generatePath(
      new PathConstraints(3.5, 2), 
      new PathPoint(
          new Translation2d(Units.inchesToMeters(-230), Units.inchesToMeters(0)),
          Rotation2d.fromDegrees(0),
          Rotation2d.fromDegrees(180)),
      
        new PathPoint(
          new Translation2d(Units.inchesToMeters(-120), Units.inchesToMeters(0)),
          Rotation2d.fromDegrees(0),
          Rotation2d.fromDegrees(180))
          );

      PathPlannerTrajectory trajectory3 =
      PathPlanner.generatePath(
        new PathConstraints(1,1),
      new PathPoint(
          new Translation2d(Units.inchesToMeters(-196), Units.inchesToMeters(0)),
          Rotation2d.fromDegrees(0),
          Rotation2d.fromDegrees(180)),
      new PathPoint(
          new Translation2d(Units.inchesToMeters(-230), Units.inchesToMeters(0)),
          Rotation2d.fromDegrees(0),
          Rotation2d.fromDegrees(180)));


          

      

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

          PPSwerveControllerCommand swerveControllerCommand3 =
        new PPSwerveControllerCommand(
          trajectory3,
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
            new Pose2d(new Translation2d(Units.inchesToMeters(0),0), Rotation2d.fromDegrees(0))
          )
      
      ),
      //new ElevatorToX(robot.m_elevator, 1.6),
      
      new RunCommand(
        () -> robot.m_intake.drive(-1),
        robot.m_intake
      ).withTimeout(.5),
      new WaitCommand(.1),

      new InstantCommand(
        () -> robot.m_intake.drive(0),
        robot.m_intake
      ),
      //new HumanStation2(robot.m_elevator, robot.m_extender),
      new ParallelCommandGroup(swerveControllerCommand1,
      new SequentialCommandGroup(new WaitCommand(1), new Pickup(robot.m_elevator, robot.m_extender))),
      
      new WaitCommand(0.5),
      new RunCommand(
        ()-> robot.m_intake.drive(1), robot.m_intake).withTimeout(.1),
      swerveControllerCommand3,
      new InstantCommand(
        () -> robot.m_intake.drive(0)
      ),
      new WaitCommand(.5),
      new ParallelDeadlineGroup(swerveControllerCommand2,new HumanStationPos(robot.m_elevator, robot.m_extender)),
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

