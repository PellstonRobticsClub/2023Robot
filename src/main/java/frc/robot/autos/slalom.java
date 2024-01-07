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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class slalom extends SequentialCommandGroup {

  
  /** Creates a new MotionAndChargeBalance. */
  public slalom(RobotContainer robot) {
    PathPlannerTrajectory trajectory1 =
    PathPlanner.generatePath(
      new PathConstraints(3.5, 2.5), 
      new PathPoint(
        new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)),
        Rotation2d.fromDegrees(0),
        Rotation2d.fromDegrees(0)), 
        new PathPoint(
          new Translation2d(4, 1),
          Rotation2d.fromDegrees(0),
          Rotation2d.fromDegrees(0)),
          //new PathPoint(
          //new Translation2d(6, 0),
          //Rotation2d.fromDegrees(0),//1
          //Rotation2d.fromDegrees(0)),
          new PathPoint(
          new Translation2d(9, -1),
          Rotation2d.fromDegrees(0),
          Rotation2d.fromDegrees(0)),
          //new PathPoint(
          //new Translation2d(10.5, 0),
          //Rotation2d.fromDegrees(0),//2
          //Rotation2d.fromDegrees(0)),
          new PathPoint(
          new Translation2d(11.5, 1.5),
          Rotation2d.fromDegrees(0),
          Rotation2d.fromDegrees(0)),
          //new PathPoint(
          //new Translation2d(14.5, 0),
          //Rotation2d.fromDegrees(0),//3
          //Rotation2d.fromDegrees(0)),
          new PathPoint(
          new Translation2d(17, -1.5),
          Rotation2d.fromDegrees(0),
          Rotation2d.fromDegrees(0)),
          //new PathPoint(
          //new Translation2d(18.5, 0),
          //Rotation2d.fromDegrees(0),//4
          //Rotation2d.fromDegrees(0)),
          new PathPoint(
          new Translation2d(20, 1.25),
          Rotation2d.fromDegrees(0),
          Rotation2d.fromDegrees(0)),
          new PathPoint(
          new Translation2d(22.25, 1.25),
          Rotation2d.fromDegrees(0),//5
          Rotation2d.fromDegrees(0)),
          new PathPoint(
          new Translation2d(22.25, -1),
          Rotation2d.fromDegrees(0),//5
          Rotation2d.fromDegrees(0)),
          new PathPoint(
          new Translation2d(24, -1.25),
          Rotation2d.fromDegrees(0),
          Rotation2d.fromDegrees(0)),
          new PathPoint(
          new Translation2d(27.5, 0),
          Rotation2d.fromDegrees(0),//6
          Rotation2d.fromDegrees(0)),
          new PathPoint(
          new Translation2d(30, 0),
          Rotation2d.fromDegrees(0),//6
          Rotation2d.fromDegrees(180))
          );



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
      //swerveControllerCommand2,
      new InstantCommand(
        ()-> robot.m_robotDrive.stop(),
        robot.m_robotDrive
      
      )
    );
  }
}

