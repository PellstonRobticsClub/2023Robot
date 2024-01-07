// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class preBalanceCommand extends CommandBase {
  private DriveSubsystem m_drive;
  /** Creates a new balanceCommand. */
  public preBalanceCommand(DriveSubsystem driveIn) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = driveIn;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.switchBrake();
    m_drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_drive.drive(1, -.3, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(1, 0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //SmartDashboard.putNumber("distance", m_drive.getAverageDistance());
    return (m_drive.getAverageDistance() > 1.5);
  }
}
