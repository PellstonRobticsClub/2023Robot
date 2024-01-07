// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;

public class extenderToX extends CommandBase {
  private static ExtenderSubsystem extender;
  private double position;
  /** Creates a new ElevatorStop. */
  public extenderToX(ExtenderSubsystem extenderIn, double positionIn) {
    extender = extenderIn;
    position = positionIn;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(extender);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    extender.enable();
    extender.setSetpoint(position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    extender.disable();
    extender.drive(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return extender.atTarget();
  }

  
}
