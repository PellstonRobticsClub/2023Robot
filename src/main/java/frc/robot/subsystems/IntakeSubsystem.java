// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private TalonSRX intakeMotor = new TalonSRX(Constants.IntakeConstants.motorID);
  private TalonSRX intakeMotor2 = new TalonSRX(Constants.IntakeConstants.motor2ID);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor2.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double speed){
    if(Math.abs(speed)< .1){
      speed = 0;
    }
    intakeMotor.set(ControlMode.PercentOutput, speed);
    intakeMotor2.set(ControlMode.PercentOutput,-speed);
  }
  public void setDirection(String object){
    if(object == "cube"){
     intakeMotor.setInverted(true);
     intakeMotor2.setInverted(true);
    }
    if(object == "cone"){
      intakeMotor.setInverted(false);
      intakeMotor2.setInverted(false);
     }
  }
}
