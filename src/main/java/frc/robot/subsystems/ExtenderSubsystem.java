// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class ExtenderSubsystem extends PIDSubsystem {
  private static final TalonSRX extenderMotor = new TalonSRX(Constants.ExtenderConstants.MotorID);
  private static final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(Constants.ExtenderConstants.absoluteEncoderPort);
  /** Creates a new ExtenderSubsystem. */
  public ExtenderSubsystem() {
    super(
        // The PIDController used by the subsystem
        new PIDController(1, 0, 0));
    extenderMotor.setInverted(true);
    getController().setTolerance(.2);
    this.disable();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    //SmartDashboard.putNumber("extenderOutput2", output);
    drive(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getAbsoPos();
  }

  public void drive(double speed ){
    if (getAbsoPos() > 12 && speed >0) {
      speed =0;
    }
    if (getAbsoPos() < .1 && speed < 0){
      speed =0;
    }
    extenderMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("extender height", getAbsoPos());
      //SmartDashboard.putBoolean("atTarget", atTarget());
      //SmartDashboard.putNumber("setpoint", getController().getSetpoint());

      super.periodic();
  }

  public double getAbsoPos(){
    return absoluteEncoder.get() - .12;
  }

  public boolean atTarget(){
    return getController().atSetpoint();
  }
}
