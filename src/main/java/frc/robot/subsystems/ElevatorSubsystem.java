// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class ElevatorSubsystem extends PIDSubsystem {
  private static TalonSRX elevatorMotor = new TalonSRX(Constants.ElevatorConstants.MotorID);
  private static AnalogInput potentiometer = new AnalogInput(Constants.ElevatorConstants.potentiometerAnologID);
  /** Creates a new ElevatorSubsytem. */
  public ElevatorSubsystem() {
    super(
        // The PIDController used by the subsystem
        new PIDController(1, .3, 0));
        elevatorMotor.setInverted(false);
        this.disable();
        getController().setTolerance(.1);
        
        
  }

  @Override
  public void useOutput(double output, double setpoint) {

    //SmartDashboard.putNumber("ElevatorOutput", output);
    
  drive(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return potentiometer.getAverageVoltage();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("elevator Height", (potentiometer.getAverageVoltage()));
      // TODO Auto-generated method stub
      super.periodic();
      
  }
  public void drive(double speed){
    if (potentiometer.getAverageVoltage() > 4.7 && speed > 0){
      speed = 0;
    }
    if (potentiometer.getAverageVoltage() < .8 && speed < 0 ){
      speed = 0;
    }
    elevatorMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean atTarget(){
    return m_controller.atSetpoint();
  }
}
