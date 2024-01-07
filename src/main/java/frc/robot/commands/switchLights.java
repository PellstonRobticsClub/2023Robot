// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class switchLights extends CommandBase {
  private RobotContainer robot;
  private String color;
  private NetworkTableInstance nti = NetworkTableInstance.getDefault();
  NetworkTable driverStationLed = nti.getTable("led");
  /** Creates a new switchLights. */
  public switchLights(RobotContainer robotIn, String colorIn) {
    robot = robotIn;
    color = colorIn;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(color == "purple"){
      robot.m_intake.setDirection("cube");
      driverStationLed.getEntry("Green").setBoolean(false);
      driverStationLed.getEntry("Red").setBoolean(true);
      driverStationLed.getEntry("Blue").setBoolean(true);
      for (var i = 0; i < robot.m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        
        
          robot.m_ledBuffer.setRGB(i, 255,  0, 150);
        
        
      } 
      
    }
    if (color == "yellow")
    {
      robot.m_intake.setDirection("cone");
      driverStationLed.getEntry("Green").setBoolean(true);
      driverStationLed.getEntry("Red").setBoolean(true);
      driverStationLed.getEntry("Blue").setBoolean(false);
      for (var i = 0; i < robot.m_ledBuffer.getLength(); i++) {
       robot.m_ledBuffer.setRGB(i, 255, 125, 0);
      }
      
     
   }
   
   robot.m_led.setData(robot.m_ledBuffer);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
