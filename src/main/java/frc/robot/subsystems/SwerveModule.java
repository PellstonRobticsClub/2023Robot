// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
  

  private final String m_name;
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;
  private final SlewRateLimiter filter = new SlewRateLimiter(ModuleConstants.kPModuleDriveSlewRate);
  private AnalogInput absoluteEncoder;

  private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);
  private double AnalogEncoderOffset;
  

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final PIDController m_turningPIDController = new PIDController(
      ModuleConstants.kPModuleTurningController,
      0,
      0
  );
   //   new TrapezoidProfile.Constraints(
   //       ModuleConstants.kMaxModuleAngularSpeedDegreesPerSecond,
   //       ModuleConstants.kMaxModuleAngularAccelerationDegreesPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param name                   the name of the module
   * @param driveMotorChannel      The channel of the drive motor.
   * @param turningMotorChannel    The channel of the turning motor.
   * @param turningEncoderChannel  The channels of the turning encoder.
   * @param driveMotorReversed   Whether the drive encoder is reversed.
   * @param turningMotorReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      String name,
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      boolean driveMotorReversed,
      boolean turningMotorReversed,
      double encoderOffset) {
        AnalogEncoderOffset = encoderOffset;
        absoluteEncoder = new AnalogInput(turningEncoderChannel);
        m_name = name;
        m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
        m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

        m_driveEncoder = m_driveMotor.getEncoder();

        m_turningEncoder = m_turningMotor.getEncoder();

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        // m_driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);
        m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderDistancePerPulse);

        // Set whether drive encoder should be reversed or not
        // m_driveEncoder.setReverseDirection(driveEncoderReversed);
        m_driveMotor.setInverted(driveMotorReversed);

        // Set the distance (in this case, angle) in radians per pulse for the turning
        // encoder.
        // This is the the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.
        // m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);
        //m_turningEncoder.setDistancePerRotation(ModuleConstants.kTurningEncoderDistancePerPulse);

        // Set whether turning encoder should be reversed or not
        // m_turningEncoder.setReverseDirection(turningEncoderReversed);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        m_turningEncoder.setPositionConversionFactor(Constants.ModuleConstants.kTurningEncoderDistancePerPulse);
        m_turningMotor.setInverted(driveMotorReversed);



        resetEncoders();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    //stops turning back to zero when you release the joystick
    if(Math.abs(desiredState.speedMetersPerSecond) < .1){
      stop();
      return;
      
    }
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getPosition()));

    // Calculate the drive output from the drive PID controller.
    //final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);
    final double driveOutput = state.speedMetersPerSecond;
    // Calculate the turning motor output from the turning PID controller.
    double turnOutput;
    //if(Math.abs((m_turningEncoder.getPosition()) - state.angle.getDegrees())< 2 || Math.abs(driveOutput) < .25){
    //  turnOutput =0;
    //}else{
    turnOutput = m_turningPIDController.calculate(m_turningEncoder.getPosition(),
       state.angle.getRadians());
    //}
    // Calculate the turning motor output from the turning PID controller.
    // SmartDashboard.putNumber(m_name + "driveSpeed", driveOutput);
    // SmartDashboard.putNumber(m_name +"turnSpeed", turnOutput);
    SmartDashboard.putNumber(m_name +" drive out", driveOutput/2);
    SmartDashboard.putNumber(m_name +" current Angle",m_turningEncoder.getPosition());
    SmartDashboard.putNumber(m_name + " desired angle", state.angle.getRadians());
   
    SmartDashboard.putNumber(m_name + " turning error", m_turningEncoder.getPosition() - state.angle.getRadians());
    m_driveMotor.set(filter.calculate(driveOutput/2));
    SmartDashboard.putNumber(m_name + " turn output", turnOutput);
    m_turningMotor.set(turnOutput);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(getAbsoluteEncoderRad());
  }

  public CANSparkMax driveSpark() {
    return m_driveMotor;
  }

  public CANSparkMax turnSpark() {
    return m_turningMotor;
  }

  public void stop(){
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  }

  public double getAbsoluteEncoderRad(){
    double angle = absoluteEncoder.getAverageVoltage() / RobotController.getVoltage5V();
    angle *= 2.0 * Math.PI;
    angle -= AnalogEncoderOffset;
    return angle ;
  }

  public void update(){
    SmartDashboard.putNumber(m_name + "absolute encoder", getAbsoluteEncoderRad());
    SmartDashboard.putNumber(m_name + "reletive encoder", m_turningEncoder.getPosition());
  }
}
