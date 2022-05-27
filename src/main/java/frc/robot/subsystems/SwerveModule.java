// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MathUtils;
import frc.robot.utils.TalonFactory;

public class SwerveModule {
  private final BaseTalon driveMotor;
  private final BaseTalon turnMotor;

  private final PIDController turnPIDController; 
  private final AnalogInput absEncoder;
  private final boolean absEncoderReversed;
  private final double absEncoderOffsetRad;

  private SwerveModuleState desiredState;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveID, int turnID, int encoderID, boolean driveReversed, boolean turnReversed, boolean encoderReversed, double encoderOffset) {
    driveMotor = TalonFactory.createTalonFX(driveID, driveReversed);
    turnMotor = TalonFactory.createTalonFX(turnID, turnReversed);

    driveMotor.config_kP(Constants.Talon.kPIDIdx, Constants.SwerveModule.kP);
    driveMotor.config_kI(Constants.Talon.kPIDIdx, Constants.SwerveModule.kI);
    driveMotor.config_kD(Constants.Talon.kPIDIdx, Constants.SwerveModule.kD);
    driveMotor.config_kF(Constants.Talon.kPIDIdx, Constants.SwerveModule.kFF);

    absEncoderReversed = encoderReversed;
    absEncoderOffsetRad = encoderOffset;
    absEncoder = new AnalogInput(encoderID);

    turnPIDController = new PIDController(
      Constants.SwerveModule.kPTurn,
      Constants.SwerveModule.kITurn,
      Constants.SwerveModule.kDTurn);
    
    turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

    desiredState = new SwerveModuleState();

    resetEncoders();
  }
  
  /**
   * Get the drive position of the module
   * @return the position in meters
   */
  public double getDrivePosition() {
    return MathUtils.ticksToMeter(
      driveMotor.getSelectedSensorPosition(), 
      Constants.Talon.talonFXTicks, 
      Constants.SwerveModule.gear_ratio_drive, 
      Constants.SwerveModule.radius);
  }

  /**
   * Get the turn position for the module
   * @return the position in radians
   */
  public double getTurnPosition() {
    return MathUtils.ticksToRadians(
      turnMotor.getSelectedSensorPosition(),
      Constants.Talon.talonFXTicks,
      Constants.SwerveModule.gear_ratio_turn);
  }

  /**
   * Get the linear velocity of the module
   * @return the linear velocity in mps
   */
  public double getDriveVelocity() {
    return MathUtils.rpmToMPS(
      MathUtils.ticksToRPM(
        driveMotor.getSelectedSensorVelocity(),
        Constants.Talon.talonFXTicks,
        Constants.SwerveModule.gear_ratio_drive),
      Constants.SwerveModule.radius);
  }

  /**
   * Get the angular velocity of the module
   * @return the angular velocity in rpm
   */
  public double getTurnVelocity() {
    return MathUtils.ticksToRPM(
      turnMotor.getSelectedSensorVelocity(),
      Constants.Talon.talonFXTicks,
      Constants.SwerveModule.gear_ratio_drive);
  }

  /**
   * Get the angle of the absolute encoder sensor on the module
   * @return the absolute encoder angle in radians
   */
  public double getAbsoluteEncoderRad() {
    double angle = absEncoder.getVoltage() / RobotController.getVoltage5V();
    angle *= 2.0 * Math.PI;
    angle -= absEncoderOffsetRad;
    return angle * (absEncoderReversed? -1.0 : 1.0);
  }

  /**
   * reset the encoders of the module
   * calibrate turn motor using abs encoder value
   */
  public void resetEncoders() {
    driveMotor.setSelectedSensorPosition(0);
    turnMotor.setSelectedSensorPosition(
      MathUtils.ticksToRadians(getAbsoluteEncoderRad(),
        Constants.Talon.talonFXTicks,
        Constants.SwerveModule.gear_ratio_turn));
  }

  /**
   * set the angle of the turn motor
   * @param radians
   */
  public void setAngle(double radians)
  {
      turnMotor.set(
        ControlMode.Position, 
        MathUtils.radiansToTicks(
          turnPIDController.calculate(getTurnPosition(), radians),
          Constants.Talon.talonFXTicks,
          Constants.SwerveModule.gear_ratio_turn));
  }

  /**
   * set the velocity of the drive motor
   * @param v_mps
   */
  public void setVelocity(double v_mps)
  {
      driveMotor.set(ControlMode.Velocity, MathUtils.mpsToRPM(v_mps, Constants.SwerveModule.radius));
  }

  /**
   * get the swerve module state (with velocity and rotation)
   * @return SwerveModuleState
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
  }

  /**
   * get the desired state of the module
   * @return SwerveModuleState
   */
  public SwerveModuleState getDesiredState() {
    return desiredState;
  }

  /**
   * set the desired state of the module
   * @param state
   */
  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001)
    {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle); // ex. move -45 degrees instead of 225 degrees if at 0 degrees
    desiredState = state;
    setVelocity(state.speedMetersPerSecond);
    setAngle(state.angle.getRadians());
    SmartDashboard.putString("Swerve [" + absEncoder.getChannel() + "] state", state.toString());
  }

  /**
   * stop the module from running
   */
  public void stop() {
    driveMotor.set(ControlMode.PercentOutput, 0);
    turnMotor.set(ControlMode.PercentOutput, 0);
    desiredState = new SwerveModuleState(0, desiredState.angle);
  }

  /**
   * get the swerve module name (useful for SmartDashboard)
   * @return String name
   */
  public String getName() {
    return "Swerve " + absEncoder.getChannel();
  }
}
