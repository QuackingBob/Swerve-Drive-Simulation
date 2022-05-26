// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.MathUtils;

public class SwerveJoystickCommand extends CommandBase {
  private final SwerveDrivetrain drivetrain;
  private final Supplier<Double> xSpeedFunc, ySpeedFunc, turnSpeedFunc;
  private final Supplier<Boolean> fieldOrientedFunc;
  private final SlewRateLimiter xLimiter, yLimiter, wLimiter;

  /** Creates a new SwerveJoystickCommand. */
  public SwerveJoystickCommand(SwerveDrivetrain drivetrain, Supplier<Double> xSpeedFunc, Supplier<Double> ySpeedFunc, Supplier<Double> angularSpeedFunc, Supplier<Boolean> fieldOrientedFunc) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.xSpeedFunc = xSpeedFunc;
    this.ySpeedFunc = ySpeedFunc;
    this.turnSpeedFunc = angularSpeedFunc;
    this.fieldOrientedFunc = fieldOrientedFunc;
    this.xLimiter = new SlewRateLimiter(Constants.SwerveDrivetrain.kDriveMaxAcceleration);
    this.yLimiter = new SlewRateLimiter(Constants.SwerveDrivetrain.kDriveMaxAcceleration);
    this.wLimiter = new SlewRateLimiter(Constants.SwerveDrivetrain.kTurnMaxAcceleration);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double vX = xSpeedFunc.get();
    double vY = ySpeedFunc.get();
    double vW = turnSpeedFunc.get();

    // apply deadband
    vX = MathUtils.handleDeadband(vX, Constants.SwerveDrivetrain.kThrottleDeadband);
    vY = MathUtils.handleDeadband(vY, Constants.SwerveDrivetrain.kThrottleDeadband);
    vW = MathUtils.handleDeadband(vW, Constants.SwerveDrivetrain.kWheelDeadband);

    // limit acceleration
    vX = xLimiter.calculate(vX) * Constants.SwerveDrivetrain.kDriveMaxSpeedMPS;
    vY = yLimiter.calculate(vY) * Constants.SwerveDrivetrain.kDriveMaxSpeedMPS;
    vW = wLimiter.calculate(vW) * Constants.SwerveDrivetrain.kTurnMaxSpeedRPS;

    // get chassis speed
    ChassisSpeeds chassisSpeeds;
    if (fieldOrientedFunc.get()) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        vX, vY, vW, drivetrain.getRotation2d());
    }
    else {
      chassisSpeeds = new ChassisSpeeds(vX, vY, vW);
    }
    
    // convert to module states and apply to each wheel
    SwerveModuleState[] moduleStates = drivetrain.getKinematics().toSwerveModuleStates(
      chassisSpeeds,
      Constants.SwerveDrivetrain.rotatePoints[drivetrain.getRotationPointIdx()]);
    drivetrain.setModuleStates(moduleStates);
    SmartDashboard.putNumber("vX", vX);
    SmartDashboard.putNumber("vY", vY);
    SmartDashboard.putNumber("vW", vW);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
