// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain();

  private final Joystick driveJoystick = new Joystick(Constants.SwerveDrivetrain.kDriveJoystickPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveDrivetrain.setDefaultCommand(new SwerveJoystickCommand(
      swerveDrivetrain, 
      () -> -driveJoystick.getRawAxis(Constants.SwerveDrivetrain.kDriveXAxis), 
      () -> -driveJoystick.getRawAxis(Constants.SwerveDrivetrain.kDriveYAxis), 
      () -> -driveJoystick.getRawAxis(Constants.SwerveDrivetrain.kDriveWAxis), 
      () -> !driveJoystick.getRawButton(Constants.SwerveDrivetrain.kDriveFieldOrientButtonIdx)));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driveJoystick, 2).whenPressed(() -> swerveDrivetrain.zeroHeading());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      Constants.SwerveDrivetrain.kDriveMaxSpeedMPS, 
      Constants.SwerveDrivetrain.kDriveMaxAcceleration);
    trajectoryConfig.setKinematics(swerveDrivetrain.getKinematics());

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        new Translation2d(1, 2),
        new Translation2d(3, 1)
      ), 
      new Pose2d(4, 2, Rotation2d.fromDegrees(180.0)),
      trajectoryConfig);
    
    swerveDrivetrain.getField().getObject("traj").setTrajectory(trajectory);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory, 
      swerveDrivetrain::getSimPose, 
      swerveDrivetrain.getKinematics(), 
      swerveDrivetrain.xController, 
      swerveDrivetrain.yController, 
      swerveDrivetrain.thetaController, 
      swerveDrivetrain::setModuleStates,
      swerveDrivetrain);
    
    return new SequentialCommandGroup(
      new InstantCommand(() -> swerveDrivetrain.resetSimOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> swerveDrivetrain.stopModules())
    );
  }
}
