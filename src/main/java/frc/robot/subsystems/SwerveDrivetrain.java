// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveModule;

public class SwerveDrivetrain extends SubsystemBase {
  // state stuff
  public static enum DrivetrainState {
    AUTON_PATH, JOYSTICK_DRIVE
  }

  private DrivetrainState state;

  // kinematics stuff
  private SwerveDriveKinematics swerveKinematics;
  private SwerveModuleState[] desiredStates = new SwerveModuleState[4];
  private SwerveModule[] motors;

  // Auton Stuff
  private Pose2d pose;
  private SwerveDriveOdometry simOdometry;
  private SwerveDriveOdometry odometry;
  private Field2d field;
  private SimpleMotorFeedforward feedforward;
  public PIDController xController;
  public PIDController yController;
  public ProfiledPIDController thetaController;

  // sensors
  private AHRS gyro;
  
  // SIM
  private double time = 0;
  DriveSimulationData simulationData;
  
  /** Creates a new SwerveDrive. */
  public SwerveDrivetrain() {
    gyro = new AHRS(SPI.Port.kMXP);

    // reset in new thread since gyro needs some time to boot up and we don't 
    // want to interfere with other code
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      }
      catch (Exception e)
      {
        System.out.println("Reset Gyro Failed");
      }
    }).start(); 

    swerveKinematics = new SwerveDriveKinematics(
      Constants.SwerveDrivetrain.m_frontLeftLocation, 
      Constants.SwerveDrivetrain.m_frontRightLocation, 
      Constants.SwerveDrivetrain.m_backLeftLocation, 
      Constants.SwerveDrivetrain.m_backRightLocation);

    motors = new SwerveModule[4];

    motors[0] = new SwerveModule(
      Constants.SwerveDrivetrain.m_frontLeftDriveID,
      Constants.SwerveDrivetrain.m_frontLeftTurnID,
      Constants.SwerveDrivetrain.m_frontLeftEncoderID,
      false,
      false,
      false,
      Constants.SwerveDrivetrain.m_frontLeftEncoderOffset);

    motors[1] = new SwerveModule(
      Constants.SwerveDrivetrain.m_frontRightDriveID,
      Constants.SwerveDrivetrain.m_frontRightTurnID,
      Constants.SwerveDrivetrain.m_frontRightEncoderID,
      false,
      false,
      false,
      Constants.SwerveDrivetrain.m_frontRightEncoderOffset);

    motors[2] = new SwerveModule(
      Constants.SwerveDrivetrain.m_backLeftDriveID,
      Constants.SwerveDrivetrain.m_backLeftTurnID,
      Constants.SwerveDrivetrain.m_backLeftEncoderID,
      false,
      false,
      false,
      Constants.SwerveDrivetrain.m_backLeftEncoderOffset);

    motors[3] = new SwerveModule(
      Constants.SwerveDrivetrain.m_backRightDriveID,
      Constants.SwerveDrivetrain.m_backRightTurnID,
      Constants.SwerveDrivetrain.m_backRightEncoderID,
      false,
      false,
      false,
      Constants.SwerveDrivetrain.m_backRightEncoderOffset);

    pose = new Pose2d();
    odometry = new SwerveDriveOdometry(swerveKinematics, getRotation2d());
    simOdometry = new SwerveDriveOdometry(swerveKinematics, getRotation2d());
    field = new Field2d();

    simulationData = new DriveSimulationData(simOdometry, field);

    desiredStates[0] = new SwerveModuleState();
    desiredStates[1] = new SwerveModuleState();
    desiredStates[2] = new SwerveModuleState();
    desiredStates[3] = new SwerveModuleState();

    xController = new PIDController(Constants.SwerveDrivetrain.m_x_control_P, Constants.SwerveDrivetrain.m_x_control_I, Constants.SwerveDrivetrain.m_x_control_D);
    yController = new PIDController(Constants.SwerveDrivetrain.m_y_control_P, Constants.SwerveDrivetrain.m_y_control_I, Constants.SwerveDrivetrain.m_y_control_D);
    thetaController = new ProfiledPIDController(
      Constants.SwerveDrivetrain.m_r_control_P, 
      Constants.SwerveDrivetrain.m_r_control_I, 
      Constants.SwerveDrivetrain.m_r_control_D, 
      Constants.SwerveDrivetrain.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
    /**
     * use IEEEremainder because it uses formula:
     * dividend - (divisor x Math.Round(dividend / divisor))
     * versus the remainder operator (%) which uses:
     * (Math.Abs(dividend) - (Math.Abs(divisor) x (Math.Floor(Math.Abs(dividend) / Math.Abs(divisor))))) x Math.Sign(dividend)
     */
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putData("Field", field);
    for (SwerveModule m : motors) {
      SmartDashboard.putString(m.getName(), m.getState().toString());
    }
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    SmartDashboard.putString("Robot Sim Location", simOdometry.getPoseMeters().getTranslation().toString());

    odometry.update(getRotation2d(), getOutputModuleStates());

    double currTime = Timer.getFPGATimestamp();
    double dt = currTime - time;
    time = currTime;
    // Translation2d vel = getLinearVelocity();
    // double xPosSim = pose.getX() + vel.getX() * dt;
    // double yPosSim = pose.getY() + vel.getY() * dt;
    double anglePosSim = simulationData.getHeading() + getDesiredRotationalVelocity() * dt;
    simulationData.update(desiredStates, anglePosSim);
  }

  public void stopModules() {
    for (SwerveModule m : motors) {
      m.stop();
    }
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveDrivetrain.kMaxSpeedMPS);
    for (int i = 0; i < motors.length; i++)
    {
      motors[i].setDesiredState(states[i]);
      desiredStates[i] = states[i];
    }
  }

  public void setSpeeds(double v_forwardMps, double v_sideMps, double v_rot, Translation2d rotatePoint) {
    ChassisSpeeds speeds = new ChassisSpeeds(v_forwardMps, v_sideMps, v_rot);
    SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(speeds, rotatePoint);
    for (int i = 0; i < 4; i++)
    {
      motors[i].setDesiredState(moduleStates[i]);
    }
  }

  public SwerveDriveKinematics getKinematics() {
    return swerveKinematics;
  }

  public Translation2d getLinearVelocity() {
    ChassisSpeeds speeds = swerveKinematics.toChassisSpeeds(
        motors[0].getState(),
        motors[1].getState(),
        motors[2].getState(),
        motors[3].getState()
    );
    return new Translation2d(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond
    );
  }

  public double getRotationalVelocity() {
    ChassisSpeeds speeds = swerveKinematics.toChassisSpeeds(
        motors[0].getState(),
        motors[1].getState(),
        motors[2].getState(),
        motors[3].getState()
    );
    return speeds.omegaRadiansPerSecond;
  }

  public double getDesiredRotationalVelocity() {
    ChassisSpeeds speeds = swerveKinematics.toChassisSpeeds(
        motors[0].getDesiredState(),
        motors[1].getDesiredState(),
        motors[2].getDesiredState(),
        motors[3].getDesiredState()
    );
    return speeds.omegaRadiansPerSecond;
  }

  public SwerveModuleState[] getOutputModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++)
    {
      states[i] = motors[i].getState();
    }
    return states;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public Pose2d getSimPose() {
    return simOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, getRotation2d());
  }

  public void resetSimOdometry(Pose2d pose) {
    simOdometry.resetPosition(pose, new Rotation2d(simulationData.getHeading()));
  }

  public Field2d getField() {
    return field;
  }
}