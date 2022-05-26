// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.SwerveModule;
import frc.robot.Constants.Talon;

/** Add your docs here. */
public class SwerveMotorGroup {
    private BaseTalon talonDrive;
    private BaseTalon talonRotate;

    public SwerveMotorGroup(int driveID, int rotateID)
    {
        talonDrive = TalonFactory.createTalonFX(driveID, false);
        talonRotate = TalonFactory.createTalonFX(rotateID, false);

        talonDrive.config_kP(Constants.Talon.kPIDIdx, Constants.SwerveModule.kP);
        talonDrive.config_kI(Constants.Talon.kPIDIdx, Constants.SwerveModule.kI);
        talonDrive.config_kD(Constants.Talon.kPIDIdx, Constants.SwerveModule.kD); // Derivative not really needed for velocity
        talonDrive.config_kF(Constants.Talon.kPIDIdx, Constants.SwerveModule.kFF);

        talonRotate.config_kP(Constants.Talon.kPIDIdx, Constants.SwerveModule.kPTurn);
        talonRotate.config_kI(Constants.Talon.kPIDIdx, Constants.SwerveModule.kITurn);
        talonRotate.config_kD(Constants.Talon.kPIDIdx, Constants.SwerveModule.kDTurn);
    }

    public void setState(SwerveModuleState newState)
    {
        SwerveModuleState optimalState = SwerveModuleState.optimize(newState, new Rotation2d(getCurrAngle()));
        setAngle(optimalState.angle.getRadians());
        setVelocity(optimalState.speedMetersPerSecond);
    }

    public double getCurrAngle()
    {
        return MathUtils.ticksToRadians(talonRotate.getSelectedSensorPosition(), Constants.Talon.talonFXTicks, Constants.SwerveModule.gear_ratio_turn);
    }

    public void setAngle(double radians)
    {
        talonRotate.set(ControlMode.Position, MathUtils.radiansToTicks(radians, Constants.Talon.talonFXTicks, Constants.SwerveModule.gear_ratio_turn));
    }

    public void setVelocity(double v_mps)
    {
        talonDrive.set(ControlMode.Velocity, MathUtils.mpsToRPM(v_mps, Constants.SwerveModule.radius));
    }

    public void resetEncoders() {
        talonDrive.setSelectedSensorPosition(0);
        talonRotate.setSelectedSensorPosition(0);
    }

    public BaseTalon getDriver()
    {
        return talonDrive;
    }

    public BaseTalon getRotator()
    {
        return talonRotate;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(talonDrive.getSelectedSensorVelocity(), new Rotation2d(getCurrAngle()));
    }
}