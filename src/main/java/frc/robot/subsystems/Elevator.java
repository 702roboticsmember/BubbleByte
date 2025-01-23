// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

//might be removed idk just putting this here

//unnescesary import, idk what it was

import java.lang.Math;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  public TalonFX elevmotor1 = new TalonFX(Constants.ElevatorConstants.Motor1ID);
  public TalonFX elevmotor2 = new TalonFX(Constants.ElevatorConstants.Motor2ID);
      
  /** Creates a new Elevator. */
  public Elevator() {
    // Elevator PID :D Will most likely be moved to Elevator PID later and errors will be fixed trust
    //Add current limits to constants??
    CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    HardwareLimitSwitchConfigs limitConfigs = new HardwareLimitSwitchConfigs();

    limitConfigs.ForwardLimitAutosetPositionEnable = Constants.ElevatorConstants.LimitEnable;
    limitConfigs.ForwardLimitAutosetPositionValue = Constants.ElevatorConstants.ForwardLimit;
    limitConfigs.ReverseLimitAutosetPositionEnable = Constants.ElevatorConstants.LimitEnable;
    limitConfigs.ReverseLimitAutosetPositionValue = Constants.ElevatorConstants.ReverseLimit;

    currentConfigs.StatorCurrentLimit = Constants.ElevatorConstants.STATOR_CURRENT_LIMIT;
    currentConfigs.StatorCurrentLimitEnable = Constants.ElevatorConstants.ENABLE_STATOR_CURRENT_LIMIT;
    currentConfigs.SupplyCurrentLimit = Constants.ElevatorConstants.CURRENT_LIMIT;
    currentConfigs.SupplyCurrentLimitEnable = Constants.ElevatorConstants.ENABLE_CURRENT_LIMIT; 

    motorConfigs.Inverted = Constants.ElevatorConstants.MotorInverted;
    motorConfigs.NeutralMode = Constants.ElevatorConstants.MotorMode;

    
    elevmotor1.getConfigurator().apply(currentConfigs);
    elevmotor2.getConfigurator().apply(currentConfigs);
    elevmotor1.getConfigurator().apply(motorConfigs);
    elevmotor2.getConfigurator().apply(motorConfigs);
    elevmotor1.getConfigurator().apply(limitConfigs);
    elevmotor2.getConfigurator().apply(limitConfigs);
    
    //elevmotor1.config_kP(0, Constants.kElevatorP, Constants.TimeoutMs);
    //elevmotor1.config_kI(0, Constants.kElevatorI, Constants.TimeoutMs);
    //elevmotor1.config_kD(0, Constants.kElevatorD, Constants.TimeoutMs);
    //Probably not needed but just in case:
    // elevmotor1.config_kP(0, Constants.kElevatorP, Constants.TimeoutMs);
    // elevmotor1.config_kI(0, Constants.kElevatorI, Constants.TimeoutMs);
    // elevmotor1.config_kD(0, Constants.kElevatorD, Constants.TimeoutMs);
    
    // elevmotor2.config_kP(0, Constants.kElevatorP, Constants.TimeoutMs);
    // elevmotor2.config_kI(0, Constants.kElevatorI, Constants.TimeoutMs);
    // elevmotor2.config_kD(0, Constants.kElevatorD, Constants.TimeoutMs);
  }



  public double TickToDeg(double tick) {
      return tick * 360;
  }

  public double DegToTick(double tick) {
    return tick / 360;
}
// trying to get it to figure out height of elevator based off motor postition. I think something is wrong.
// Instead, maybe figure out a constant from tick to distance up? Like multiply tick by a constant we figure out.

// seems to work fine to me, it should return the height, but just in case I would use getRaw() for PIDs and
// reserve this for smartdashboard
  public double getElevatorHeight(/*double tick*/) {
    double radius = Constants.ElevatorConstants.Radius;
    double radians = Math.toRadians(TickToDeg(getRaw()));
    return radians * radius;
  }

  public double getRaw() {
    return elevmotor1.getPosition().getValueAsDouble();
  }

  public void set(double value) {
    elevmotor1.set(value);
    elevmotor2.set(value);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Height", getElevatorHeight());
    // This method will be called once per scheduler run
  }
}
