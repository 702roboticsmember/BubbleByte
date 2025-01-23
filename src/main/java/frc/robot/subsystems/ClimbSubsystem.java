// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimbSubsystem extends SubsystemBase {
  private TalonFX Motor = new TalonFX(Constants.ClimberConstants.MotorID);
  private ClimberConstants constants = new ClimberConstants();
  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    Motor.setInverted(Constants.ClimberConstants.MotorInverted);

    TalonFXConfigurator talonFXConfigurator = Motor.getConfigurator();
    CurrentLimitsConfigs configs = new CurrentLimitsConfigs();

    configs.StatorCurrentLimit = constants.STATOR_CURRENT_LIMIT;
    configs.SupplyCurrentLimit = constants.CURRENT_LIMIT;
    configs.StatorCurrentLimitEnable = constants.ENABLE_STATOR_CURRENT_LIMIT;
    configs.SupplyCurrentLimitEnable = constants.ENABLE_CURRENT_LIMIT;

    talonFXConfigurator.apply(configs);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ClimbPose", getDegrees());
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    Motor.set(speed);
  }

  public double getRawPose() {
    return Motor.getPosition().getValueAsDouble();
  }

  public double getDegrees() {
    return getRawPose()*360;
  }
}
