// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CoralIntakeConstants;

public class CoralIntakeSubsystem extends SubsystemBase {
  private CoralIntakeConstants constants = new CoralIntakeConstants();
  private Spark LeftMotor = new Spark(constants.LeftMotorID);
  private Spark RightMotor = new Spark(constants.RightMotorID);

  /** Creates a new ClimbSubsystem. */
  public CoralIntakeSubsystem() {

    // TalonFXConfigurator LeftConfig = LeftMotor.se
    // CurrentLimitsConfigs configs = new CurrentLimitsConfigs();

    // configs.StatorCurrentLimit = constants.STATOR_CURRENT_LIMIT;
    // configs.SupplyCurrentLimit = constants.CURRENT_LIMIT;
    // configs.StatorCurrentLimitEnable = constants.ENABLE_STATOR_CURRENT_LIMIT;
    // configs.SupplyCurrentLimitEnable = constants.ENABLE_CURRENT_LIMIT;

    // talonFXConfigurator.apply(configs);

    // Ive searched for a while but couldnt find any way to currnet limit sparks
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("ClimbPose", getDegrees());
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    LeftMotor.set(speed);
    RightMotor.set(-speed);
  }

  // public double getRawPose() {
  //   return Motor.getPosition().getValueAsDouble();
  // }

  // public double getDegrees() {
  //   return getRawPose()*360;
  // }
}
