// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntakeSubsystem extends SubsystemBase {
  private Spark LeftMotor = new Spark(Constants.CoralIntakeConstants.LeftMotorID);
  private Spark RightMotor = new Spark(Constants.CoralIntakeConstants.RightMotorID);

  /** Creates a new ClimbSubsystem. */
  public CoralIntakeSubsystem() {
    LeftMotor.setInverted(Constants.CoralIntakeConstants.LeftMotorInverted);
    RightMotor.setInverted(Constants.CoralIntakeConstants.RightMotorInverted);

    
    

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
    RightMotor.set(speed);
  }

}
