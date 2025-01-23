// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimbSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbPID extends Command {
  
  private ClimbSubsystem c_ClimbSubsystem;
  private PIDController pid = new PIDController(
      Constants.ClimberConstants.kP, 
      Constants.ClimberConstants.kI, 
      Constants.ClimberConstants.kD);
  /** Creates a new ClimbPID. */
  public ClimbPID(ClimbSubsystem climb, double Setpoint) {
    this.c_ClimbSubsystem = climb;
    this.pid.setSetpoint(Setpoint);
    this.pid.setTolerance(Constants.ClimberConstants.Tolerance);
    addRequirements(c_ClimbSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pose = c_ClimbSubsystem.getAngle();
    double calculated = pid.calculate(pose);
    c_ClimbSubsystem.setSpeed(calculated);
    SmartDashboard.putNumber("ClimbPID", calculated);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    c_ClimbSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
