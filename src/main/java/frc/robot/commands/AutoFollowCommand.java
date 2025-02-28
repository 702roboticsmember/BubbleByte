// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class AutoFollowCommand extends Command {
  boolean interrupted;

  private PIDController AutoFollowPID = new PIDController(

      Constants.AutoFollowConstants.kP/11,
      Constants.AutoFollowConstants.kI,
      Constants.AutoFollowConstants.kD);

      private PIDController AutoAimPID = new PIDController(
        Constants.AutoAimConstants.kP,
        Constants.AutoAimConstants.kI,
        Constants.AutoAimConstants.kD);
  
  
  DoubleSupplier tx;
  DoubleSupplier ta;
  BooleanSupplier tv;
  Swerve s_Swerve;
  double turn;
  double prev;

  /** Creates a new AutoAim. */
  public AutoFollowCommand(DoubleSupplier tx, DoubleSupplier ta, BooleanSupplier tv, Swerve s_Swerve, double turn) {
    this.ta = ta;
    this.tx = tx;
    this.tv = tv;
    this.s_Swerve = s_Swerve;
    this.turn = turn;
    this.prev = 0;
    
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // RobotContainer.robotCentric = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AutoFollowPID.setSetpoint(15);
    AutoFollowPID.setTolerance(7);
    AutoAimPID.setSetpoint(1);
    AutoAimPID.setTolerance(1);
    


    double a = ta.getAsDouble();
    boolean Target = tv.getAsBoolean();
    double value = AutoFollowPID.calculate(a);
    double FollowPID = (Target ? MathUtil.clamp(value, -0.87, 0.87) : 0);
    SmartDashboard.putNumber("FPID", value);
    SmartDashboard.putNumber("Fta", a);

    double x = tx.getAsDouble();

    double value2 = AutoAimPID.calculate(x);
    //double result2 = Math.copySign(Math.abs(value2) + 0.0955, value2); 
    double AimPID = (Target ? MathUtil.clamp(value2, -0.57, 0.57) : turn);
    // SmartDashboard.putNumber("FollowPID", RobotContainer.FollowPID);
    SmartDashboard.putNumber("Ftx", x);
    s_Swerve.drive(
                new Translation2d(0, FollowPID).times(Constants.Swerve.MAX_SPEED),
                AimPID * Constants.Swerve.MAX_ANGULAR_VELOCITY,
                !true,
                true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //  RobotContainer.robotCentric = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
