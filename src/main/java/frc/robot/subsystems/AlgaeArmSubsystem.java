package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeArmSubsystem extends SubsystemBase {
    Constants.AlgeeArmConstants constants = new Constants.AlgeeArmConstants();
    TalonFX Motor = new TalonFX(Constants.AlgeeArmConstants.MotorID);

    public AlgaeArmSubsystem(){
        TalonFXConfigurator talonFXConfigurator = Motor.getConfigurator();
        CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        
        configs.StatorCurrentLimit = Constants.AlgeeArmConstants.STATOR_CURRENT_LIMIT;
        configs.SupplyCurrentLimit = Constants.AlgeeArmConstants.CURRENT_LIMIT;
        configs.StatorCurrentLimitEnable = Constants.AlgeeArmConstants.ENABLE_STATOR_CURRENT_LIMIT;
        configs.SupplyCurrentLimitEnable = Constants.AlgeeArmConstants.ENABLE_CURRENT_LIMIT;
        
        talonFXConfigurator.apply(configs);
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("AlgaePose", getAngle());
        // This method will be called once per scheduler run
    }

    public void setSpeed(double speed) {
        Motor.set(speed);
    }

    public double getRawPose() {
        return Motor.getPosition().getValueAsDouble();
    }

    public double tickToDeg(double tick){
        return tick * 360;
    }

    public double getAngle() {
        return tickToDeg(getRawPose());
    }
    
}
