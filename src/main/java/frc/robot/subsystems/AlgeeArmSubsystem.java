package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgeeArmSubsystem extends SubsystemBase {
    Constants.AlgeeArmConstants constants = new Constants.AlgeeArmConstants();
    TalonFX Motor = new TalonFX(Constants.AlgeeArmConstants.MotorID);

    public AlgeeArmSubsystem(){
    TalonFXConfigurator talonFXConfigurator = Motor.getConfigurator();
    CurrentLimitsConfigs configs = new CurrentLimitsConfigs();

    configs.StatorCurrentLimit = Constants.AlgeeArmConstants.STATOR_CURRENT_LIMIT;
    configs.SupplyCurrentLimit = Constants.AlgeeArmConstants.CURRENT_LIMIT;
    configs.StatorCurrentLimitEnable = Constants.AlgeeArmConstants.ENABLE_STATOR_CURRENT_LIMIT;
    configs.SupplyCurrentLimitEnable = Constants.AlgeeArmConstants.ENABLE_CURRENT_LIMIT;
    
    talonFXConfigurator.apply(configs);
    }



    
}
