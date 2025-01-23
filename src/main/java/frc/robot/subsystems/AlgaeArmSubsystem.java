package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeArmSubsystem extends SubsystemBase {
    Constants.AlgaeArmConstants constants = new Constants.AlgaeArmConstants();
    TalonFX Motor = new TalonFX(Constants.AlgaeArmConstants.MotorID);

    public AlgaeArmSubsystem(){
        TalonFXConfigurator talonFXConfigurator = Motor.getConfigurator();
        CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        HardwareLimitSwitchConfigs limitConfigs = new HardwareLimitSwitchConfigs();

        limitConfigs.ForwardLimitAutosetPositionEnable = Constants.AlgaeArmConstants.LimitEnable;
        limitConfigs.ForwardLimitAutosetPositionValue = Constants.AlgaeArmConstants.ForwardLimit;
        limitConfigs.ReverseLimitAutosetPositionEnable = Constants.AlgaeArmConstants.LimitEnable;
        limitConfigs.ReverseLimitAutosetPositionValue = Constants.AlgaeArmConstants.ReverseLimit;
        
        configs.StatorCurrentLimit = Constants.AlgaeArmConstants.STATOR_CURRENT_LIMIT;
        configs.SupplyCurrentLimit = Constants.AlgaeArmConstants.CURRENT_LIMIT;
        configs.StatorCurrentLimitEnable = Constants.AlgaeArmConstants.ENABLE_STATOR_CURRENT_LIMIT;
        configs.SupplyCurrentLimitEnable = Constants.AlgaeArmConstants.ENABLE_CURRENT_LIMIT;

        motorConfigs.Inverted = Constants.AlgaeArmConstants.MotorInverted;
        motorConfigs.NeutralMode = Constants.AlgaeArmConstants.MotorMode;
        
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
