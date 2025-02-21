
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase 
{

    private final TalonFX m_wrist = new TalonFX(20);  
    //private final DigitalInput DIO = new DigitalInput(0);  
    

// PID constants
    private double kP = 2.4;  // Proportional gain
    private double kI = 0;  // Integral gain
    private double kD = 0.0;  // Derivative gain
    private double kS = 0.0;  // Static friction compensation
    private double kV = 0.0;  // Velocity term for overcoming friction

    private double positionSetPoint = 0.0;
    
    public WristSubsystem() {
        
        m_wrist.setNeutralMode(NeutralModeValue.Brake);
       

        var configs = new TalonFXConfiguration();
        configs.Voltage.PeakForwardVoltage = 4;
        configs.Voltage.PeakReverseVoltage = -4;
        configs.TorqueCurrent.PeakForwardTorqueCurrent = 20;
        configs.TorqueCurrent.PeakReverseTorqueCurrent = -20;
        m_wrist.getConfigurator().apply(configs);

  var talonFXConfigurator = m_wrist.getConfigurator();        
        var limitConfigs = new CurrentLimitsConfigs();
          limitConfigs.StatorCurrentLimit = 20;
          limitConfigs.StatorCurrentLimitEnable = true;
          talonFXConfigurator.apply(limitConfigs);


      
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;
        slot0Configs.kS = kS;
        slot0Configs.kV = kV;
        m_wrist.getConfigurator().apply(slot0Configs);
            
    }

   
    public void setWristPosition(double positionSetPoint) {
        this.positionSetPoint = positionSetPoint;
        PositionVoltage request = new PositionVoltage(positionSetPoint).withSlot(0);
        m_wrist.setControl(request);
    }

    public double getPosition() {
        return m_wrist.getPosition().getValueAsDouble();
    }

    public void setWristBrake() {
        m_wrist.setNeutralMode(NeutralModeValue.Brake);
       
    }

    public void setWristCoast() {
        m_wrist.setNeutralMode(NeutralModeValue.Coast);
     }


    @Override
    public void periodic() 
	{             
        SmartDashboard.putNumber("Wrist Position", m_wrist.getPosition().getValueAsDouble());       
        SmartDashboard.putNumber("Wrist Setpoint", positionSetPoint);

        double statorCurrent = m_wrist.getStatorCurrent().getValueAsDouble();   
        SmartDashboard.putNumber("Wrist Stator Current", statorCurrent);           
    }

}