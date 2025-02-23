
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

 public class ShoulderSubsystem extends SubsystemBase 
{
  private double kP = 1.0;  // Proportional gain
  private double kI = 0;  // Integral gain
  private double kD = 0.1;  // Derivative gain
  private double kS = 0.25;  // Static friction compensation
  private double kV = 0.12;  // Velocity term for overcoming friction
 
  private double positionSetPoint=0.0;

  private final TalonFX m_shoulder = new TalonFX(18);  
  private final CANcoder m_cc = new CANcoder(19);
  public TalonFXConfiguration configs = new TalonFXConfiguration();
  public CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
  public TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
  private final PositionVoltage positionControl = new PositionVoltage(0);
    
  public  ShoulderSubsystem()
  {

cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
cc_cfg.MagnetSensor.MagnetOffset = 0.4;
m_cc.getConfigurator().apply(cc_cfg);


fx_cfg.Feedback.FeedbackRemoteSensorID = m_cc.getDeviceID();
fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
fx_cfg.Feedback.SensorToMechanismRatio = 1.0;
fx_cfg.Feedback.RotorToSensorRatio = 81.26984;
m_shoulder.getConfigurator().apply(fx_cfg);

m_shoulder.setNeutralMode(NeutralModeValue.Brake);

configs.Voltage.PeakForwardVoltage = 8;
configs.Voltage.PeakReverseVoltage = -8;
configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
m_shoulder.getConfigurator().apply(configs);


 var talonFXConfigurator = m_shoulder.getConfigurator();        
        var limitConfigs = new CurrentLimitsConfigs();
          limitConfigs.StatorCurrentLimit = 50;
          limitConfigs.StatorCurrentLimitEnable = true;
          talonFXConfigurator.apply(limitConfigs);




 // Configure the TalonFX for position control (PID)
 var slot0Configs = new Slot0Configs();
 slot0Configs.kP = kP;
 slot0Configs.kI = kI;
 slot0Configs.kD = kD;
 slot0Configs.kS = kS;
 slot0Configs.kV = kV;
 m_shoulder.getConfigurator().apply(slot0Configs);
 
 
 }
   

 //public void setShoulderPosition(double positionSetPoint)
//  this.positionSetPoint = positionSetPoint;
//        PositionVoltage request = new PositionVoltage(positionSetPoint).withSlot(0);
//        m_shoulder.setControl(request);  // Set control mode for the main motor      
//}

public void setShoulderPosition(double positionSetPoint) {
  this.positionSetPoint = positionSetPoint;
  m_shoulder.setControl(positionControl.withPosition(positionSetPoint));
}



   public void setShoulderSpeed(double percentOutput)
   {
     m_shoulder.set(percentOutput);  
   }
 
  public double getPosition()
  {    
    return (m_cc.getAbsolutePosition().getValueAsDouble());
  }
 
  public void setShoulderCoast()
  {    
    m_shoulder.setNeutralMode(NeutralModeValue.Coast);        
  }

  public void setShoulderBrake()
  {
         m_shoulder.setNeutralMode(NeutralModeValue.Brake );     
  }
 
   @Override
  public void periodic() 
  {  

// Continuously hold position
m_shoulder.setControl(positionControl.withPosition(positionSetPoint));

    double position = m_cc.getPosition().getValueAsDouble();
    double statorCurrent = m_shoulder.getStatorCurrent().getValueAsDouble();
    
    SmartDashboard.putNumber("Shoulder CANCoder position", m_cc.getAbsolutePosition().getValueAsDouble()); 
    SmartDashboard.putNumber("Shoulder position", position); 
    SmartDashboard.putNumber("Shoulder Set Point", positionSetPoint);
    SmartDashboard.putNumber("Shoulder Stator Current", statorCurrent);
  
  }

 
}
