
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    private final TalonFX m_elevator = new TalonFX(15);  // Main elevator motor
    private final TalonFX m_elevator2 = new TalonFX(16);  // Follower motor
    private final DigitalInput DIO = new DigitalInput(0);  // Limit switch
    private final PositionVoltage positionControl = new PositionVoltage(0);
    

// PID constants
    private double kP = 2.0;  // Proportional gain
    private double kI = 0;  // Integral gain
    private double kD = 0.2;  // Derivative gain
    private double kS = 0.1;  // Static friction compensation
    private double kV = 0.1;  // Velocity term for overcoming friction

    private double positionSetPoint = 0.0;
    
    public ElevatorSubsystem() {
        // Set neutral mode for the elevator motors
        m_elevator.setNeutralMode(NeutralModeValue.Brake);
        m_elevator2.setNeutralMode(NeutralModeValue.Brake);
        m_elevator.setPosition(0);    

        var configs = new TalonFXConfiguration();
        configs.Voltage.PeakForwardVoltage = 6;
        configs.Voltage.PeakReverseVoltage = -6;
        configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        m_elevator.getConfigurator().apply(configs);
        m_elevator2.getConfigurator().apply(configs);

        var talonFXConfigurator = m_elevator.getConfigurator();        
        var limitConfigs = new CurrentLimitsConfigs();
          limitConfigs.StatorCurrentLimit = 50;
          limitConfigs.StatorCurrentLimitEnable = true;
          talonFXConfigurator.apply(limitConfigs);

        var talonFXConfigurator2 = m_elevator2.getConfigurator();
        var limitConfigs2 = new CurrentLimitsConfigs();
            limitConfigs2.StatorCurrentLimit = 50;
            limitConfigs2.StatorCurrentLimitEnable = true;
            talonFXConfigurator2.apply(limitConfigs2);

        // Configure the TalonFX for position control (PID)
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = kP;
        slot0Configs.kI  = kI;
        slot0Configs.kD = kD;
        slot0Configs.kS = kS;
        slot0Configs.kV = kV;
        m_elevator.getConfigurator().apply(slot0Configs);
        m_elevator2.getConfigurator().apply(slot0Configs);

         m_elevator2.setControl(new com.ctre.phoenix6.controls.Follower(15, true)); // Follower motor
    }

  //      public void setElevatorPosition(double positionSetPoint) 
  //  {
  //      this.positionSetPoint = positionSetPoint;
  //      PositionVoltage request = new PositionVoltage(positionSetPoint).withSlot(0);
  //      m_elevator.setControl(request);  // Set control mode for the main motor
  //  }

  public void setElevatorPosition(double positionSetPoint) {
    this.positionSetPoint = positionSetPoint;
    m_elevator.setControl(positionControl.withPosition(positionSetPoint));
}

    public double getPosition() {
        return m_elevator.getPosition().getValueAsDouble();
    }

    public void setElevatorBrake() {
        m_elevator.setNeutralMode(NeutralModeValue.Brake);
        m_elevator2.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setElevatorCoast() {
        m_elevator.setNeutralMode(NeutralModeValue.Coast);
        m_elevator2.setNeutralMode(NeutralModeValue.Coast);
    }

    public void holdPosition() {
        // Reapply the last known position to hold
        m_elevator.setControl(positionControl.withPosition(positionSetPoint));
    }

    @Override
    public void periodic() {

        if(!DIO.get())
        {
          m_elevator.setPosition(0);
          positionSetPoint = 0; 
        }
    
        // Continuously hold position
        m_elevator.setControl(positionControl.withPosition(positionSetPoint));
        
        SmartDashboard.putNumber("Elevator Position", m_elevator.getPosition().getValueAsDouble());       
        SmartDashboard.putNumber("Elevator Setpoint", positionSetPoint);
        double statorCurrent = m_elevator.getStatorCurrent().getValueAsDouble();   
        SmartDashboard.putNumber("Elevator Stator Current", statorCurrent);   
        SmartDashboard.putBoolean("Elevator DIO",  DIO.get());
        
    }
}

