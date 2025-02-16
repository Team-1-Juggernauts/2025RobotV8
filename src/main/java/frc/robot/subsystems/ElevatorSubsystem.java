
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.FileWriter;
import java.io.IOException;


 public class ElevatorSubsystem extends SubsystemBase 
{
  private double kP = 1.0;  // Proportional gain
  private double kI = 0;  // Integral gain
  private double kD = 0.1;  // Derivative gain
  private double kS = 0.25;  // Static friction compensation
  private double kV = 0.12;  // Velocity term for overcoming friction

  private double elevHomePos = 0.0;
  public double positionSetPoint=0.0;
  private final double resetThreshold = 2.0;
 
  private final TalonFX m_elevator = new TalonFX(15);
  private final TalonFX m_elevator2 = new TalonFX(16);
  private final CANcoder range = new CANcoder(17);
  private final TalonFXConfiguration configs = new TalonFXConfiguration();  

  private FileWriter logFile;
    
  public  ElevatorSubsystem()
  {
    m_elevator2.setControl(new com.ctre.phoenix6.controls.Follower(15, true));  
 
    m_elevator.setNeutralMode(NeutralModeValue.Brake);
    //m_elevator2.setNeutralMode(NeutralModeValue.Brake);
   
    var slot0Configs = new Slot0Configs();
    
    slot0Configs.kP = kP; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = kI; 
    slot0Configs.kD = kD; 
    slot0Configs.kS = kS; // Add 0.1 V output to overcome static friction
    slot0Configs.kV = kV; // A velocity target of 1 rps results in 0.12 V output
    m_elevator.getConfigurator().apply(slot0Configs);
    //m_elevator2.getConfigurator().apply(slot0Configs);
    
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;    
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
    m_elevator.getConfigurator().apply(configs);
    //m_elevator2.getConfigurator().apply(configs);
    
    initLogFile();

  }

private void initLogFile()
     {
       try
       {
        logFile = new FileWriter("/home/lvuser/elevator_log.csv");
        logFile.append("Timestamp,Position,Stator Current,kP,kI,kD,kV,kS,SetPoint,\n");
       } 

        catch (IOException e) 
          {
          System.out.println("Error initializing log file: " + e.getMessage());
          }
  }

  public void setElevatorPosition(double positionSetPoint)
  {
    this.positionSetPoint = positionSetPoint;
        PositionVoltage request = new PositionVoltage(positionSetPoint).withSlot(0);
        m_elevator.setControl(request);  // Set control mode for the main motor       
   }

   public double getPosition()
  {    
    return (m_elevator.getPosition().getValueAsDouble());
  }
 
  public void setElevatorCoast()
  {    
    m_elevator.setNeutralMode(NeutralModeValue.Coast);     
    m_elevator2.setNeutralMode(NeutralModeValue.Coast);    
  }

  public void setElevatorBrake()
  {
    m_elevator.setNeutralMode(NeutralModeValue.Brake );     
    m_elevator2.setNeutralMode(NeutralModeValue.Brake );   
  }

public void ResetElevatorIfAtBottom()
{
double elevPos = range.getPosition().getValueAsDouble();
if (elevPos <= resetThreshold)
    {
      m_elevator.setPosition(elevHomePos);
    }
}

  @Override
  public void periodic() 
  {  
    ResetElevatorIfAtBottom();

    double position = m_elevator.getPosition().getValueAsDouble();
    SmartDashboard.putNumber("Elevator Setpoint", positionSetPoint);
    double statorCurrent = m_elevator.getStatorCurrent().getValueAsDouble();
    SmartDashboard.putNumber("Elevator Position", position);
    SmartDashboard.putNumber("Elevator Stator Current", statorCurrent);
    SmartDashboard.putNumber("Elevator Rotor position", m_elevator.getRotorPosition().getValueAsDouble());
  
    logData(position, statorCurrent);

  }

    private void logData(double position, double current) {
      try {
          long timestamp = System.currentTimeMillis();
          logFile.append(timestamp + "," + position + "," + current + "," + kP + "," + kI + "," + kD + "," + kV + "," + kS +  "," + positionSetPoint + "\n");
          logFile.flush();
      } catch (IOException e) {
          System.out.println("Error writing to log file: " + e.getMessage());
      }
  }

  public void closeLogFile() {
      try {
          if (logFile != null) {
              logFile.close();
          }
      } catch (IOException e) {
          System.out.println("Error closing log file: " + e.getMessage());
      }
  }
}
