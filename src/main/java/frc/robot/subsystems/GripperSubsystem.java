// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperSubsystem extends SubsystemBase 
{
   
final TalonFX _GripperTalonFX = new TalonFX(25);
    
	 
public GripperSubsystem() 
  {
    configureMotor();
    _GripperTalonFX.setNeutralMode(NeutralModeValue.Brake);
              
   }
  
public void configureMotor()
  {
  CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
     currentLimits.StatorCurrentLimit = 20;
     currentLimits.StatorCurrentLimitEnable = true;     
     _GripperTalonFX.getConfigurator().apply(currentLimits);
  }

public void setGripperSpeed(double speed)
  {
    _GripperTalonFX.set(speed);
  }
    
public double getStatorCurrent()
   {
    return _GripperTalonFX.getStatorCurrent().getValueAsDouble(); 
   }




  @Override
  public void periodic() 
  {
    SmartDashboard.putNumber("Gripper Out Percent", _GripperTalonFX.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("Gripper Current", _GripperTalonFX.getSupplyCurrent().getValueAsDouble());
  }
}
