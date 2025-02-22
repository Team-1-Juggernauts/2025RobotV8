// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase 
{
  
  final TalonFX _ClimberTalonFX = new TalonFX(35,"Chassis CAN");
  final DutyCycleOut m_leftRequest = new DutyCycleOut(0.0);    
 
  public ClimbSubsystem() 
  {
    
    _ClimberTalonFX.setNeutralMode(NeutralModeValue.Brake);
              
   }
  
  public void setClimbSpeed(double speed)
  {
    _ClimberTalonFX.setControl(m_leftRequest.withOutput(speed));
  }

  @Override
  public void periodic() 
  {

    SmartDashboard.putNumber("Climb Out Percent", _ClimberTalonFX.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("Climb Current", _ClimberTalonFX.getSupplyCurrent().getValueAsDouble());

  }
}
