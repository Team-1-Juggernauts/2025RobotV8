// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

public class CANdleSubsystem extends SubsystemBase 
{

  private final CANdle m_candle = new CANdle(36);
  private final int LedCount = 300;
  private Animation m_toAnimate = null;

  public CANdleSubsystem() 
  {  
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.RGB;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    m_candle.configAllSettings(configAll, 100);
  }

  public void CANdle_init() 
  {  
  
  }
 
  public void CANdle_off() 
  {
    m_toAnimate = new StrobeAnimation(0, 0, 0, 0, 0, LedCount, 8);
  }
  
  public void CANdle_Animate() 
  {
    m_candle.animate(m_toAnimate);
  }
  
  public void CANdle_Yellow() 
  {  
    m_toAnimate = new StrobeAnimation(100, 100, 0, 0, 1.0, LedCount, 0);
  }

  public void CANdle_Fast_Yellow() 
  {  
    m_toAnimate = new StrobeAnimation(100, 100, 0, 0, 0.1, LedCount, 0);
  }

  public void CANdle_Orange() 
  {  
    m_toAnimate = new StrobeAnimation(150, 25 , 0, 0, 1, LedCount, 0);
  }

   public void CANdle_Default() 
  {  
    m_toAnimate = new StrobeAnimation(100, 100, 0, 0, 1.0, LedCount, 0);
    
  }

  public void CANdle_Rainbow() 
  {  
    m_toAnimate = new RainbowAnimation(0.5, 0.5, LedCount, false, 0);
  }

  public void CANdle_Solid_Green() 
  {  
    m_toAnimate = new StrobeAnimation(0, 100, 0, 0, 1.0, LedCount, 0);
  }
  public void CANdle_Red() 
  {  
    m_toAnimate = new StrobeAnimation(120, 0, 0, 0, 0.5, LedCount, 0);
  }

  public void CANdle_Blue() 
  {  
    m_toAnimate = new StrobeAnimation(0, 0, 120, 0, 1, LedCount, 0);
  }

  public void CANdle_Yellow_Larson() 
  {  
    m_toAnimate = new LarsonAnimation(100, 100, 0, 0, 1.0, LedCount, BounceMode.Front, 7, 0);
  }

  public void CANdle_Fire_Animation() 
  {  
    m_toAnimate = new FireAnimation(1, 0.5, LedCount, 0.75, 0.1, true, 0);
  }
  public void CANdle_Solid_White() 
  {  
    m_toAnimate = new StrobeAnimation(100, 100, 100, 100, 1.0, LedCount, 0);
  }

    @Override
  public void periodic() 
  {  
    
    m_candle.animate(m_toAnimate);

  }
}
