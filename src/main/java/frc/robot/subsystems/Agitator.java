// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Agitator extends SubsystemBase {




  


  /** Creates a new Agitator. */
 public SparkFlex agitator; 
 public SparkFlex funnel; 

 public Agitator(){
  agitator = new SparkFlex(0, MotorType.kBrushless);
  funnel = new SparkFlex(0, MotorType.kBrushless);

  SparkFlexConfig agitatorConfig = new SparkFlexConfig()
  {{
      this.smartCurrentLimit(40);
      softLimit.forwardSoftLimit(0.25);
  }};

  SparkFlexConfig funnelConfig = new SparkFlexConfig()
  {{
      this.smartCurrentLimit(40);
      softLimit.forwardSoftLimit(0.25);

  }};

  
    agitator.configure(agitatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    funnel.configure(funnelConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters); 

   this.setDefaultCommand(
    new FunctionalCommand(() -> agitator.set(0), () -> {}, (killed) -> {}, () -> {return false;}, this));
   

    
 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public Command agitatorFunnelMove(){
return new ParallelCommandGroup(
  new InstantCommand(()-> agitator.set(0.5)),
  new InstantCommand(()-> funnel.set(0.5))); 
}

public Command agitatorMove(){
return new InstantCommand(()-> agitator.set(0.5)); 
}



}
