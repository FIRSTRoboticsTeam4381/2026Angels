// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CanIDs;
@Logged 
public class Agitator extends SubsystemBase {

  /** Creates a new Agitator. */
 public SparkMax agitator; 
 public SparkFlex funnel; 

 public Agitator(){

  agitator = new SparkMax(CanIDs.AGITATOR_MOTOR_ID, MotorType.kBrushless);
  funnel = new SparkFlex(CanIDs.FUNNEL_MOTOR_ID, MotorType.kBrushless);

  SparkMaxConfig agitatorConfig = new SparkMaxConfig()
  {{
      this.smartCurrentLimit(20);
      this.advanceCommutation(60);
      this.signals.primaryEncoderVelocityAlwaysOn(true);
       this.idleMode(IdleMode.kBrake);
     // this.encoder.
     this.inverted(true);
  }};

  SparkFlexConfig funnelConfig = new SparkFlexConfig()
  {{
      this.smartCurrentLimit(80);
      this.signals.primaryEncoderVelocityAlwaysOn(true);
       this.idleMode(IdleMode.kBrake);
  }};

  
    agitator.configure(agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    funnel.configure(funnelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

    NamedCommands.registerCommand("agitatorMove", agitatorMove());
    NamedCommands.registerCommand("shoot", agitatorFunnelMove());

   this.setDefaultCommand(
    new FunctionalCommand(() -> {
      agitator.set(0);
      funnel.set(0);
     }, () -> {}, (killed) -> {}, () -> {return false;}, this));
   
    SmartDashboard.putData("Subsystem/Agitator",this);
    



 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public Command agitatorFunnelMove(){
return new InstantCommand(
  ()-> {agitator.set(1); funnel.set(1);},this).withName("agitatorFunnelMove").repeatedly();
  
}

public Command agitatorMove(){
  return new InstantCommand(
  ()-> {agitator.set(1); funnel.set(0);},this).withName("agitatorMove").repeatedly();
}

public Command agitatorFunnelMoveReverse(){
return new InstantCommand(
  () -> {agitator.set(-1); funnel.set(-1);},this).withName("agitatorFunnelMoveReverse").repeatedly();
  
}}
