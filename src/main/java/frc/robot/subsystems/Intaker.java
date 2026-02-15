// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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
public class Intaker extends SubsystemBase /** Creates a new Shooter. */
{
  
  public SparkMax intakemotor;
  
  public Intaker() 
  {
    intakemotor = new SparkMax(CanIDs.INTAKER_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig intakemotorConfig = new SparkMaxConfig()
    {{
        smartCurrentLimit(20);
        this.advanceCommutation(60);
        this.signals.primaryEncoderVelocityAlwaysOn(true);
         this.idleMode(IdleMode.kBrake);
    }};
    

    intakemotor.configure(intakemotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  this.setDefaultCommand //default command
  (
    new FunctionalCommand
    (
    () -> intakemotor.set(0),
    () -> {},
    (killed) -> {},
    () -> {return false;},
    this)
    );

    SmartDashboard.putData("Subsystem/Intaker",this);

  }

  @Override
  public void periodic() // This method will be called once per scheduler run 
  {
    
  }



  public Command intake() 
    {
      return new ParallelCommandGroup
      (
        new InstantCommand(() -> intakemotor.set(0.8), this).repeatedly() 

      ).withName("Intake");
    
    }

    public Command outake() 
    {
      return new ParallelCommandGroup
      (
        new InstantCommand(() -> intakemotor.set(-0.8), this).repeatedly()
      
      ).withName("Outtake");
    
    }


}
