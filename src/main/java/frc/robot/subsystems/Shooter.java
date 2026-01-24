// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

/** Creates a new Shooter. */
public SparkFlex shooter;


  public Shooter() {
  shooter = new SparkFlex(0, MotorType.kBrushless); 

    SparkFlexConfig shooterConfig = new SparkFlexConfig()
    {{
        this.smartCurrentLimit(40);
        softLimit.forwardSoftLimit(0.25);
    }};

    shooter.configure(shooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    this.setDefaultCommand(
      new FunctionalCommand(() -> shooter.set(0), () -> {}, (killed) -> {}, () -> {return false;}, this));
    

  }





  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public Command shooterShoot()
    {
      return new InstantCommand(()-> shooter.set(0.5));
    }
}
