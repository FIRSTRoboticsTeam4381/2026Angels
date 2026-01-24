// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.commands.SparkPositionProfiled;

public class IntakePivot extends SubsystemBase {
  
  public SparkFlex pivot;
  public SparkFlex hopperslide;
  
  public IntakePivot() 
  {
    pivot = new SparkFlex(45, MotorType.kBrushless);
    hopperslide = new SparkFlex(46, MotorType.kBrushless);

    SparkFlexConfig pivotConfig = new SparkFlexConfig(){{
      smartCurrentLimit(40);
      closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    }};

    SparkFlexConfig hopperslideConfig = new SparkFlexConfig(){{
      smartCurrentLimit(40);
      closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
      this.softLimit.forwardSoftLimit(45);//45 to be changed
    }};

    pivot.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    hopperslide.configure(hopperslideConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command hoppertoPosition(double target, double range) {
    return new SparkPositionProfiled(hopperslide, target, range, this);
  }

  public Command pivottoPosition(double target, double range) {
    return new SparkPositionProfiled(pivot, target, range, this);
  }
  //all numbers will be changed
  public Command fullopen() {
    return new ParallelCommandGroup(
      hoppertoPosition(45, 0.2),
      pivottoPosition(20, 0.2)
    );
  }

  public Command fullclose() {
    return new ParallelCommandGroup(
      hoppertoPosition(0, 0.2),
      pivottoPosition(0, 0.2)
    );
  }

  public Command halfopen() {
    return new ParallelCommandGroup(
      hoppertoPosition(45, 0.2),
      pivottoPosition(0, 0.2)
    );
  }






}
