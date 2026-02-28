// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CanIDs;

@Logged


public class Hang extends SubsystemBase {

  public SparkFlex hangMotor1;
  public SparkFlex hangMotor2;

  /** Creates a new Hang. */
  public Hang() 
  {
    hangMotor1 = new SparkFlex(CanIDs.HANG_1_MOTOR_ID, MotorType.kBrushless);
    hangMotor2 = new SparkFlex(CanIDs.HANG_2_MOTOR_ID, MotorType.kBrushless);

    SparkFlexConfig hangMotor1Config = new SparkFlexConfig()
    {{
      smartCurrentLimit(80);
      closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
      softLimit.forwardSoftLimit(0.732).reverseSoftLimit(0.1)
      .forwardSoftLimitEnabled(true ).reverseSoftLimitEnabled(true);
      this.signals.absoluteEncoderPositionAlwaysOn(true);
      this.signals.isAtSetpointAlwaysOn(true);
      this.signals.maxMotionSetpointPositionAlwaysOn(true);
      this.signals.maxMotionSetpointVelocityAlwaysOn(true);
      this.signals.setSetpointAlwaysOn(true);
      this.idleMode(IdleMode.kBrake);
      
      this.absoluteEncoder.inverted(true);
    }};

    SparkFlexConfig hangMotor2Config = new SparkFlexConfig()
    {{
       smartCurrentLimit(80);
       follow(hangMotor1, true);
       this.idleMode(IdleMode.kBrake);
    }};

  hangMotor1.configure(hangMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  hangMotor2.configure(hangMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public Command hangUp()
    {
      return new InstantCommand(()-> hangMotor1.set(0.5), this);
    }

  public Command hangDown()
    {
      return new InstantCommand(()-> hangMotor1.set(-0.5), this);
    }
public Command manualControl(Supplier <Double> joystick)
    {
      return new InstantCommand(() -> hangMotor1.set(joystick.get()), this).repeatedly().withName("joystick control");
    }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
