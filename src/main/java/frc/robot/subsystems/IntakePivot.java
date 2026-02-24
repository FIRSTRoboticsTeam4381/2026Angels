// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.commands.SparkPositionProfiled;
import frc.lib.commands.SparkSysIDTest;
import frc.robot.CanIDs;
@Logged
public class IntakePivot extends SubsystemBase {
  
  public SparkFlex pivot;
  //public SparkFlex hopperslide;
  
  public IntakePivot() 
  {
    NamedCommands.registerCommand("pivotDown", down());
    NamedCommands.registerCommand("pivotUp", up());

    pivot = new SparkFlex(CanIDs.INTAKE_PIVOT_MOTOR_ID, MotorType.kBrushless);
   // hopperslide = new SparkFlex(CanIDs.HOODED_MOTOR_MOTOR_ID, MotorType.kBrushless);

    SparkFlexConfig pivotConfig = new SparkFlexConfig(){{
      smartCurrentLimit(40);
      //limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);
      closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
      this.signals.primaryEncoderPositionAlwaysOn(true);
      this.signals.absoluteEncoderPositionAlwaysOn(true);
      this.signals.isAtSetpointAlwaysOn(true);
      this.signals.maxMotionSetpointPositionAlwaysOn(true);
      this.signals.maxMotionSetpointVelocityAlwaysOn(true);
      this.signals.setSetpointAlwaysOn(true);
      this.idleMode(IdleMode.kBrake);
      this.inverted(true);
      this.absoluteEncoder.zeroCentered(true);
      this.absoluteEncoder.inverted(true);
      this.softLimit.forwardSoftLimit(0.265).forwardSoftLimitEnabled(true);
      this.softLimit.reverseSoftLimit(0).reverseSoftLimitEnabled(true);
      closedLoop.feedForward.sva(0.40968, 0.23589, 0.041185);
      closedLoop.p(0.056226);
      closedLoop.maxMotion.cruiseVelocity(14).maxAcceleration(200);
      closedLoop.maxMotion.allowedProfileError(0.05);

    }};

    SparkFlexConfig hopperslideConfig = new SparkFlexConfig(){{
      smartCurrentLimit(40);
      closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
      this.softLimit.forwardSoftLimit(45);//45 to be changed
      this.signals.primaryEncoderVelocityAlwaysOn(true);
       this.idleMode(IdleMode.kBrake);
    }};

    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //hopperslide.configure(hopperslideConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putData("Subsystem/IntakePivot",this);
    SmartDashboard.putData("SysID/IntakePivot",
      new SparkSysIDTest(pivot, this, 1, 0.01, 0.24, pivot.getAbsoluteEncoder()::getPosition));

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intakepivot/targetposition", pivot.getClosedLoopController().getSetpoint());
  }

 // public Command hoppertoPosition(double target, double range) {
 //   return new SparkPositionProfiled(hopperslide, target, range, this).withName("hoppertoPosition");
 // }

  public Command pivottoPosition(double target, double range) {
    return new SparkPositionProfiled(pivot, target, range, this).withName("pivottoPosition");
  }
  //all numbers will be changed
 /*public Command fullopen() {
    return new ParallelCommandGroup(
      //hoppertoPosition(45, 0.2),
      down()
    ).withName("fullopen");
  }

  public Command fullclose() {
    return new SequentialCommandGroup(
    up(),
    pivottoPosition(0, 0.05)
    ).withName("fullclose");
  }*/

 /*  public Command halfopen() {
    return new SequentialCommandGroup(
    up(),
    //hoppertoPosition(45, 0.2)
    ).withName("halfopen");
  } */


/*  public Command up()
 {
  return new FunctionalCommand
  (() -> pivot.set(1), 
  () -> {}, 
  (killed) -> 
  {
    pivot.configureAsync(new SparkFlexConfig().idleMode(IdleMode.kBrake),ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
    pivot.stopMotor();
  }, 
  () -> {return pivot.getForwardLimitSwitch().isPressed();}, 
  this).withName("Up");
}
*/

public Command up()
{
return new SequentialCommandGroup
(
  //new InstantCommand(() -> pivot.configureAsync(new SparkFlexConfig().idleMode(IdleMode.kBrake),ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters)),
  pivottoPosition(0.28, 0.01).until(pivot.getForwardSoftLimit()::isReached),
  new InstantCommand(() -> pivot.set(0), this) 
).withName("Up");
}

public Command down()
 {
  return new SequentialCommandGroup
  (
    //new InstantCommand(() -> pivot.set(-1)),
    //new InstantCommand(() -> pivot.configureAsync(new SparkFlexConfig().idleMode(IdleMode.kCoast),ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters)),
    //new WaitCommand(0.5),
    //new InstantCommand(() -> pivot.set(0))
    pivottoPosition(-0.1, 0.01).until(pivot.getReverseSoftLimit()::isReached),
   new  InstantCommand(() -> pivot.set(0), this )
  ).withName("Down");
}

}
