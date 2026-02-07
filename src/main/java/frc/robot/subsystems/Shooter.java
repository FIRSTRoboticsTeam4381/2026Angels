// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AutoAim;
import frc.robot.CanIDs;



public class Shooter extends SubsystemBase {

/** Creates a new Shooter. */
public SparkFlex shooter1;
public SparkFlex shooter2;
public SparkFlex shooter3;
public SparkFlex shooter4;
public InterpolatingDoubleTreeMap shoottable;
public InterpolatingDoubleTreeMap passtable;

      public void shootSetUp()
        {
          shoottable.put(0.0, 0.0);
          shoottable.put(1.0, 10.0);
          shoottable.put(2.0, 30.0);
        }

      public void passSetUp()
        {
          passtable.put(0.0, 0.0);
          passtable.put(1.0, 10.0);
          passtable.put(2.0, 30.0);
        }

  public Shooter() {
  shooter1 = new SparkFlex(CanIDs.SHOOTER_1_MOTOR_ID, MotorType.kBrushless); 
  shooter2 = new SparkFlex(CanIDs.SHOOTER_2_MOTOR_ID, MotorType.kBrushless);
  shooter3 = new SparkFlex(CanIDs.SHOOTER_3_MOTOR_ID, MotorType.kBrushless);
  shooter4 = new SparkFlex(CanIDs.SHOOTER_4_MOTOR_ID, MotorType.kBrushless);
  shoottable = new InterpolatingDoubleTreeMap();
  passtable = new InterpolatingDoubleTreeMap();
  shootSetUp();
  passSetUp();

    SparkFlexConfig shooterConfig = new SparkFlexConfig()
    {{
        this.smartCurrentLimit(40);
    }};

     SparkFlexConfig shooter2Config = new SparkFlexConfig()
    {{
       apply(shooterConfig);
       follow(shooter1, true);
       //inverted(true);
       //makes shooter 2 ther same as shooter 1
    }};

  
    
    shooter1.configure(shooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    shooter2.configure(shooter2Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    shooter3.configure(shooter2Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    shooter4.configure(shooter2Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    this.setDefaultCommand(
      new FunctionalCommand(() -> shooter1.set(0), () -> {}, (killed) -> {}, () -> {return false;}, this));
  
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
  }




  public Command shooterShoot()
    {
      return new InstantCommand(()-> shooter1.set(0.5), this);
      //gives 0.5 current to the motor 
    }

  private void setVelocity(double velocity)
    {
        shooter1.getClosedLoopController().setSetpoint(velocity, ControlType.kVelocity);
        // keeps the speed around the same speed 
    }

  public double shootVelocityFromDistance()
  {
    double distance = AutoAim.distanceToHub();
      return shoottable.get(distance);
  }

  public double passVelocityFromDistance()
  {
    double distance = AutoAim.distanceToPass(); //change distance to hub with another distance variable
      return passtable.get(distance);
  }

  public Command setShootVelocityFromDistance()
      {
        return new InstantCommand(() -> setVelocity(shootVelocityFromDistance()), this).repeatedly();
      }

  public Command setPassVelocityFromDistance()
      {
        return new InstantCommand(() -> setVelocity(passVelocityFromDistance()), this).repeatedly();
      }
 public Command setVelocity(Supplier<Double> supplier)
      {
        return new InstantCommand(() -> setVelocity(supplier.get()), this).repeatedly();
      }





}
