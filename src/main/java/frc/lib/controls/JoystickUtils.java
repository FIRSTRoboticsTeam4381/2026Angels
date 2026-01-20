// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.controls;

import java.util.function.Supplier;

/** Utility class for  */
public class JoystickUtils {

  /**
   * Smooths joystic input for easier precice control without sacrificing full power.
   * @param in Input from joystic
   * @param deadzone Joystick deadzone
   * @return A supplier for the transformed output
   */
  public static Supplier<Double> interpolateJoystick(Supplier<Double> in, double deadzone)
  {
    return () -> JoystickUtils.interpolateNow(in.get(), deadzone);
  }

  /**
   * Smooths joystic input for easier precice control without sacrificing full power.
   * @param in Input from joystic
   * @param deadzone Joystick deadzone
   * @return The transformed output
   */
  public static double interpolateNow(double in, double deadzone)
  {
    if(Math.abs(in) < deadzone)
        return 0.0;
    else if (in>0)
        return Math.pow((in - deadzone)*(1.0/(1.0-deadzone)), 3);
    else 
        return -Math.pow((-in - deadzone)*(1.0/(1.0-deadzone)), 3);
  }


  /**
   * This function fixes joysticks where an axis doesn't quite reach
   * 1.00 or -1.00 (but correctly rests at 0) by stretching the positive
   * and/or negative range of the joystick based on the min/max value it reads.
   * @param in Supplier for raw joystick value
   * @param min Minimum measured joystick value. Must be between -1.00 and 0.
   * @param max Maximum measured joystick value. Must be between 0 and 1.00.
   * @return Supplier which resolves to a "fixed" joystick value that reaches
   * from -1.00 to 1.00.
   */
  public static Supplier<Double> stretchJoystick(Supplier<Double> in, double min, double max)
  {
    return () -> {
      double x = in.get();
      if(x>0)
        // Stretch positive joystick value
        return x * (1d / max);
      else if(x < 0)
        // Stretch negative joystick value
        return x * (1d / -min);
      else
        return 0d;
    };
  }

}