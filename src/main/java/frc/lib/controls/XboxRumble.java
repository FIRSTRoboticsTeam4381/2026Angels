// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.controls;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Utility class with Commands to rumble CommandXboxControllers. */
public class XboxRumble {

    /**
   * Rumble a controller while a command is running, stopping when the command finishes
   * @param controller Controller to rumble
   * @param rumbleside Which half of the controller to rumble, or both
   * @param rumblestrength Strength of the rumble, from 0.0 to 1.0
   * @param c Command to run while rumbling
   * @return Instant command to set rumble strength
   */
  public static Command vibrateWhile(CommandXboxController controller, RumbleType rumbleside, double rumblestrength, Command c)
  {
    return new ParallelRaceGroup(
      c,
      new FunctionalCommand(() -> controller.setRumble(rumbleside, rumblestrength),
      () -> controller.setRumble(rumbleside, rumblestrength),
      (interrupted) -> controller.setRumble(rumbleside, 0),
      () -> {return false;})
    );
  }

  

  /**
   * Set a controller to rumble for a certain amount of time.
   * This isn't blocking- it schedules a separate command to end the rumbe later.
   * @param controller controller to rumble
   * @param rumbleside Which half of the controller to rumble, or both
   * @param rumblestrength Strength of the rumble, from 0.0 to 1.0
   * @param time How long to rumble for
   * @return Command to schedule the rumble
   */
  public static Command vibrateForTime(CommandXboxController controller,RumbleType rumbleside, double rumblestrength, double time)
  {
    return new ScheduleCommand(vibrateWhile(controller, rumbleside, rumblestrength, new WaitCommand(time)));
  }

}

