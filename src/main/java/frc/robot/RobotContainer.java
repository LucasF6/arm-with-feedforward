// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.PivotCommand;
import frc.robot.subsystems.ExtendSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class RobotContainer {
  private final CommandJoystick m_joystick = new CommandJoystick(0);

  private final ExtendSubsystem m_extendSubsystem = new ExtendSubsystem();
  private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem(m_extendSubsystem::getPosition);

  public RobotContainer() {
    m_pivotSubsystem.setDefaultCommand(new PivotCommand(m_pivotSubsystem, m_joystick::getY));

    configureBindings();
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
