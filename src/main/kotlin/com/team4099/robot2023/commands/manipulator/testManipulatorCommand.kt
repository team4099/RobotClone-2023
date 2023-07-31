package com.team4099.robot2023.commands.manipulator

import com.team4099.robot2023.subsystems.manipulator.Manipulator
import com.team4099.robot2023.subsystems.superstructure.Request
import edu.wpi.first.wpilibj2.command.CommandBase
import org.team4099.lib.units.derived.volts

class testManipulatorCommand(val manipulator: Manipulator) : CommandBase() {
  init {
    addRequirements(manipulator)
  }

  override fun initialize() {
    manipulator.currentRequest = Request.ManipulatorRequest.OpenLoop(10.volts)
  }

  override fun isFinished(): Boolean {
    return false
  }

  override fun end(interrupted: Boolean) {
    manipulator.currentRequest = Request.ManipulatorRequest.OpenLoop(0.volts)
  }
}
