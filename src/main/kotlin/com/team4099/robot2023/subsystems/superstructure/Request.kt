package com.team4099.robot2023.subsystems.superstructure

import com.team4099.robot2023.subsystems.groundintake.GroundIntake
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential

sealed interface Request {

  sealed interface GroundIntakeRequest : Request {
    class OpenLoop(val rollerVoltage: ElectricalPotential, val armVoltage: ElectricalPotential) : GroundIntakeRequest
    class TargetingPosition(val rollerVoltage: ElectricalPotential, val armPosition: Angle) : GroundIntakeRequest
    class ZeroArm() : GroundIntakeRequest
  }
}


