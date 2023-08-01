package com.team4099.robot2023.subsystems.superstructure

import org.team4099.lib.units.derived.ElectricalPotential

sealed interface Request {
  sealed interface SuperstructureRequest : Request

  sealed interface ManipulatorRequest : Request {
    class OpenLoop(val rollerVoltage: ElectricalPotential) : ManipulatorRequest
    class Brake : ManipulatorRequest
  }
}
