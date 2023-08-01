package com.team4099.robot2023.subsystems.manipulator

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

interface ManipulatorIO {

  class ManipulatorIOInputs : LoggableInputs {
    var rollerVelocity = 0.0.rotations.perMinute
    var rollerAppliedVoltage = 0.0.volts
    var rollerSupplyCurrent = 0.0.amps
    var rollerStatorCurrent = 0.0.amps
    var rollerTemp = 0.0.celsius
    var isSimulating = false

    override fun toLog(table: LogTable?) {

      table?.put("rollerVelocityRPM", rollerVelocity.inRotationsPerMinute)

      table?.put("rollerAppliedVoltage", rollerAppliedVoltage.inVolts)

      table?.put("rollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)

      table?.put("rollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)

      table?.put("rollerTempCelsius", rollerTemp.inCelsius)
    }

    override fun fromLog(table: LogTable?) {

      table?.getDouble("rollerVelocityRPM", rollerVelocity.inRotationsPerMinute)?.let {
        rollerVelocity = it.rotations.perSecond
      }
      table?.getDouble("rollerAppliedVoltage", rollerAppliedVoltage.inVolts)?.let {
        rollerAppliedVoltage = it.volts
      }
      table?.getDouble("rollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)?.let {
        rollerSupplyCurrent = it.amps
      }
      table?.getDouble("rollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)?.let {
        rollerStatorCurrent = it.amps
      }
      table?.getDouble("rollerTempCelsius", rollerTemp.inCelsius)?.let { rollerTemp = it.celsius }
    }
  }

  fun updateInputs(inputs: ManipulatorIOInputs) {}

  /**
   * Sets the voltage of the roller motor but also checks to make sure the voltage doesn't exceed
   * limit
   *
   * @param voltage the voltage to set the motor to
   */
  fun setRollerVoltage(voltage: ElectricalPotential) {}

  /**
   * Sets the roller motor brake mode
   *
   * @param brake if it brakes
   */
  fun setRollerBrakeMode(brake: Boolean) {}
}
