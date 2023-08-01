package com.team4099.robot2023.subsystems.manipulator

import com.team4099.lib.math.clamp
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ManipulatorConstants
import com.team4099.robot2023.subsystems.falconspin.MotorChecker
import com.team4099.robot2023.subsystems.falconspin.MotorCollection
import com.team4099.robot2023.subsystems.falconspin.SimulatedMotor
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.asDrivenOverDriving
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ManipulatorIOSim : ManipulatorIO {

  val rollerSim: FlywheelSim =
    FlywheelSim(
      DCMotor.getNEO(1),
      ManipulatorConstants.ROLLER_GEAR_RATIO.asDrivenOverDriving,
      ManipulatorConstants.MOMENT_INERTIA.inKilogramsMeterSquared
    )

  init {
    MotorChecker.add(
      "Manipulator",
      "Roller",
      MotorCollection(
        mutableListOf(
          SimulatedMotor(
            rollerSim,
            "Roller Motor",
          )
        ),
        60.amps,
        10.celsius,
        45.amps,
        20.celsius
      )
    )
  }

  override fun updateInputs(inputs: ManipulatorIO.ManipulatorIOInputs) {
    rollerSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.rollerVelocity = rollerSim.angularVelocityRPM.radians.perSecond
    inputs.rollerSupplyCurrent = 0.0.amps
    inputs.rollerAppliedVoltage = 0.volts
    inputs.rollerStatorCurrent = rollerSim.currentDrawAmps.amps
    inputs.rollerTemp = 0.0.celsius

    inputs.isSimulating = true

    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(rollerSim.currentDrawAmps)
    )
  }

  /**
   * Sets the voltage of the roller motor using a clamp to limit between min and max volts
   *
   * @param voltage the voltage to set the motor to
   */
  override fun setRollerVoltage(voltage: ElectricalPotential) {
    rollerSim.setInputVoltage(
      clamp(
        voltage,
        -ManipulatorConstants.ROLLER_VOLTAGE_COMPENSATION,
        ManipulatorConstants.ROLLER_VOLTAGE_COMPENSATION
      )
        .inVolts
    )
  }
}
