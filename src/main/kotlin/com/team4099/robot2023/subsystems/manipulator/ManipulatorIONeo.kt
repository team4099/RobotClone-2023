package com.team4099.robot2023.subsystems.manipulator

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.team4099.lib.math.clamp
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ManipulatorConstants
import com.team4099.robot2023.subsystems.falconspin.MotorChecker
import com.team4099.robot2023.subsystems.falconspin.MotorCollection
import com.team4099.robot2023.subsystems.falconspin.Neo
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.asDrivenOverDriving
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.sparkMaxAngularMechanismSensor
import kotlin.math.absoluteValue

object ManipulatorIONeo : ManipulatorIO {
  private val rollerSparkMax = CANSparkMax(Constants.Manipulator.ROLLER_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless)

  private val rollerSensor = sparkMaxAngularMechanismSensor(
    rollerSparkMax,
    ManipulatorConstants.ROLLER_GEAR_RATIO.asDrivenOverDriving,
    ManipulatorConstants.ROLLER_VOLTAGE_COMPENSATION
  )

 init {
   rollerSparkMax.restoreFactoryDefaults()
   rollerSparkMax.clearFaults()

   rollerSparkMax.enableVoltageCompensation(ManipulatorConstants.ROLLER_VOLTAGE_COMPENSATION.inVolts)
   rollerSparkMax.setSmartCurrentLimit(
     ManipulatorConstants.ROLLER_CURRENT_LIMIT.inAmperes.toInt()
   )
   rollerSparkMax.inverted = ManipulatorConstants.ROLLER_MOTOR_INVERTED

   rollerSparkMax.idleMode = CANSparkMax.IdleMode.kCoast

   rollerSparkMax.openLoopRampRate = 0.0
   rollerSparkMax.burnFlash()

   MotorChecker.add(
     "Manipulator",
     "Roller",
     MotorCollection(
       mutableListOf(Neo(rollerSparkMax, "Roller Motor")),
       ManipulatorConstants.ROLLER_CURRENT_LIMIT,
       70.celsius,
       ManipulatorConstants.ROLLER_CURRENT_LIMIT - 0.amps,
       90.celsius
     ),
   )

 }

  override fun updateInputs(inputs: ManipulatorIO.ManipulatorIOInputs) {
    inputs.rollerVelocity = rollerSensor.velocity
    inputs.rollerAppliedVoltage = rollerSparkMax.busVoltage.volts * rollerSparkMax.appliedOutput
    inputs.rollerStatorCurrent = rollerSparkMax.outputCurrent.amps

    // BusVoltage * SupplyCurrent = AppliedVoltage * StatorCurrent
    // AppliedVoltage = percentOutput * BusVoltage
    // SupplyCurrent = (percentOutput * BusVoltage / BusVoltage) * StatorCurrent =
    // percentOutput * statorCurrent
    inputs.rollerSupplyCurrent =
      inputs.rollerStatorCurrent * rollerSparkMax.appliedOutput.absoluteValue
    inputs.rollerTemp = rollerSparkMax.motorTemperature.celsius

  }

  /**
   * Sets the roller motor voltage, ensures the voltage is limited to battery voltage compensation
   *
   * @param voltage the voltage to set the roller motor to
   */
  override fun setRollerVoltage(voltage: ElectricalPotential) {
    rollerSparkMax.setVoltage(
      clamp(
        voltage,
        -ManipulatorConstants.ROLLER_VOLTAGE_COMPENSATION,
        ManipulatorConstants.ROLLER_VOLTAGE_COMPENSATION
      )
        .inVolts
    )
  }

  /**
   * Sets the roller motor brake mode
   *
   * @param brake if it brakes
   */
  override fun setRollerBrakeMode(brake: Boolean) {
    if (brake) {
      rollerSparkMax.idleMode = CANSparkMax.IdleMode.kBrake
    } else {
      rollerSparkMax.idleMode = CANSparkMax.IdleMode.kCoast
    }
  }


}
