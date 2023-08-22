package com.team4099.robot2023.subsystems.groundintake

import com.team4099.lib.math.clamp
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.GroundIntakeConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import org.team4099.lib.controller.PIDController
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.asDrivingOverDriven
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

object GroundIntakeIOSim : GroundIntakeIO {
  private val rollerSim = FlywheelSim(
    DCMotor.getNEO(1),
    GroundIntakeConstants.ROLLER_GEAR_RATIO.asDrivingOverDriven,
    GroundIntakeConstants.ROLLER_MOMENT_INERTIA.inKilogramsMeterSquared
  )

  private val armSim = SingleJointedArmSim(
    DCMotor.getNEO(1),
    GroundIntakeConstants.ARM_GEAR_RATIO.asDrivingOverDriven,
    GroundIntakeConstants.ARM_MOMENT_INERTIA.inKilogramsMeterSquared,
    GroundIntakeConstants.ARM_LENGTH.inMeters,
    GroundIntakeConstants.ARM_MIN_ROTATION.inRadians,
    GroundIntakeConstants.ARM_MAX_ROTATION.inRadians,
    true
  )

  private val armController =
    PIDController(
      GroundIntakeConstants.PID.SIM_KP,
      GroundIntakeConstants.PID.SIM_KI,
      GroundIntakeConstants.PID.SIM_KD
    )

  private var rollerAppliedVoltage = 0.volts

  private var armVoltage = 0.volts

  override fun updateInputs(inputs: GroundIntakeIO.GroundIntakeIOInputs) {
    rollerSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    armSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.rollerVelocity = rollerSim.angularVelocityRPM.rotations.perMinute
    inputs.rollerTemp = 0.celsius
    inputs.rollerStatorCurrent = 0.amps
    inputs.rollerSupplyCurrent = 0.amps
    inputs.rollerAppliedVoltage = rollerAppliedVoltage

    inputs.armPosition = armSim.angleRads.radians
    inputs.armVelocity = armSim.velocityRadPerSec.radians.perSecond
    inputs.armStatorCurrent = 0.amps
    inputs.armSupplyCurrent = 0.amps
    inputs.armAppliedVoltage = armVoltage
    inputs.armTemp = 0.celsius

    inputs.isSimulated = true
  }

  override fun setRollersVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
      clamp(
        voltage,
        -GroundIntakeConstants.ROLLER_VOLTAGE_COMPENSATION,
        GroundIntakeConstants.ROLLER_VOLTAGE_COMPENSATION
      )

    rollerAppliedVoltage = clampedVoltage
    armSim.setInputVoltage(clampedVoltage.inVolts)
  }

  override fun setArmVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
      clamp(
        voltage,
        -GroundIntakeConstants.ARM_VOLTAGE_COMPENSATION,
        GroundIntakeConstants.ARM_VOLTAGE_COMPENSATION
      )

    armVoltage = clampedVoltage
    armSim.setInputVoltage(clampedVoltage.inVolts)
  }

  override fun setArmPosition(armPosition: Angle, feedforward: ElectricalPotential) {
    val feedback = armController.calculate(armSim.angleRads.radians, armPosition)
    setArmVoltage(feedforward + feedback)
  }

  override fun configPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    armController.setPID(kP, kI, kD)
  }


  override fun zeroEncoder() {}

  override fun setRollerBrakeMode(brake: Boolean) {}

  override fun setArmBrakeMode(brake: Boolean) {}

}
