package com.team4099.robot2023.subsystems.manipulator

import com.team4099.lib.hal.Clock
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.GamePiece
import com.team4099.robot2023.config.constants.ManipulatorConstants
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

class Manipulator(private val io: ManipulatorIO){
  val inputs = ManipulatorIO.ManipulatorIOInputs()

  var lastRollerRunTime = Clock.fpgaTime

  var lastHeldGamePieceDetected  = Clock.fpgaTime

  var lastHeldGamePiece = GamePiece.NONE

  var lastDropTime = Clock.fpgaTime

  var rumbleTrigger = false

  var rumbleTime = 0.5.seconds

  val hasCube: Boolean
    get() {
      return inputs.rollerVelocity.absoluteValue <= ManipulatorConstants.CONE_ROTATION_THRESHOLD &&
        inputs.rollerStatorCurrent > 10.amps &&
        inputs.rollerAppliedVoltage.sign < 0 &&
        (Clock.fpgaTime - lastRollerRunTime) >=
        ManipulatorConstants.MANIPULATOR_WAIT_BEFORE_DETECT_VELOCITY_DROP ||
        inputs.isSimulating
    }

  val hasCone: Boolean
    get() {
      return (
        inputs.rollerVelocity.absoluteValue <= ManipulatorConstants.CONE_ROTATION_THRESHOLD &&
          inputs.rollerStatorCurrent > 10.amps &&
          inputs.rollerAppliedVoltage.sign < 0 &&
          (Clock.fpgaTime - lastRollerRunTime) >=
          ManipulatorConstants.MANIPULATOR_WAIT_BEFORE_DETECT_VELOCITY_DROP
        ) ||
        inputs.isSimulating
    }

  val holdingGamePiece: GamePiece
    get() {
      if (hasCone) {
        return GamePiece.CONE
      }
      if (hasCube) {
        return GamePiece.CUBE
      }
      return GamePiece.NONE
    }

  var rollerVoltageTarget: ElectricalPotential = 0.0.volts

  private var lastRollerVoltage = 0.0.volts


  init {
  }

  fun periodic() {
    io.updateInputs(inputs)

    if (lastHeldGamePiece != holdingGamePiece && !rumbleTrigger) {
      rumbleTrigger = true
      lastDropTime = Clock.fpgaTime
    }

    if (Clock.fpgaTime - lastDropTime > rumbleTime) {
      rumbleTrigger = false
    }

    lastHeldGamePiece = holdingGamePiece

    Logger.getInstance().recordOutput("Manipulator/gamePieceRumble", rumbleTrigger)

    Logger.getInstance().processInputs("Manipulator", inputs)

    if (Constants.Tuning.DEBUGING_MODE) {

      Logger.getInstance().recordOutput("Manipulator/hasCube", hasCube)

      Logger.getInstance().recordOutput("Manipulator/hasCone", hasCone)

      Logger.getInstance()
        .recordOutput("Manipulator/lastRollerRunTime", lastRollerRunTime.inSeconds)

      Logger.getInstance()
        .recordOutput("Manipulator/lastRollerVoltageTarget", lastRollerVoltage.inVolts)

      Logger.getInstance()
        .recordOutput("Manipulator/rollerVoltageTarget", rollerVoltageTarget.inVolts)

    }

  }

  /**
   * Sets the power of the roller in volts.
   *
   * @param rpm: The revolutions per minute of the roller as stated in radians per second.
   */
  fun setRollerVoltage(voltage: ElectricalPotential) {
    io.setRollerVoltage(voltage)
    Logger.getInstance().recordOutput("Manipulator/rollerTargetVoltage", voltage.inVolts)
  }


  /**
   * Sets the roller motor brake mode.
   *
   * @param brake A flag representing the state of the break mode and whether it should be on/off.
   */
  fun setRollerBrakeMode(brake: Boolean) {
    io.setRollerBrakeMode(brake)
  }




}
