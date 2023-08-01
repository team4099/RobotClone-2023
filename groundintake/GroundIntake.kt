package com.team4099.robot2023.subsystems.groundintake

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableNumber
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.GroundIntakeConstants
import com.team4099.robot2023.subsystems.superstructure.Request
import edu.wpi.first.wpilibj.RobotBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ArmFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond
import com.team4099.robot2023.subsystems.superstructure.Request.GroundIntakeRequest as GroundIntakeRequest

class GroundIntake(private val io: GroundIntakeIO) {
  val inputs = GroundIntakeIO.GroundIntakeIOInputs()

  var armFeedforward: ArmFeedforward

  private val kP =
    LoggedTunableValue("GroundIntake/kP", Pair({ it.inVoltsPerDegree }, { it.volts.perDegree }))
  private val kI =
    LoggedTunableValue(
      "GroundIntake/kI", Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
    )
  private val kD =
    LoggedTunableValue(
      "GroundIntake/kD",
      Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
    )

  object TunableGroundIntakeStates {
    val enableArm =
      LoggedTunableNumber(
        "GroundIntake/enableArmIntake",
        GroundIntakeConstants.ENABLE_ARM,
      )

    val enableRotation =
      LoggedTunableNumber(
        "GroundIntake/enableRotationIntake",
        GroundIntakeConstants.ENABLE_ROTATION,
      )

    val intakeAngle =
      LoggedTunableValue(
        "GroundIntake/intakeAngle",
        GroundIntakeConstants.INTAKE_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val outtakeAngle =
      LoggedTunableValue(
        "GroundIntake/outtakeAngle",
        GroundIntakeConstants.OUTTAKE_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val stowedUpAngle =
      LoggedTunableValue(
        "GroundIntake/stowedUpAngle",
        GroundIntakeConstants.STOWED_UP_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val stowedDownAngle =
      LoggedTunableValue(
        "GroundIntake/stowedDownAngle",
        GroundIntakeConstants.STOWED_DOWN_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val intakeVoltage =
      LoggedTunableValue(
        "GroundIntake/intakeVoltage",
        GroundIntakeConstants.INTAKE_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )

    val outtakeVoltage =
      LoggedTunableValue(
        "GroundIntake/outtakeVoltage",
        GroundIntakeConstants.OUTTAKE_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )

    val neutralVoltage =
      LoggedTunableValue(
        "GroundIntake/neutralVoltage",
        GroundIntakeConstants.NEUTRAL_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )

    val helpScoreVoltage =
      LoggedTunableValue(
        "GroundIntake/helpScoreVoltage",
        GroundIntakeConstants.HELP_SCORE_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )
  }

  var armPositionTarget: Angle = 0.0.degrees

  var armVoltageTarget: ElectricalPotential = 0.0.volts

  var rollerVoltageTarget: ElectricalPotential = 0.0.volts

  var isZeroed = false

  private var lastArmPositionTarget = 0.0.degrees

  private var lastArmVoltage = 0.0.volts

  val forwardLimitReached: Boolean
    get() = inputs.armPosition >= GroundIntakeConstants.ARM_MAX_ROTATION
  val reverseLimitReached: Boolean
    get() = inputs.armPosition <= GroundIntakeConstants.ARM_MIN_ROTATION

  val openLoopForwardLimitReached: Boolean
    get() = inputs.armPosition >= GroundIntakeConstants.ARM_OPEN_LOOP_MAX_ROTATION

  val openLoopReverseLimitReached: Boolean
    get() = inputs.armPosition <= GroundIntakeConstants.ARM_OPEN_LOOP_MIN_ROTATION

  var lastIntakeRunTime = Clock.fpgaTime

  var currentState: GroundIntakeState = GroundIntakeState.UNINITIALIZED

  var currentRequest: GroundIntakeRequest = GroundIntakeRequest.ZeroArm()
    set(value) {
      when (value) {
        is GroundIntakeRequest.OpenLoop -> {
          armVoltageTarget = value.voltage
          rollerVoltageTarget = value.rollerVoltage
        }
        is GroundIntakeRequest.TargetingPosition -> {
          armPositionTarget = value.position
          rollerVoltageTarget = value.rollerVoltage
        }
        else -> {}
      }
      field = value
    }

  private var armConstraints: TrapezoidProfile.Constraints<Radian> =
    TrapezoidProfile.Constraints(
      GroundIntakeConstants.MAX_ARM_VELOCITY, GroundIntakeConstants.MAX_ARM_ACCELERATION
    )

  private var armProfile =
    TrapezoidProfile(
      armConstraints,
      TrapezoidProfile.State(armPositionTarget, 0.0.degrees.perSecond),
      TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)
    )

  private var timeProfileGeneratedAt = Clock.fpgaTime

  private var prevArmSetpoint: TrapezoidProfile.State<Radian> =
    TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)

  val isAtTargetedPosition: Boolean
    get() =
      (
        currentState == GroundIntakeState.TARGETING_POSITION &&
          armProfile.isFinished(Clock.fpgaTime - timeProfileGeneratedAt) &&
          (inputs.armPosition - armPositionTarget).absoluteValue <=
          GroundIntakeConstants.ARM_TOLERANCE
        ) ||
        (TunableGroundIntakeStates.enableArm.get() != 1.0)

  val canContinueSafely: Boolean
    get() =
      currentRequest is Request.GroundIntakeRequest.TargetingPosition &&
        (
          ((Clock.fpgaTime - timeProfileGeneratedAt) - armProfile.totalTime() < 1.0.seconds) ||
            armProfile.isFinished(Clock.fpgaTime - timeProfileGeneratedAt)
          ) &&
        (inputs.armPosition - armPositionTarget).absoluteValue <= 5.degrees

  init {

    if (RobotBase.isReal()) {
      kP.initDefault(GroundIntakeConstants.PID.NEO_KP)
      kI.initDefault(GroundIntakeConstants.PID.NEO_KI)
      kD.initDefault(GroundIntakeConstants.PID.NEO_KD)

      armFeedforward =
        ArmFeedforward(
          GroundIntakeConstants.PID.ARM_KS,
          GroundIntakeConstants.PID.ARM_KG,
          GroundIntakeConstants.PID.ARM_KV,
          GroundIntakeConstants.PID.ARM_KA
        )
    } else {
      kP.initDefault(GroundIntakeConstants.PID.SIM_KP)
      kI.initDefault(GroundIntakeConstants.PID.SIM_KI)
      kD.initDefault(GroundIntakeConstants.PID.SIM_KD)

      armFeedforward =
        ArmFeedforward(
          0.0.volts,
          GroundIntakeConstants.PID.ARM_KG,
          GroundIntakeConstants.PID.ARM_KV,
          GroundIntakeConstants.PID.ARM_KA
        )
    }
  }

}
