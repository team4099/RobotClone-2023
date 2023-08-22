package com.team4099.robot2023.config.constants
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.gearRatio
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.kilo
import org.team4099.lib.units.perSecond

object GroundIntakeConstants {
  // TODO Get roller values from CAD
  val ROLLER_GEAR_RATIO = 1.0.gearRatio
  val ROLLER_CURRENT_LIMIT = 0.amps
  val ROLLER_MOTOR_INVERTED = true
  val ROLLER_VOLTAGE_COMPENSATION = 12.volts
  val ROLLER_MOMENT_INERTIA = 0.0000478.kilo.grams * 1.0.meters.squared
  val ARM_MOMENT_INERTIA = 0.0000478.kilo.grams * 1.0.meters.squared
  val ARM_LENGTH = 12.695.inches

  // TODO Get arm values from CAD
  val ARM_GEAR_RATIO = 1.0.gearRatio
  val ARM_CURRENT_LIMIT = 0.amps
  val ARM_MOTOR_INVERTED = true
  val ARM_VOLTAGE_COMPENSATION = 12.volts

  val ARM_ENCODER_GEAR_RATIO = 1.0.gearRatio
  // TODO Get actual absolute encoder offset
  val ABSOLUTE_ENCODER_OFFSET = 0.radians

  // TODO Figure out values for ground intake states
  val ENABLE_ARM = 0.0
  val ENABLE_ROTATION = 0.0
  val INTAKE_ANGLE = 0.degrees
  val OUTTAKE_ANGLE = 0.degrees
  val STOWED_UP_ANGLE = 0.degrees
  val STOWED_DOWN_ANGLE = 0.degrees
  val INTAKE_VOLTAGE = 0.volts
  val OUTTAKE_VOLTAGE = 0.volts
  val NEUTRAL_VOLTAGE = 0.volts
  val HELP_SCORE_VOLTAGE = 0.volts

  // TODO Figure out values for ground intake motion
  val ARM_MIN_ROTATION = 0.degrees
  val ARM_MAX_ROTATION = 56.6.degrees
  val ARM_OPEN_LOOP_MIN_ROTATION = 0.degrees
  val ARM_OPEN_LOOP_MAX_ROTATION = 0.degrees
  val MAX_ARM_VELOCITY = 0.degrees.perSecond
  val MAX_ARM_ACCELERATION = 0.degrees.perSecond.perSecond
  val ARM_TOLERANCE = 0.radians

  // TODO Change PID values
  object PID {
    val NEO_KP = 1.5.volts / 1.degrees // 0.5
    val NEO_KI = 0.0.volts / (1.degrees * 1.seconds)
    val NEO_KD = 0.0.volts / (1.degrees.perSecond)

    val SIM_KP = 1.5.volts / 1.degrees
    val SIM_KI = 0.0.volts / (1.degrees * 1.seconds)
    val SIM_KD = 0.01.volts / (1.degrees.perSecond)

    val ARM_KS = 0.2.volts

    val ARM_KG = 0.86.volts
    val ARM_KV = 1.8.volts / 1.0.radians.perSecond
    val ARM_KA = 0.1.volts / 1.0.radians.perSecond.perSecond
  }
}
