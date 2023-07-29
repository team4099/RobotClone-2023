package com.team4099.robot2023.config.constants
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.gearRatio
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perMinute

object ManipulatorConstants {
  //TODO get actual gear ration
  val ROLLER_GEAR_RATIO = 1.0.gearRatio
  val ROLLER_VOLTAGE_COMPENSATION = 12.volts
  val ROLLER_CURRENT_LIMIT = 40.amps

  val ROLLER_RAMP_RATE = 0.2

  //TODO check
  val ROLLER_MOTOR_INVERTED = false

  val MANIPULATOR_WAIT_BEFORE_DETECT_CURRENT_SPIKE = 0.3.seconds
  val MANIPULATOR_WAIT_BEFORE_DETECT_VELOCITY_DROP = 0.35.seconds

  val ROLLER_VOLTAGE_TOLERANCE = 0.4.volts

  val CONE_CURRENT_THRESHOLD = 25.amps
  val CONE_ROTATION_THRESHOLD = 40.rotations.perMinute
  val CUBE_CURRENT_THRESHOLD = 30.amps
  val CUBE_ROTATION_THRESHOLD = 40.rotations.perMinute

  val IDLE_VOLTAGE = 0.0.volts
  val CONE_IDLE = 6.volts
  val CUBE_IDLE = 3.volts
  val CONE_IN = 12.volts
  val CUBE_IN = 6.volts
  val CONE_OUT = -12.volts
  val CUBE_OUT = -12.volts
}
