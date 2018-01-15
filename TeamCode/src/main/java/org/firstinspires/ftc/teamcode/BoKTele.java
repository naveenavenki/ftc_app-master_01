package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKTele
{
    // CONSTANTS
    private static final double GAME_STICK_DEAD_ZONE = 0.05;
    private static final double GAME_TRIGGER_DEAD_ZONE = 0.2;
    private static final double TURNTABLE_STICK_DEAD_ZONE = 0.8;
    private static final double UPPER_ARM_STICK_DEAD_ZONE = 0.2;
    private static final double TURNTABLE_MOTOR_POWER = 0.2;
    private static final int TURNTABLE_COUNTS_PER_MOTOR_REV = 1120; // AndyMark 40
    private static final double UPPER_ARM_MOTOR_POWER_SLOW = 0.4;//0.2
    private static final double UPPER_ARM_MOTOR_POWER_FAST = 0.6;//0.4
    private static final double SPEED_COEFF_SLOW = 0.25;
    private static final double SPEED_COEFF_FAST = 0.5;
    private static final int RA_JOYSTICK_RATIO = 500;
    private static final int RELIC_DELAY_START = 40;
    private static final int RELIC_DEPLOY_STOP = 80; // loop runs 25 times a second
    private static final double RELIC_DEPLOY_POWER = 0.6;
    private static final float GLYPH_FLICKER_INCREMENT = 0.01F;

    private static int glyphArmInitialPos = 0;
    private static double glyphWristInitialPos = 0.0;


    private BoKHardwareBot robot;
    private LinearOpMode opMode;
    private double speedCoef = SPEED_COEFF_FAST;
    private boolean trigger_left_decrease = false;

    public enum BoKTeleStatus
    {
        BOK_TELE_FAILURE,
        BOK_TELE_SUCCESS
    }


    public BoKTeleStatus initSoftware(LinearOpMode opMode,
                                      BoKHardwareBot robot,
                                      boolean trigger_left_decrease)
    {
        this.trigger_left_decrease = trigger_left_decrease;
        this.opMode = opMode;
        this.robot = robot;
        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }

    public BoKTeleStatus runSoftware()
    {
        boolean tank = false;


        // run until the end of the match (driver presses STOP)
        while (opMode.opModeIsActive()) {
            // GAMEPAD 1 CONTROLS:
            // Left & Right stick: Drive
            // B:                  Go in tank mode
            // X:                  Go in mecanum mode
            // A:                  Go in slow mode
            // Y:                  Go in fast mode
            // Left bumper:        Raise glyph flicker
            // Right bumper:       Lower glyph flicker
            if (opMode.gamepad1.b) {
                tank = true;
            }
            if (opMode.gamepad1.x) {
                tank = false;
            }
            if (!tank) {
                moveRobot();
            } else {
                moveRobotTank();
            }

            if (opMode.gamepad1.y) {
                speedCoef = SPEED_COEFF_SLOW;
            }
            if (opMode.gamepad1.a) {
                speedCoef = SPEED_COEFF_FAST;
            }

            // GAMEPAD 2 CONTROLS
            // Left stick:             Upper Arm
            // Right stick:            Turntable
            // Left and right trigger: Claw wrist down and up
            // B:                      Claw open
            // A:                      Claw closed
            // Y:                      End Game
            // X:                      Not End Game
            // Dpad Left:              Enter placement mode & raise the arm to 138 degrees
            // Dpad Right:             Exit placement mode
            // When in Placement Mode, use
            // Left bumper:            adjust upper arm position to 160 degrees
            // Right bumper:           adjust upper arm position to 180 degrees
            // X:                      adjust upper arm position to 138 degrees
            // When in End Game, use
            // Dpad Down:              Start Relic Mode
            // DPad Up:                End Relic Mode
            // Left Stick:             Relic Arm
            // Right Stick:            Relic Lift
            // B:                      Relic claw open
            // A:                      Relic claw closed
            // Left bumper:            Raise relic arm to clear the wall
            // Right bumper:           Lower relic arm for relic placement







            robot.waitForTick(BoKHardwareBot.WAIT_PERIOD);
            //Log.v("BOK", "Distance of Glyph: " +
            //        robot.rangeSensorGA.getDistance(DistanceUnit.CM));
            //Log.v("BOK", "Distance of Glyph (opt): " +
            //        robot.rangeSensorGA.rawOptical());
        }

        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }

    private void moveRobot()
    {
        /*
         * Gamepad1: Driver 1 controls the robot using the left joystick for throttle and
         * the right joystick for steering
         */
        // NOTE: the left joystick goes negative when pushed upwards
        double gamePad1LeftStickY = opMode.gamepad1.left_stick_y;
        double gamePad1LeftStickX = opMode.gamepad1.left_stick_x;
        double gamePad1RightStickX = opMode.gamepad1.right_stick_x;

        double motorPowerLF = 0;
        double motorPowerLB = 0;
        double motorPowerRF = 0;
        double motorPowerRB = 0;

        //Log.v("BOK","moveRobot: " + String.format("%.2f", gamePad1LeftStickY) + ", " +
        //        String.format("%.2f", gamePad1LeftStickX) + ", " +
        //        String.format("%.2f", gamePad1RightStickX));

        // Run mecanum wheels

        if ((Math.abs(gamePad1LeftStickY) > GAME_STICK_DEAD_ZONE) ||
                (Math.abs(gamePad1LeftStickY) < -GAME_STICK_DEAD_ZONE) ||
                (Math.abs(gamePad1LeftStickX) > GAME_STICK_DEAD_ZONE) ||
                (Math.abs(gamePad1LeftStickX) < -GAME_STICK_DEAD_ZONE) ) {
            motorPowerLF = -gamePad1LeftStickY - (-gamePad1LeftStickX);
            motorPowerLB = -gamePad1LeftStickY - gamePad1LeftStickX;
            motorPowerRF = gamePad1LeftStickY - (-gamePad1LeftStickX);
            motorPowerRB = gamePad1LeftStickY - gamePad1LeftStickX;

            //Log.v("BOK","LF:" + String.format("%.2f", motorPowerLF*speedCoef) +
            //        "LB: " + String.format("%.2f", motorPowerLB*speedCoef) +
            //        "RF: " + String.format("%.2f", motorPowerRF*speedCoef) +
            //        "RB: " + String.format("%.2f", motorPowerRB*speedCoef));
        }
        else if ((gamePad1RightStickX > GAME_STICK_DEAD_ZONE) ||
                (gamePad1RightStickX < -GAME_STICK_DEAD_ZONE)) {
            // Right joystick is for turning
            motorPowerLF = motorPowerLB = gamePad1RightStickX;
            motorPowerRF = motorPowerRB = gamePad1RightStickX;

            //Log.v("BOK","Turn: LF:" + String.format("%.2f", motorPowerLF) +
            //        "LB: " + String.format("%.2f", motorPowerLB) +
            //        "RF: " + String.format("%.2f", motorPowerRF) +
            //        "RB: " + String.format("%.2f", motorPowerRB));
        }
        robot.setPowerToDTMotors(
                (motorPowerLF * speedCoef),
                (motorPowerLB * speedCoef),
                (motorPowerRF * speedCoef),
                (motorPowerRB * speedCoef));
    }

    private void moveRobotTank()
    {
        double gamePad1LeftStickY = opMode.gamepad1.left_stick_y;
        double gamePad1RightStickY = opMode.gamepad1.right_stick_y;

        double motorPowerLF = 0;
        double motorPowerLB = 0;
        double motorPowerRF = 0;
        double motorPowerRB = 0;

        if((Math.abs(gamePad1LeftStickY) > GAME_STICK_DEAD_ZONE) ||
        (Math.abs(gamePad1LeftStickY) < -GAME_STICK_DEAD_ZONE))
        {
            motorPowerLB = -gamePad1LeftStickY;
            motorPowerLF = -gamePad1LeftStickY;
        }

        if((Math.abs(gamePad1RightStickY) > GAME_STICK_DEAD_ZONE) ||
                (Math.abs(gamePad1LeftStickY) < -GAME_STICK_DEAD_ZONE))
        {
            motorPowerRB = gamePad1RightStickY;
            motorPowerRF = gamePad1RightStickY;
        }


        robot.setPowerToDTMotors(
                (motorPowerLF * speedCoef),
                (motorPowerLB * speedCoef),
                (motorPowerRF * speedCoef),
                (motorPowerRB * speedCoef));
    }

}
