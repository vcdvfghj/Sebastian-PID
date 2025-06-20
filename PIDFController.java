import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PIDFController;

@TeleOp(name = "PID Move to Targe Pose")
public class PID extends LinearOpMode {

        GoBildaPinpointDriver pinpoint;
        MecanumDrive drivetrain;

        // Target center pose
        final double TARGET_X = -759.46;
        final double TARGET_Y = 864.87;
        final double TARGET_HEADING_DEGREES = -33.4;
        final double xykp = 0.045;
        final double xyki = 0.0;
        final double xykd = 0.0015;
        final double xykf = 0.0;
        final double hkp = 0.03;
        final double hki = 0.0;
        final double hkd = 0.002;
        final double hkf = 0.0;

        // PID tuning (start low and tune!)
        PIDFController xPID = new PIDFController(xykp, xyki, xykd, xykf);
        PIDFController yPID = new PIDFController(xykp, xyki, xykd, xykf);
        PIDFController hPID = new PIDFController(hkp, hki, hkd, hkf); // angle PID needs less power

        @Override
        public void runOpMode() {

                // Init hardware
                GoBildaPinpointDriver pinpointSensor = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
                pinpoint = new GoBildaPinpointDriver(pinpointSensor.getDeviceClient(), true);

                Pose2d startPose = new Pose2d(
                        pinpoint.getPosX()/25.4,
                        pinpoint.getPosY()/25.4,
                        Math.toRadians(pinpoint.getHeading()));
                drivetrain = new MecanumDrive(hardwareMap, startPose);



                // Set targets
                xPID.setSetPoint(TARGET_X/25.4);
                yPID.setSetPoint(TARGET_Y/25.4);
                hPID.setSetPoint(TARGET_HEADING_DEGREES);

                // Tolerances (tweak as needed)
                xPID.setTolerance(0.5, 1);
                yPID.setTolerance(0.5, 1);
                hPID.setTolerance(2.0, 5.0);             // in degrees

                waitForStart();

                while (opModeIsActive() &&
                        !(xPID.atSetPoint() && yPID.atSetPoint() && hPID.atSetPoint())) {

                        double x = pinpoint.getPosX() / 25.4;
                        double y = pinpoint.getPosY() / 25.4;
                        double headingDeg = normalizeAngle(pinpoint.getHeading());
                        double headingRad = Math.toRadians(headingDeg);

                        // PID outputs
                        double xPower = xPID.calculate(x);
                        double yPower = yPID.calculate(y);
                        double hPower = hPID.calculate(headingDeg);  // degrees used consistently

                        // Clamp outputs
                        xPower = clamp(xPower, -0.6, 0.6);
                        yPower = clamp(yPower, -0.6, 0.6);
                        hPower = clamp(hPower, -0.5, 0.5);

                        // Field-centric drive
                        drivetrain.driveFieldCentric(yPower, xPower, hPower);  // forward, strafe, turn

                        // Debugging info
                        telemetry.addData("Error (X,Y,H)", "%.2f, %.2f, %.2f",
                                xPID.getPositionError(), xPID.getVelocityError(), yPID.getPositionError());
                        telemetry.addData("Current (X,Y,H)", "%.2f, %.2f, %.2f", x, y, headingDeg);
                        telemetry.addData("PID Output", "X: %.2f, Y: %.2f, H: %.2f", xPower, yPower, hPower);
                        telemetry.addData("xPower (strafe)", xPower);
                        telemetry.addData("yPower (forward)", yPower);
                        telemetry.addData("hPower (turn)", hPower);
                        telemetry.update();
                }
                drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        }

        // Clamp utility
        private double clamp(double value, double min, double max) {
                return Math.max(min, Math.min(max, value));
        }

        // Normalize degrees to [-180, 180]
        private double normalizeAngle(double angle) {
                while (angle > 180) angle -= 360;
                while (angle < -180) angle += 360;
                return angle;
        }
}
