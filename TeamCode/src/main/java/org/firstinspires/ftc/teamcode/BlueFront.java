package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

// remove this line to have this show up on your robot
@Autonomous(name = "BLUE Front", group = "Autonomous Main")
public class BlueFront extends LinearOpMode {
    private VisionPortal visionPortal;
    private ColourMassDetectionProcessor colourMassDetectionProcessor;

    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */

    @Override
    public void runOpMode() {
        // the current range set by lower and upper is the full range
        // HSV takes the form: (HUE, SATURATION, VALUE)
        // which means to select our colour, only need to change HUE
        // the domains are: ([0, 180], [0, 255], [0, 255])
        // this is tuned to detect red, so you will need to experiment to fine tune it for your robot
        // and experiment to fine tune it for blue
        Scalar lower = new Scalar(110, 50, 50); // the lower hsv threshold for your detection
        Scalar upper = new Scalar(130, 255, 255); // the upper hsv threshold for your detection
        double minArea = 90; // the minimum area for the detection to consider for your prop
/*** up min area value incase the distance is limited - cause it sees it up close but not far ***/
        colourMassDetectionProcessor = new ColourMassDetectionProcessor(
                lower,
                upper,
                () -> minArea, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
                () -> 213, // the left dividing line, in this case the left third of the frame
                () -> 426 // the left dividing line, in this case the right third of the frame
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .addProcessor(colourMassDetectionProcessor)
                .build();

        // you may also want to take a look at some of the examples for instructions on
        // how to have a switchable camera (switch back and forth between two cameras)
        // or how to manually edit the exposure and gain, to account for different lighting conditions
        // these may be extra features for you to work on to ensure that your robot performs
        // consistently, even in different environments

        /**
         * User-defined init_loop method
         * <p>
         * This method will be called repeatedly during the period between when
         * the init button is pressed and when the play button is pressed (or the
         * OpMode is stopped).
         * <p>
         * This method is optional. By default, this method takes no action.
         */
        IdontWannaDie robot = new IdontWannaDie();
        robot.init(hardwareMap);
        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Currently Recorded Position", colourMassDetectionProcessor.getRecordedPropPosition());
            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.addData("Currently Detected Mass Center", "x: " + colourMassDetectionProcessor.getLargestContourX() + ", y: " + colourMassDetectionProcessor.getLargestContourY());
            telemetry.addData("Currently Detected Mass Area", colourMassDetectionProcessor.getLargestContourArea());
            telemetry.update();
        }


        /**
         * User-defined start method
         * <p>
         * This method will be called once, when the play button is pressed.
         * <p>
         * This method is optional. By default, this method takes no action.
         * <p>
         * Example usage: Starting another thread.
         */
        // shuts down the camera once the match starts, we dont need to look any more
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        // gets the recorded prop position
        ColourMassDetectionProcessor.PropPositions recordedPropPosition = colourMassDetectionProcessor.getRecordedPropPosition();

        // now we can use recordedPropPosition to determine where the prop is! if we never saw a prop, your recorded position will be UNFOUND.
        // if it is UNFOUND, you can manually set it to any of the other positions to guess
        if (recordedPropPosition == ColourMassDetectionProcessor.PropPositions.UNFOUND) {
            recordedPropPosition = ColourMassDetectionProcessor.PropPositions.MIDDLE;
        }

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-35, 60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
/**MID**/
        //middle forward
        Trajectory Vietnam = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-35, 35), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .build();

        TrajectorySequence Shrike = drive.trajectorySequenceBuilder(Vietnam.end())
                .lineToConstantHeading(new Vector2d(-35, 37), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(-49, 37), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-49, 10, Math.toRadians(350)))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(55, 15), SampleMecanumDrive.getVelocityConstraint(25, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .build();
        /**left**/
        //right forward, strafe
        TrajectorySequence Canada = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-35, 38), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .waitSeconds(1)
                /*.lineToConstantHeading(new Vector2d(-25, 38), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(70))*/
                .lineToLinearHeading(new Pose2d(-33, 38, Math.toRadians(292)))
                .build();
        TrajectorySequence Goose = drive.trajectorySequenceBuilder(Canada.end())
                .lineToConstantHeading(new Vector2d(-43, 38), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-43, 10, Math.toRadians(350)))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(55, 15), SampleMecanumDrive.getVelocityConstraint(25, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .build();

        /**right**/
        //left forward, strafe
        TrajectorySequence America = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-35, 38), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(-47, 38), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .build();
        TrajectorySequence Eagle = drive.trajectorySequenceBuilder(America.end())
                .lineToConstantHeading(new Vector2d(-47, 45), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(-35, 45), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(-35, 9), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-35, 6, Math.toRadians(350)))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(55, 10), SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .build();

        // now we can use recordedPropPosition in our auto code to modify where we place the purple and yellow pixels
        switch (recordedPropPosition) {
            case LEFT:
                robot.W(1);
                sleep(1000);
                //forward
                //turn right
                drive.followTrajectorySequence(Canada);
                robot.W(1);
                //open left claw
                robot.CL(.5);
                sleep(1000);
                drive.followTrajectorySequence(Goose);
                sleep(500);
                robot.CR(0);

                break;

            case MIDDLE:
                robot.W(1);
                sleep(1000);
                //forward
                //turn right
                drive.followTrajectory(Vietnam);
                robot.W(1);
                //open left claw
                robot.CL(.5);
                sleep(1000);
                drive.followTrajectorySequence(Shrike);
                sleep(500);
                robot.CR(0);
                break;

            case RIGHT:
                robot.W(1);
                sleep(1000);
                //forward
                //turn right
                drive.followTrajectorySequence(America);
                robot.W(1);
                //open left claw
                robot.CL(.5);
                sleep(1000);
                drive.followTrajectorySequence(Eagle);
                sleep(500);
                robot.CR(0);
                break;

        }
        while (opModeIsActive()) {
            // Don't burn CPU cycles busy-looping in this sample
            sleep(5);
        }
        while (!isStopRequested())
        {
            colourMassDetectionProcessor.close();
            visionPortal.close();
        }
    }
}