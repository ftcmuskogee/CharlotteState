package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

// remove this line to have this show up on your robot
@Autonomous(name = "BLUE Back", group = "Autonomous Main")
public class ColourMassDetectionOpModeBlue extends LinearOpMode {
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
       Pose2d startPose = new Pose2d(35, 60, Math.toRadians(270));
       drive.setPoseEstimate(startPose);
/**MID**/
       //middle forward
       TrajectorySequence Vietnam = drive.trajectorySequenceBuilder(startPose)
               .lineToConstantHeading(new Vector2d(35, 36), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                       SampleMecanumDrive.getAccelerationConstraint(70))
               .build();
       //back,strafe
       TrajectorySequence Shrike = drive.trajectorySequenceBuilder(Vietnam.end())
               .waitSeconds(1)
               .lineToConstantHeading(new Vector2d(35, 37), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                       SampleMecanumDrive.getAccelerationConstraint(70))
               .waitSeconds(1)
               .lineToConstantHeading(new Vector2d(65, 37), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                       SampleMecanumDrive.getAccelerationConstraint(70))
               .build();
       /**RIGHT**/
       //right forward, strafe
       TrajectorySequence Canada = drive.trajectorySequenceBuilder(startPose)
               .lineToConstantHeading(new Vector2d(35, 36), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                       SampleMecanumDrive.getAccelerationConstraint(70))
               .waitSeconds(1)
               .lineToConstantHeading(new Vector2d(29, 36), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                       SampleMecanumDrive.getAccelerationConstraint(70))
               .waitSeconds(1)
               .turn(Math.toRadians(45))
               .build();
       //little back, strafe
       TrajectorySequence Goose = drive.trajectorySequenceBuilder(Canada.end())
               .turn(Math.toRadians(-45))
               .waitSeconds(1)
               .lineToConstantHeading(new Vector2d(65, 37), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                       SampleMecanumDrive.getAccelerationConstraint(70))
               .build();
       /**left**/
       //left forward, strafe
       TrajectorySequence America = drive.trajectorySequenceBuilder(startPose)
               .lineToConstantHeading(new Vector2d(35, 37), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                       SampleMecanumDrive.getAccelerationConstraint(70))
               .waitSeconds(1)
               .lineToConstantHeading(new Vector2d(48, 37), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                       SampleMecanumDrive.getAccelerationConstraint(70))
               .build();
       //little back, strafe
       TrajectorySequence Eagle = drive.trajectorySequenceBuilder(America.end())
               .waitSeconds(1)
               .lineToConstantHeading(new Vector2d(50, 40), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                       SampleMecanumDrive.getAccelerationConstraint(70))
               .waitSeconds(1)
               .lineToConstantHeading(new Vector2d(67, 40), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                       SampleMecanumDrive.getAccelerationConstraint(70))
               .build();
       // now we can use recordedPropPosition in our auto code to modify where we place the purple and yellow pixels
       switch (recordedPropPosition) {
           case LEFT:
               telemetry.addLine("left");
               robot.W(0.25);
               //close
               //robot.CL(0);
               //robot.CR(.5);
               sleep(500);
               //wrist down
               robot.W(0.22);

               sleep(1000);
               robot.UP(.03);
               sleep(500);
               //forward
               //strafe left
               drive.followTrajectorySequence(America);
               //open left claw
               robot.CR(0);
               //back alittle, strafe
               drive.followTrajectorySequence(Eagle);
               sleep(500);
               //drop yelow
               robot.CL(.5);
               break;

           case MIDDLE:
               telemetry.addLine("mid");
               robot.W(0.25);
               //close
               //robot.CL(0);
               //robot.CR(.5);
               sleep(500);
               //wrist down
               robot.W(0.22);

               sleep(1000);
               robot.UP(.03);
               sleep(500);
               //forward
               drive.followTrajectorySequence(Vietnam);
               //open right claw
               robot.CR(0);
               sleep(500);
               robot.UP(.03);
               sleep(500);
               //back up
               //strafe left
               //go forward
               drive.followTrajectorySequence(Shrike);
               //drop yellow
               robot.CL(.5);
               break;

           case RIGHT:
               telemetry.addLine("right");
               robot.W(0.25);
               //close
               //robot.CL(0);
               //robot.CR(.5);
               sleep(500);
               //wrist down
               robot.W(0.22);

               sleep(1000);
               robot.UP(.03);
               sleep(500);
               //forward
               //turn right
               drive.followTrajectorySequence(Canada);
               //open left claw
               robot.CR(0);
               //back alittle, strafe
               drive.followTrajectorySequence(Goose);
               //drop yelow
               robot.CL(.5);
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