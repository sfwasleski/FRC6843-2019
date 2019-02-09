/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6843.robot.subsystems;

import java.util.List;
import java.util.SortedSet;
import java.util.TreeSet;

import org.opencv.core.CvType;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class VisionSubsystem extends Subsystem {
  public static final int IMAGE_WIDTH = 320;
  public static final int IMAGE_HEIGHT = 240;
  public static final double CAMERA_FOV = 23.35;
  public static final double CAMERA_OFFSET = 0.0;

  private VisionThread visionThread;
  private double targetAngle;
  private Object targetLock = new Object();
  private int threadMiscount = 0;

  public VisionSubsystem() {
    UsbCamera visionCamera = CameraServer.getInstance().startAutomaticCapture(0);

    visionCamera.setFPS(20);
    visionCamera.setResolution(IMAGE_WIDTH, IMAGE_HEIGHT);
    visionCamera.setExposureManual(25);
    visionCamera.setBrightness(25);
    visionCamera.setWhiteBalanceManual(4500);

    visionThread = new VisionThread(visionCamera, new GripPipeline(), pipeline -> {
      try {
        double calcTurnAngle = 0.0;
        boolean foundAngle = false;
        //List<MatOfPoint> contours = pipeline.convexHullsOutput();
        List<MatOfPoint> contours = pipeline.filterContoursOutput();
        if (contours.size() >= 2) {
          SortedSet<LtoRSortableContour> sortedContours = new TreeSet<>();
          for (MatOfPoint mop : contours) {
            MatOfPoint2f contour_2f = new MatOfPoint2f();
            mop.convertTo(contour_2f, CvType.CV_32FC2);
            RotatedRect rrect = Imgproc.minAreaRect(contour_2f);
            Rect rect = rrect.boundingRect();
            sortedContours.add(new LtoRSortableContour(mop, contour_2f, rrect, rect));
          }
          // Slight chance same x value contours eliminated a target.
          if (sortedContours.size() >= 2) {
            // Taking first good pair for now. Could optimized to most centered.
            LtoRSortableContour left = null;
            LtoRSortableContour right = null;
            for (LtoRSortableContour sortedContour : sortedContours) {
              if (left == null) {
                if (((sortedContour.rrect.angle > 10.0) && (sortedContour.rrect.angle < 30.0))
                    || ((sortedContour.rrect.angle > -80.0) && (sortedContour.rrect.angle < -60.0))) {
                  left = sortedContour;
                }
              } else {
                if (((sortedContour.rrect.angle > -30.0) && (sortedContour.rrect.angle < -10.0))
                    || ((sortedContour.rrect.angle > 60.0) && (sortedContour.rrect.angle < 80.0))) {
                  right = sortedContour;
                }
                // Either we have both or we have bad data.
                break;
              }
            }

            if (right != null) {
              foundAngle = true;
              final Rect r1 = left.rect;
              final Rect r2 = right.rect;
              final double centerX = (((r1.x + (r1.width / 2)) + ((r2.x + r2.width) - (r2.width / 2))) / 2);
              final double turnRatio = (centerX / (IMAGE_WIDTH / 2)) - 1;
              calcTurnAngle = (turnRatio * CAMERA_FOV) + CAMERA_OFFSET;
            }
          }
        }

        if (foundAngle) {
          this.threadMiscount = 0;
          setTargetAngle(calcTurnAngle);
        } else {
          this.threadMiscount++;
        }

        if (this.threadMiscount > 5) {
          setTargetAngle(0.0);
        }
        SmartDashboard.putNumber("Target Angle", getTargetAngle());
        SmartDashboard.putNumber("Target Miss Count", this.threadMiscount);
        VisionThread.sleep(100);
      } catch (Throwable e) {
        System.out.println("Vision thread exception");
        e.printStackTrace();
      }
    });
    visionThread.start();
  }

  public double getTargetAngle() {
    synchronized (targetLock) {
      return this.targetAngle;
    }
  }

  private void setTargetAngle(double targetAngle) {
    synchronized (targetLock) {
      this.targetAngle = targetAngle;
    }
  }

  @Override
  public void initDefaultCommand() {
    // No default command.
  }

  private static class LtoRSortableContour implements Comparable<LtoRSortableContour> {
    public final MatOfPoint mop;
    public final MatOfPoint2f contour_2f;
    public final RotatedRect rrect;
    public final Rect rect;

    public LtoRSortableContour(final MatOfPoint mop, final MatOfPoint2f contour_2f, final RotatedRect rrect,
        final Rect rect) {
      this.mop = mop;
      this.contour_2f = contour_2f;
      this.rrect = rrect;
      this.rect = rect;
    }

    public int compareTo(LtoRSortableContour other) {
      return this.rect.x - other.rect.x;
    }
  }
}
