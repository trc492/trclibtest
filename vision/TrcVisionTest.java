/*
 * Copyright (c) 2026 Titan Robotics Club (http://titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package trclib.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.Test;
import trclib.pathdrive.TrcPose3D;
import trclib.pathdrive.TrcPose2D;

public class TrcVisionTest
{
    private static final double EPSILON = 1e-2;
    // REAL CAMERA POSE CONSTANTS: Change these to match your actual robot mounting anytime!
    private static final double CAM_X = 3.0;        // Inches right from center
    private static final double CAM_Y = 4.5;        // Inches forward from center
    private static final double CAM_Z = 8.0;        // Inches up from ground
    private static final double CAM_YAW = -45.0;    // Yawed 45 degrees left (Counter-Clockwise)

    @Test
    public void testTransformCameraSpaceToRobotSpace_SimpleOffset()
    {
        // Setup camera with no rotation
        TrcPose3D cameraPose = new TrcPose3D(CAM_X, CAM_Y, CAM_Z, 0.0, 0.0, 0.0);

        // Target sitting exactly 24 inches straight out of the camera lens
        double targetDistance = 24.0;
        TrcPose3D targetPoseCameraSpace = new TrcPose3D(0.0, targetDistance, 0.0, 0.0, 0.0, 0.0);

        // 🧮 FORMULA FROM COMMENTS:
        // X_robot = camera_x + target_x
        // Y_robot = camera_y + target_y
        // Heading = atan2(X_robot, Y_robot)
        double expectedX = CAM_X + targetPoseCameraSpace.x;
        double expectedY = CAM_Y + targetDistance;
        double expectedHeading = Math.toDegrees(Math.atan2(expectedX, expectedY));

        TrcPose2D result = TrcVision.TargetInfo.transformCameraSpaceToRobotSpace(targetPoseCameraSpace, cameraPose);

        assertNotNull(result);
        assertEquals(expectedX, result.x, EPSILON);
        assertEquals(expectedY, result.y, EPSILON);
        assertEquals(expectedHeading, result.angle, EPSILON);
    }

    @Test
    public void testTransformCameraSpaceToRobotSpace_YawedCamera()
    {
        // Setup camera at the robot center but yawed left
        TrcPose3D cameraPose = new TrcPose3D(0.0, 0.0, 0.0, 0.0, 0.0, CAM_YAW);

        // Target sitting exactly 50 inches out along the camera lens centerline
        double targetDistance = 50.0;
        TrcPose3D targetPoseCameraSpace = new TrcPose3D(0.0, targetDistance, 0.0, 0.0, 0.0, 0.0);

        // 🧮 FORMULA FROM COMMENTS (Standard 2D Trigonometric Rotation Matrix):
        // Convert the camera yaw to radians to pass to cos/sin.
        // Because a target straight out of the lens means its position vector points
        // along the camera's local forward direction, we project using the camera's angle.
        double headingRad = Math.toRadians(CAM_YAW);

        // X = target * sin(heading), Y = target * cos(heading)
        // This shifts the unit circle to match your Y-forward, X-right orientation
        double expectedX = targetDistance * Math.sin(headingRad);
        double expectedY = targetDistance * Math.cos(headingRad);
        double expectedHeading = Math.toDegrees(Math.atan2(expectedX, expectedY));

        TrcPose2D result = TrcVision.TargetInfo.transformCameraSpaceToRobotSpace(targetPoseCameraSpace, cameraPose);

        assertNotNull(result);
        assertEquals(expectedX, result.x, EPSILON);
        assertEquals(expectedY, result.y, EPSILON);
        assertEquals(expectedHeading, result.angle, EPSILON);
    }
}   //class TrcVisionTest
