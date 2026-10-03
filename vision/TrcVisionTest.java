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

import org.junit.jupiter.api.Test;
import trclib.pathdrive.TrcPose2D;
import trclib.pathdrive.TrcPose3D;

public class TrcVisionTest
{
    private static final double EPSILON = 1e-2;

    // Camera pose relative to the robot center.
    private static final double CAM_X = 3.0;        // Inches right from center
    private static final double CAM_Y = 4.5;        // Inches forward from center
    private static final double CAM_Z = 8.0;        // Inches up from ground
    private static final double CAM_YAW = -45.0;    // Yawed 45 degrees left

    /**
     * Asserts that two 3D poses have the same position and orientation.
     */
    private static void assertPoseEquals(TrcPose3D expected, TrcPose3D actual)
    {
        assertEquals(expected.x, actual.x, EPSILON, "x");
        assertEquals(expected.y, actual.y, EPSILON, "y");
        assertEquals(expected.z, actual.z, EPSILON, "z");
        assertEquals(expected.pitch, actual.pitch, EPSILON, "pitch");
        assertEquals(expected.roll, actual.roll, EPSILON, "roll");
        assertEquals(expected.yaw, actual.yaw, EPSILON, "yaw");
    }

    /**
     * Asserts that two 2D poses have the same position and heading.
     */
    private static void assertPoseEquals(TrcPose2D expected, TrcPose2D actual)
    {
        assertEquals(expected.x, actual.x, EPSILON, "x");
        assertEquals(expected.y, actual.y, EPSILON, "y");
        assertEquals(expected.angle, actual.angle, EPSILON, "angle");
    }

    @Test
    public void testTransformCameraSpaceToRobotSpace_SimpleOffset()
    {
        // Camera mounted at a simple offset with no rotation.
        TrcPose3D cameraPose =
            new TrcPose3D(CAM_X, CAM_Y, CAM_Z, 0.0, 0.0, 0.0);

        // Target is 24 inches straight ahead of the camera.
        TrcPose3D targetPoseCameraSpace =
            new TrcPose3D(0.0, 24.0, 0.0, 0.0, 0.0, 0.0);

        TrcPose3D result =
            TrcVision.TargetInfo.transformCameraSpaceToRobotSpace(
                targetPoseCameraSpace, cameraPose);

        assertPoseEquals(
            new TrcPose3D(
                CAM_X,
                CAM_Y + 24.0,
                CAM_Z,
                0.0,
                0.0,
                0.0),
            result);
    }

    @Test
    public void testTransformCameraSpaceToRobotSpace_YawedCamera()
    {
        // Camera is at the robot center but yawed 45 degrees to the left.
        TrcPose3D cameraPose =
            new TrcPose3D(0.0, 0.0, 0.0, 0.0, 0.0, CAM_YAW);

        // Target is 50 inches straight ahead of the camera.
        TrcPose3D targetPoseCameraSpace =
            new TrcPose3D(0.0, 50.0, 0.0, 0.0, 0.0, 0.0);

        double angleRad = Math.toRadians(CAM_YAW);
        double expectedX = 50.0 * Math.sin(angleRad);
        double expectedY = 50.0 * Math.cos(angleRad);

        TrcPose3D result =
            TrcVision.TargetInfo.transformCameraSpaceToRobotSpace(
                targetPoseCameraSpace, cameraPose);

        assertPoseEquals(
            new TrcPose3D(
                expectedX,
                expectedY,
                0.0,
                0.0,
                0.0,
                CAM_YAW),
            result);
    }

    @Test
    public void testTransformCameraSpaceToRobotSpace_ComplexPose()
    {
        TrcPose3D cameraPose =
            new TrcPose3D(
                CAM_X, CAM_Y, CAM_Z,
                10.0, 20.0, CAM_YAW);

        TrcPose3D targetPoseCameraSpace =
            new TrcPose3D(
                5.0, 30.0, 2.0,
                5.0, 10.0, 15.0);

        TrcPose3D result =
            TrcVision.TargetInfo.transformCameraSpaceToRobotSpace(
                targetPoseCameraSpace, cameraPose);

        /*
         * Verify the transformation by reversing it. This avoids hard-coding
         * Euler-angle composition results for a general 3D rotation.
         */
        TrcPose3D recovered = result.relativeTo(cameraPose);

        assertPoseEquals(targetPoseCameraSpace, recovered);
    }

    @Test
    public void testProject3dTo2dSpace()
    {
        TrcPose3D targetPose3d =
            new TrcPose3D(
                12.0, 30.0, 8.0,
                20.0, 10.0, 45.0);

        TrcPose2D result =
            TrcVision.TargetInfo.project3dTo2dSpace(targetPose3d);

        double expectedHeading =
            Math.toDegrees(Math.atan2(12.0, 30.0));

        assertPoseEquals(
            new TrcPose2D(12.0, 30.0, expectedHeading),
            result);
    }

    @Test
    public void testProject3dTo2dSpace_ZeroHeading()
    {
        TrcPose3D targetPose3d =
            new TrcPose3D(
                0.0, 24.0, 8.0,
                15.0, 25.0, 90.0);

        TrcPose2D result =
            TrcVision.TargetInfo.project3dTo2dSpace(targetPose3d);

        assertPoseEquals(
            new TrcPose2D(0.0, 24.0, 0.0),
            result);
    }

    @Test
    public void testProject3dTo2dSpace_NegativeHeading()
    {
        TrcPose3D targetPose3d =
            new TrcPose3D(
                -10.0, 10.0, 5.0,
                0.0, 0.0, 0.0);

        TrcPose2D result =
            TrcVision.TargetInfo.project3dTo2dSpace(targetPose3d);

        assertPoseEquals(
            new TrcPose2D(-10.0, 10.0, -45.0),
            result);
    }

    @Test
    public void testTransformAndProject3dTo2dSpace()
    {
        TrcPose3D cameraPose =
            new TrcPose3D(CAM_X, CAM_Y, CAM_Z, 0.0, 0.0, CAM_YAW);

        TrcPose3D targetPoseCameraSpace =
            new TrcPose3D(0.0, 50.0, 0.0, 0.0, 0.0, 0.0);

        TrcPose3D targetPoseRobotSpace =
            TrcVision.TargetInfo.transformCameraSpaceToRobotSpace(
                targetPoseCameraSpace, cameraPose);

        TrcPose2D targetPose2d =
            TrcVision.TargetInfo.project3dTo2dSpace(targetPoseRobotSpace);

        double yawRad = Math.toRadians(CAM_YAW);
        double expectedX = CAM_X + 50.0 * Math.sin(yawRad);
        double expectedY = CAM_Y + 50.0 * Math.cos(yawRad);
        double expectedHeading =
            Math.toDegrees(Math.atan2(expectedX, expectedY));

        assertPoseEquals(
            new TrcPose2D(expectedX, expectedY, expectedHeading),
            targetPose2d);
    }
}   //class TrcVisionTest
