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

import trclib.pathdrive.TrcPose2D;
import trclib.pathdrive.TrcPose3D;

public class TrcVisionTest
{
    private static final double EPSILON = 1e-6;

    private static final double CAM_X = 3.0;
    private static final double CAM_Y = 4.5;
    private static final double CAM_Z = 8.0;
    private static final double CAM_YAW = -45.0;

    private static void assertPoseEquals(
        TrcPose3D expected, TrcPose3D actual)
    {
        assertEquals(expected.x, actual.x, EPSILON);
        assertEquals(expected.y, actual.y, EPSILON);
        assertEquals(expected.z, actual.z, EPSILON);
        assertEquals(expected.pitch, actual.pitch, EPSILON);
        assertEquals(expected.roll, actual.roll, EPSILON);
        assertEquals(expected.yaw, actual.yaw, EPSILON);
    }

    private static void assertPoseEquals(
        TrcPose2D expected, TrcPose2D actual)
    {
        assertEquals(expected.x, actual.x, EPSILON);
        assertEquals(expected.y, actual.y, EPSILON);
        assertEquals(expected.angle, actual.angle, EPSILON);
    }

    @Test
    public void testTransformCameraSpaceToRobotSpace_SimpleOffset()
    {
        TrcPose3D cameraPose =
            new TrcPose3D(
                CAM_X, CAM_Y, CAM_Z,
                0.0, 0.0, 0.0);

        TrcPose3D targetPoseCameraSpace =
            new TrcPose3D(
                0.0, 24.0, 0.0,
                0.0, 0.0, 0.0);

        TrcPose3D result =
            TrcVision.TargetInfo.transformCameraSpaceToRobotSpace(
                targetPoseCameraSpace, cameraPose);

        assertNotNull(result);

        assertEquals(CAM_X, result.x, EPSILON);
        assertEquals(CAM_Y + 24.0, result.y, EPSILON);
        assertEquals(CAM_Z, result.z, EPSILON);

        assertEquals(0.0, result.pitch, EPSILON);
        assertEquals(0.0, result.roll, EPSILON);
        assertEquals(0.0, result.yaw, EPSILON);
    }

    @Test
    public void testTransformCameraSpaceToRobotSpace_YawedCamera()
    {
        TrcPose3D cameraPose =
            new TrcPose3D(
                0.0, 0.0, 0.0,
                0.0, 0.0, CAM_YAW);

        TrcPose3D targetPoseCameraSpace =
            new TrcPose3D(
                0.0, 50.0, 0.0,
                0.0, 0.0, 0.0);

        TrcPose3D result =
            TrcVision.TargetInfo.transformCameraSpaceToRobotSpace(
                targetPoseCameraSpace, cameraPose);

        double yawRad = Math.toRadians(CAM_YAW);
        double expectedX = 50.0 * Math.sin(yawRad);
        double expectedY = 50.0 * Math.cos(yawRad);

        assertEquals(expectedX, result.x, EPSILON);
        assertEquals(expectedY, result.y, EPSILON);
        assertEquals(0.0, result.z, EPSILON);

        assertEquals(0.0, result.pitch, EPSILON);
        assertEquals(0.0, result.roll, EPSILON);
        assertEquals(CAM_YAW, result.yaw, EPSILON);
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
                5.0, 12.0, 7.0,
                15.0, 25.0, 30.0);

        TrcPose3D result =
            TrcVision.TargetInfo.transformCameraSpaceToRobotSpace(
                targetPoseCameraSpace, cameraPose);

        TrcPose3D recovered =
            result.relativeTo(cameraPose);

        assertPoseEquals(targetPoseCameraSpace, recovered);
    }
}   //class TrcVisionTest
