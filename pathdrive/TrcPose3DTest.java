/*
 * Copyright (c) 2026 Titan Robotics Club (http://www.titanrobotics.com)
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

package trclib.pathdrive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import org.junit.jupiter.api.Test;

public class TrcPose3DTest
{
    private static final double EPSILON = 1e-3;

    @Test
    public void testLinearVectorMath()
    {
        TrcPose3D pose1 = new TrcPose3D(1.0, 2.0, 3.0, 10.0, 20.0, 30.0);
        TrcPose3D pose2 = new TrcPose3D(4.0, 5.0, 6.0, 5.0, 5.0, 5.0);

        // Test Add
        TrcPose3D sum = pose1.add(pose2);
        assertEquals(5.0, sum.x, EPSILON);
        assertEquals(7.0, sum.y, EPSILON);
        assertEquals(9.0, sum.z, EPSILON);
        assertEquals(10.0, sum.pitch, EPSILON); // Orientation untouched by linear math

        // Test Subtract
        TrcPose3D diff = pose1.subtract(pose2);
        assertEquals(-3.0, diff.x, EPSILON);
        assertEquals(-3.0, diff.y, EPSILON);
        assertEquals(-3.0, diff.z, EPSILON);

        // Test Negate
        TrcPose3D negated = pose1.negate();
        assertEquals(-1.0, negated.x, EPSILON);
        assertEquals(-2.0, negated.y, EPSILON);
        assertEquals(-3.0, negated.z, EPSILON);

        // Test Scale
        TrcPose3D scaled = pose1.scale(2.5);
        assertEquals(2.5, scaled.x, EPSILON);
        assertEquals(5.0, scaled.y, EPSILON);
        assertEquals(7.5, scaled.z, EPSILON);
    }

    @Test
    public void testDistanceAndConversions()
    {
        TrcPose3D origin = new TrcPose3D(0.0, 0.0, 0.0, 0.0, 0.0, 45.0);
        TrcPose3D target = new TrcPose3D(3.0, 4.0, 12.0, 0.0, 0.0, 0.0);

        // Test distanceTo (3D Pythagorean: sqrt(3^2 + 4^2 + 12^2) = sqrt(169) = 13)
        assertEquals(13.0, origin.distanceTo(target), EPSILON);

        // Test toTrcPose2D projection (Drops Z, Pitch, Roll; retains X, Y, Yaw)
        TrcPose2D pose2d = origin.toTrcPose2D();
        assertEquals(0.0, pose2d.x, EPSILON);
        assertEquals(0.0, pose2d.y, EPSILON);
        assertEquals(45.0, pose2d.angle, EPSILON);
    }

    @Test
    public void testObjectContracts()
    {
        TrcPose3D original = new TrcPose3D(1.2, 3.4, 5.6, 10.0, 20.0, 30.0);

        // Test Clone
        TrcPose3D cloned = original.clone();
        assertEquals(original, cloned);
        assertEquals(original.hashCode(), cloned.hashCode());

        // Test SetAs
        TrcPose3D blank = new TrcPose3D();
        blank.setAs(original);
        assertEquals(original, blank);

        // Test Inequality
        TrcPose3D different = new TrcPose3D(1.2, 3.4, 99.9, 10.0, 20.0, 30.0);
        assertNotEquals(original, different);
    }

    @Test
    public void testRotateMethodCwPositiveConvention()
    {
        TrcPose3D point = new TrcPose3D(0.0, 1.0, 0.0);
        TrcPose3D rotation = new TrcPose3D(0.0, 0.0, 0.0, 0.0, 0.0, 90.0);  // 90 deg Clockwise around Z

        TrcPose3D rotatedPoint = point.rotate(rotation);

        // Actively rotating (0, 1, 0) by 90 deg CW shifts it to the left in standard CCW systems
        assertEquals(1.0, rotatedPoint.x, EPSILON);
        assertEquals(0.0, rotatedPoint.y, EPSILON);
        assertEquals(0.0, rotatedPoint.z, EPSILON);
    }

    @Test
    public void testTranslatePose()
    {
        // Robot at origin, turned 90 degrees Clockwise (facing global Right)
        TrcPose3D robotPose = new TrcPose3D(0.0, 0.0, 0.0, 0.0, 0.0, 90.0);

        // Move 2 meters forward relative to its own face
        TrcPose3D translated = robotPose.translatePose(0.0, 2.0, 0.0);

        // Since it's turned 90 deg CW, local forward is global +X
        assertEquals(2.0, translated.x, EPSILON);
        assertEquals(0.0, translated.y, EPSILON);
        assertEquals(90.0, translated.yaw, EPSILON);
    }

    @Test
    public void testAddRelativePoseOrientationCompounding()
    {
        TrcPose3D currentRobotPose = new TrcPose3D(0.0, 0.0, 0.0, 0.0, 0.0, 90.0);
        TrcPose3D cameraRelativePose = new TrcPose3D(0.0, 0.0, 0.0, 45.0, 0.0, 0.0);

        TrcPose3D combinedGlobalPose = currentRobotPose.addRelativePose(cameraRelativePose);

        assertEquals(45.0, combinedGlobalPose.pitch, EPSILON);
        assertEquals(0.0, combinedGlobalPose.roll, EPSILON);
        assertEquals(90.0, combinedGlobalPose.yaw, EPSILON);
    }

    @Test
    public void testComplexRelativeToInversion()
    {
        // A target sitting at global (10, 10, 5) with a complex orientation
        TrcPose3D targetGlobal = new TrcPose3D(10.0, 10.0, 5.0, 0.0, 0.0, 90.0);

        // A robot observing from global (10, 5, 5), turned 90 degrees Clockwise
        TrcPose3D robotGlobal = new TrcPose3D(10.0, 5.0, 5.0, 0.0, 0.0, 90.0);

        // Calculate where the target is from the perspective of the robot's coordinate frame
        TrcPose3D relativePose = targetGlobal.relativeTo(robotGlobal, true);

        // Since the robot is at Y=5 facing towards the target at Y=10,
        // the target is directly out along the robot's local Y-axis (Forward) by 5 units
        assertEquals(-5.0, relativePose.x, EPSILON);
        assertEquals(0.0, relativePose.y, EPSILON);
        assertEquals(0.0, relativePose.z, EPSILON);

        // Both are yawed 90 degrees globally, so their relative yaw difference should be 0
        assertEquals(0.0, relativePose.yaw, EPSILON);
    }
}   //class TrcPose3DTest
