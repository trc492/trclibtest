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

    /**
     * Asserts that two 3D poses have the same position and orientation.
     */
    private static void assertPoseEquals(
        TrcPose3D expected, TrcPose3D actual)
    {
        assertEquals(expected.x, actual.x, EPSILON, "x");
        assertEquals(expected.y, actual.y, EPSILON, "y");
        assertEquals(expected.z, actual.z, EPSILON, "z");
        assertEquals(expected.pitch, actual.pitch, EPSILON, "pitch");
        assertEquals(expected.roll, actual.roll, EPSILON, "roll");
        assertEquals(expected.yaw, actual.yaw, EPSILON, "yaw");
    }

    /**
     * Asserts that two 2D poses have the same position and orientation.
     */
    private static void assertPoseEquals(
        TrcPose2D expected, TrcPose2D actual)
    {
        assertEquals(expected.x, actual.x, EPSILON, "x");
        assertEquals(expected.y, actual.y, EPSILON, "y");
        assertEquals(expected.angle, actual.angle, EPSILON, "angle");
    }

    @Test
    public void testToTrcPose2DBearing()
    {
        TrcPose3D pose =
            new TrcPose3D(10.0, 20.0, 30.0, 40.0, 50.0, 60.0);

        TrcPose2D result = pose.toTrcPose2DBearing();

        assertEquals(10.0, result.x, EPSILON);
        assertEquals(20.0, result.y, EPSILON);
        assertEquals(
            Math.toDegrees(Math.atan2(10.0, 20.0)),
            result.angle,
            EPSILON);
    }

    @Test
    public void testToTrcPose2DBearingCardinalDirections()
    {
        assertPoseEquals(
            new TrcPose2D(0.0, 10.0, 0.0),
            new TrcPose3D(0.0, 10.0, 5.0).toTrcPose2DBearing());

        assertPoseEquals(
            new TrcPose2D(10.0, 0.0, 90.0),
            new TrcPose3D(10.0, 0.0, 5.0).toTrcPose2DBearing());

        assertPoseEquals(
            new TrcPose2D(0.0, -10.0, 180.0),
            new TrcPose3D(0.0, -10.0, 5.0).toTrcPose2DBearing());

        assertPoseEquals(
            new TrcPose2D(-10.0, 0.0, -90.0),
            new TrcPose3D(-10.0, 0.0, 5.0).toTrcPose2DBearing());
    }

    @Test
    public void testToTrcPose2DBearingIgnoresYaw()
    {
        TrcPose3D pose1 =
            new TrcPose3D(10.0, 20.0, 30.0, 40.0, 50.0, 0.0);
        TrcPose3D pose2 =
            new TrcPose3D(10.0, 20.0, 30.0, 40.0, 50.0, 120.0);

        TrcPose2D result1 = pose1.toTrcPose2DBearing();
        TrcPose2D result2 = pose2.toTrcPose2DBearing();

        assertPoseEquals(result1, result2);
    }

    @Test
    public void testLinearVectorMath()
    {
        TrcPose3D pose1 =
            new TrcPose3D(1.0, 2.0, 3.0, 10.0, 20.0, 30.0);

        TrcPose3D pose2 =
            new TrcPose3D(4.0, 5.0, 6.0, 5.0, 5.0, 5.0);

        // Add.
        TrcPose3D sum = pose1.add(pose2);

        assertEquals(5.0, sum.x, EPSILON);
        assertEquals(7.0, sum.y, EPSILON);
        assertEquals(9.0, sum.z, EPSILON);

        // add() only operates on translation.
        assertEquals(pose1.pitch, sum.pitch, EPSILON);
        assertEquals(pose1.roll, sum.roll, EPSILON);
        assertEquals(pose1.yaw, sum.yaw, EPSILON);

        // Subtract.
        TrcPose3D diff = pose1.subtract(pose2);

        assertEquals(-3.0, diff.x, EPSILON);
        assertEquals(-3.0, diff.y, EPSILON);
        assertEquals(-3.0, diff.z, EPSILON);

        // subtract() only operates on translation.
        assertEquals(pose1.pitch, diff.pitch, EPSILON);
        assertEquals(pose1.roll, diff.roll, EPSILON);
        assertEquals(pose1.yaw, diff.yaw, EPSILON);

        // Negate.
        TrcPose3D negated = pose1.negate();

        assertEquals(-1.0, negated.x, EPSILON);
        assertEquals(-2.0, negated.y, EPSILON);
        assertEquals(-3.0, negated.z, EPSILON);

        // negate() only operates on translation.
        assertEquals(pose1.pitch, negated.pitch, EPSILON);
        assertEquals(pose1.roll, negated.roll, EPSILON);
        assertEquals(pose1.yaw, negated.yaw, EPSILON);

        // Scale.
        TrcPose3D scaled = pose1.scale(2.5);

        assertEquals(2.5, scaled.x, EPSILON);
        assertEquals(5.0, scaled.y, EPSILON);
        assertEquals(7.5, scaled.z, EPSILON);

        // scale() only operates on translation.
        assertEquals(pose1.pitch, scaled.pitch, EPSILON);
        assertEquals(pose1.roll, scaled.roll, EPSILON);
        assertEquals(pose1.yaw, scaled.yaw, EPSILON);
    }

    @Test
    public void testDistanceAndConversions()
    {
        TrcPose3D origin =
            new TrcPose3D(0.0, 0.0, 0.0, 0.0, 0.0, 45.0);

        TrcPose3D target =
            new TrcPose3D(3.0, 4.0, 12.0, 0.0, 0.0, 0.0);

        // sqrt(3^2 + 4^2 + 12^2) = 13.
        assertEquals(13.0, origin.distanceTo(target), EPSILON);

        // toTrcPose2D drops Z, pitch and roll.
        TrcPose2D pose2d = origin.toTrcPose2D();

        assertEquals(0.0, pose2d.x, EPSILON);
        assertEquals(0.0, pose2d.y, EPSILON);
        assertEquals(45.0, pose2d.angle, EPSILON);
    }

    @Test
    public void testObjectContracts()
    {
        TrcPose3D original =
            new TrcPose3D(1.2, 3.4, 5.6, 10.0, 20.0, 30.0);

        // Clone.
        TrcPose3D cloned = original.clone();

        assertEquals(original, cloned);
        assertEquals(original.hashCode(), cloned.hashCode());

        // SetAs.
        TrcPose3D blank = new TrcPose3D();
        blank.setAs(original);

        assertEquals(original, blank);

        // Inequality.
        TrcPose3D different =
            new TrcPose3D(1.2, 3.4, 99.9, 10.0, 20.0, 30.0);

        assertNotEquals(original, different);
    }

    /**
     * Verifies positive clockwise yaw rotation without changing orientation.
     */
    @Test
    public void testRotateYawCwPositive()
    {
        TrcPose3D point =
            new TrcPose3D(0.0, 1.0, 0.0, 10.0, 20.0, 30.0);

        // TRC yaw is clockwise-positive.
        // +Y rotated 90 degrees CW becomes +X.
        TrcPose3D rotatedPoint =
            point.rotate(0.0, 0.0, 90.0);

        assertPoseEquals(
            new TrcPose3D(1.0, 0.0, 0.0, 10.0, 20.0, 30.0),
            rotatedPoint);
    }

    /**
     * Verifies positive pitch rotation without changing orientation.
     */
    @Test
    public void testRotatePitch()
    {
        TrcPose3D point =
            new TrcPose3D(0.0, 1.0, 0.0, 10.0, 20.0, 30.0);

        // Positive pitch rotates +Y toward +Z.
        TrcPose3D rotatedPoint =
            point.rotate(90.0, 0.0, 0.0);

        assertPoseEquals(
            new TrcPose3D(0.0, 0.0, 1.0, 10.0, 20.0, 30.0),
            rotatedPoint);
    }

    /**
     * Verifies positive roll rotation without changing orientation.
     */
    @Test
    public void testRotateRoll()
    {
        TrcPose3D point =
            new TrcPose3D(1.0, 0.0, 0.0, 10.0, 20.0, 30.0);

        // Positive roll rotates +X toward -Z.
        TrcPose3D rotatedPoint =
            point.rotate(0.0, 90.0, 0.0);

        assertPoseEquals(
            new TrcPose3D(0.0, 0.0, -1.0, 10.0, 20.0, 30.0),
            rotatedPoint);
    }

    /**
     * Verifies rotation around the world origin at cardinal yaw angles
     * without changing the pose's orientation.
     */
    @Test
    public void testRotateCardinalYawAngles()
    {
        TrcPose3D pose =
            new TrcPose3D(3.0, 4.0, 5.0, 10.0, 20.0, 30.0);

        assertPoseEquals(
            new TrcPose3D(3.0, 4.0, 5.0, 10.0, 20.0, 30.0),
            pose.rotate(0.0, 0.0, 0.0));

        assertPoseEquals(
            new TrcPose3D(4.0, -3.0, 5.0, 10.0, 20.0, 30.0),
            pose.rotate(0.0, 0.0, 90.0));

        assertPoseEquals(
            new TrcPose3D(-3.0, -4.0, 5.0, 10.0, 20.0, 30.0),
            pose.rotate(0.0, 0.0, 180.0));

        assertPoseEquals(
            new TrcPose3D(-4.0, 3.0, 5.0, 10.0, 20.0, 30.0),
            pose.rotate(0.0, 0.0, 270.0));
    }

    /**
     * Verifies negative yaw rotation without changing orientation.
     */
    @Test
    public void testRotateNegativeYaw()
    {
        TrcPose3D pose =
            new TrcPose3D(3.0, 4.0, 5.0, 10.0, 20.0, 30.0);

        assertPoseEquals(
            new TrcPose3D(-4.0, 3.0, 5.0, 10.0, 20.0, 30.0),
            pose.rotate(0.0, 0.0, -90.0));
    }

    /**
     * Verifies that rotatePose() rotates both position and orientation.
     */
    @Test
    public void testRotatePoseYaw()
    {
        TrcPose3D pose =
            new TrcPose3D(0.0, 1.0, 0.0, 0.0, 0.0, 30.0);

        TrcPose3D rotated =
            pose.rotatePose(0.0, 0.0, 90.0);

        assertPoseEquals(
            new TrcPose3D(1.0, 0.0, 0.0, 0.0, 0.0, 120.0),
            rotated);

        // Original pose must remain unchanged.
        assertPoseEquals(
            new TrcPose3D(0.0, 1.0, 0.0, 0.0, 0.0, 30.0),
            pose);
    }

    /**
     * Verifies that rotatePose() with zero rotation returns an equivalent pose.
     */
    @Test
    public void testRotatePoseZeroRotation()
    {
        TrcPose3D pose =
            new TrcPose3D(
                10.0, 20.0, 30.0,
                20.0, 30.0, 40.0);

        assertPoseEquals(
            pose,
            pose.rotatePose(0.0, 0.0, 0.0));
    }

    @Test
    public void testTranslatePose()
    {
        // Robot at origin, facing global +X.
        TrcPose3D robotPose =
            new TrcPose3D(0.0, 0.0, 0.0, 0.0, 0.0, 90.0);

        // Move 2 units forward in the robot's local +Y direction.
        TrcPose3D translated =
            robotPose.translatePose(0.0, 2.0, 0.0);

        // Local +Y maps to global +X.
        assertEquals(2.0, translated.x, EPSILON);
        assertEquals(0.0, translated.y, EPSILON);
        assertEquals(0.0, translated.z, EPSILON);

        // Orientation is unchanged.
        assertEquals(0.0, translated.pitch, EPSILON);
        assertEquals(0.0, translated.roll, EPSILON);
        assertEquals(90.0, translated.yaw, EPSILON);
    }

    @Test
    public void testTranslatePoseWithComplexOrientation()
    {
        TrcPose3D pose =
            new TrcPose3D(
                10.0, 20.0, 30.0,
                20.0, 30.0, 40.0);

        TrcPose3D result =
            pose.translatePose(0.0, 0.0, 0.0);

        assertPoseEquals(pose, result);
    }

    @Test
    public void testAddRelativePoseTranslation()
    {
        // Current robot pose: origin, facing +X globally.
        TrcPose3D currentPose =
            new TrcPose3D(
                0.0, 0.0, 0.0,
                0.0, 0.0, 90.0);

        // Relative displacement: local +Y = robot forward.
        TrcPose3D relativePose =
            new TrcPose3D(
                0.0, 2.0, 0.0,
                0.0, 0.0, 0.0);

        TrcPose3D result =
            currentPose.addRelativePose(relativePose);

        assertEquals(2.0, result.x, EPSILON);
        assertEquals(0.0, result.y, EPSILON);
        assertEquals(0.0, result.z, EPSILON);

        assertEquals(0.0, result.pitch, EPSILON);
        assertEquals(0.0, result.roll, EPSILON);
        assertEquals(90.0, result.yaw, EPSILON);
    }

    @Test
    public void testAddRelativePoseOrientationCompounding()
    {
        TrcPose3D currentPose =
            new TrcPose3D(
                0.0, 0.0, 0.0,
                0.0, 0.0, 90.0);

        TrcPose3D relativePose =
            new TrcPose3D(
                0.0, 0.0, 0.0,
                45.0, 0.0, 0.0);

        TrcPose3D result =
            currentPose.addRelativePose(relativePose);

        assertEquals(45.0, result.pitch, EPSILON);
        assertEquals(0.0, result.roll, EPSILON);
        assertEquals(90.0, result.yaw, EPSILON);
    }

    @Test
    public void testRelativeToSimpleTranslation()
    {
        TrcPose3D target =
            new TrcPose3D(
                10.0, 10.0, 5.0,
                0.0, 0.0, 90.0);

        TrcPose3D robot =
            new TrcPose3D(
                10.0, 5.0, 5.0,
                0.0, 0.0, 90.0);

        TrcPose3D relative =
            target.relativeTo(robot, true);

        /*
         * Robot is facing global +X.
         *
         * Global +Y therefore corresponds to local -X.
         */
        assertEquals(-5.0, relative.x, EPSILON);
        assertEquals(0.0, relative.y, EPSILON);
        assertEquals(0.0, relative.z, EPSILON);

        assertEquals(0.0, relative.pitch, EPSILON);
        assertEquals(0.0, relative.roll, EPSILON);
        assertEquals(0.0, relative.yaw, EPSILON);
    }

    @Test
    public void testRelativeToAndAddRelativePoseAreInverseOperations()
    {
        TrcPose3D reference =
            new TrcPose3D(
                10.0, -20.0, 5.0,
                20.0, 30.0, 40.0);

        TrcPose3D target =
            new TrcPose3D(
                -15.0, 25.0, 30.0,
                -15.0, 10.0, 70.0);

        TrcPose3D relative =
            target.relativeTo(reference, true);

        TrcPose3D reconstructed =
            reference.addRelativePose(relative);

        assertPoseEquals(target, reconstructed);
    }

    @Test
    public void testRelativeToWithoutTransformingAngle()
    {
        TrcPose3D reference =
            new TrcPose3D(
                10.0, 20.0, 30.0,
                15.0, 25.0, 35.0);

        TrcPose3D target =
            new TrcPose3D(
                20.0, 30.0, 40.0,
                -10.0, -20.0, -30.0);

        TrcPose3D relative =
            target.relativeTo(reference, false);

        /*
         * Translation is still expressed in the reference frame.
         */
        TrcPose3D reconstructedTranslation =
            reference.addRelativePose(
                new TrcPose3D(
                    relative.x,
                    relative.y,
                    relative.z));

        assertEquals(target.x, reconstructedTranslation.x, EPSILON);
        assertEquals(target.y, reconstructedTranslation.y, EPSILON);
        assertEquals(target.z, reconstructedTranslation.z, EPSILON);

        /*
         * transformAngle=false means the target's original orientation
         * is retained.
         */
        assertEquals(target.pitch, relative.pitch, EPSILON);
        assertEquals(target.roll, relative.roll, EPSILON);
        assertEquals(target.yaw, relative.yaw, EPSILON);
    }

    @Test
    public void testTrcPose3DInverse()
    {
        TrcPose3D original =
            new TrcPose3D(
                10.0, -5.0, 20.0,
                30.0, 15.0, 90.0);

        TrcPose3D inverse =
            original.inverse();

        TrcPose3D identity1 =
            original.addRelativePose(inverse);

        assertEquals(0.0, identity1.x, EPSILON);
        assertEquals(0.0, identity1.y, EPSILON);
        assertEquals(0.0, identity1.z, EPSILON);
        assertEquals(0.0, identity1.pitch, EPSILON);
        assertEquals(0.0, identity1.roll, EPSILON);
        assertEquals(0.0, identity1.yaw, EPSILON);

        /*
         * A correct rigid-body inverse must also satisfy:
         *
         *     T^-1 * T = I
         */
        TrcPose3D identity2 =
            inverse.addRelativePose(original);

        assertEquals(0.0, identity2.x, EPSILON);
        assertEquals(0.0, identity2.y, EPSILON);
        assertEquals(0.0, identity2.z, EPSILON);
        assertEquals(0.0, identity2.pitch, EPSILON);
        assertEquals(0.0, identity2.roll, EPSILON);
        assertEquals(0.0, identity2.yaw, EPSILON);
    }

    @Test
    public void testInverseOfPureTranslation()
    {
        TrcPose3D pose =
            new TrcPose3D(
                10.0, -5.0, 20.0,
                0.0, 0.0, 0.0);

        TrcPose3D inverse = pose.inverse();

        assertEquals(-10.0, inverse.x, EPSILON);
        assertEquals(5.0, inverse.y, EPSILON);
        assertEquals(-20.0, inverse.z, EPSILON);

        assertEquals(0.0, inverse.pitch, EPSILON);
        assertEquals(0.0, inverse.roll, EPSILON);
        assertEquals(0.0, inverse.yaw, EPSILON);
    }

    @Test
    public void testInverseOfPureYaw()
    {
        TrcPose3D pose =
            new TrcPose3D(
                10.0, -5.0, 20.0,
                0.0, 0.0, 90.0);

        TrcPose3D inverse = pose.inverse();

        /*
         * A 90-degree CW yaw corresponds to a -90-degree mathematical
         * rotation. Its inverse is therefore +90 degrees mathematically,
         * or -90 degrees in the TRC CW-positive convention.
         */
        assertEquals(-5.0, inverse.x, EPSILON);
        assertEquals(-10.0, inverse.y, EPSILON);
        assertEquals(-20.0, inverse.z, EPSILON);

        assertEquals(0.0, inverse.pitch, EPSILON);
        assertEquals(0.0, inverse.roll, EPSILON);
        assertEquals(-90.0, inverse.yaw, EPSILON);
    }

    @Test
    public void testInverseRoundTrip()
    {
        TrcPose3D pose =
            new TrcPose3D(
                -12.0, 7.0, 18.0,
                25.0, -35.0, 55.0);

        TrcPose3D inverse = pose.inverse();

        TrcPose3D identity =
            pose.addRelativePose(inverse);

        assertEquals(0.0, identity.x, EPSILON);
        assertEquals(0.0, identity.y, EPSILON);
        assertEquals(0.0, identity.z, EPSILON);
        assertEquals(0.0, identity.pitch, EPSILON);
        assertEquals(0.0, identity.roll, EPSILON);
        assertEquals(0.0, identity.yaw, EPSILON);
    }
}   //class TrcPose3DTest
