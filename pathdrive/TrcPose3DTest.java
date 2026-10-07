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

    /**
     * Verifies that combined rotations use the TRC extrinsic XYZ convention.
     *
     * <p>Extrinsic XYZ means rotations are applied about the fixed world axes
     * in X, then Y, then Z order.</p>
     */
    @Test
    public void testRotateExtrinsicXyz()
    {
        TrcPose3D point =
            new TrcPose3D(0.0, 1.0, 0.0);

        /*
         * Start at +Y.
         *
         * Extrinsic X +90:
         *     +Y -> +Z
         *
         * Extrinsic Z +90 CW does not affect +Z.
         *
         * Therefore the final position must still be +Z.
         *
         * This test deliberately combines rotations because single-axis
         * rotations cannot distinguish the rotation-order convention.
         */
        TrcPose3D rotated =
            point.rotate(90.0, 0.0, 90.0);

        assertPoseEquals(
            new TrcPose3D(0.0, 0.0, 1.0),
            rotated);
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

    /**
     * Creates an independent rotation-matrix oracle for the documented TRC
     * convention: extrinsic XYZ, with clockwise-positive yaw.
     *
     * <p>This deliberately does not use TrcPose3D or Apache Rotation. For
     * column vectors, fixed-axis/extrinsic XYZ is Rz * Ry * Rx. Since TRC yaw
     * is clockwise-positive, the mathematical Z angle is -yaw.</p>
     */
    private static double[][] oracleRotationMatrix(
        double pitch, double roll, double yaw)
    {
        double x = Math.toRadians(pitch);
        double y = Math.toRadians(roll);
        double z = Math.toRadians(-yaw);
        double cx = Math.cos(x);
        double sx = Math.sin(x);
        double cy = Math.cos(y);
        double sy = Math.sin(y);
        double cz = Math.cos(z);
        double sz = Math.sin(z);

        return new double[][]
        {
            {
                cz*cy,
                cz*sy*sx - sz*cx,
                cz*sy*cx + sz*sx
            },
            {
                sz*cy,
                sz*sy*sx + cz*cx,
                sz*sy*cx - cz*sx
            },
            {
                -sy,
                cy*sx,
                cy*cx
            }
        };
    }

    /**
     * Multiplies two 3x3 matrices.
     */
    private static double[][] multiply(double[][] a, double[][] b)
    {
        double[][] result = new double[3][3];

        for (int row = 0; row < 3; row++)
        {
            for (int col = 0; col < 3; col++)
            {
                for (int i = 0; i < 3; i++)
                {
                    result[row][col] += a[row][i]*b[i][col];
                }
            }
        }

        return result;
    }

    /**
     * Multiplies a 3x3 matrix by a 3-vector.
     */
    private static double[] multiply(double[][] matrix, double[] vector)
    {
        return new double[]
        {
            matrix[0][0]*vector[0] + matrix[0][1]*vector[1] + matrix[0][2]*vector[2],
            matrix[1][0]*vector[0] + matrix[1][1]*vector[1] + matrix[1][2]*vector[2],
            matrix[2][0]*vector[0] + matrix[2][1]*vector[1] + matrix[2][2]*vector[2]
        };
    }

    /**
     * Returns the transpose of a 3x3 matrix.
     */
    private static double[][] transpose(double[][] matrix)
    {
        return new double[][]
        {
            {matrix[0][0], matrix[1][0], matrix[2][0]},
            {matrix[0][1], matrix[1][1], matrix[2][1]},
            {matrix[0][2], matrix[1][2], matrix[2][2]}
        };
    }

    /**
     * Asserts that the orientation stored in a pose represents the supplied
     * independently calculated rotation matrix.
     *
     * <p>Comparing matrices instead of Euler components also avoids false
     * failures when two different Euler triples represent the same rotation.</p>
     */
    private static void assertRotationEquals(
        double[][] expected, TrcPose3D actual)
    {
        double[][] actualMatrix =
            oracleRotationMatrix(actual.pitch, actual.roll, actual.yaw);

        for (int row = 0; row < 3; row++)
        {
            for (int col = 0; col < 3; col++)
            {
                assertEquals(
                    expected[row][col], actualMatrix[row][col], EPSILON,
                    "R[" + row + "][" + col + "]");
            }
        }
    }

    /**
     * Verifies a general compound rotation against explicit matrix math.
     */
    @Test
    public void testRotateCompoundAgainstIndependentMatrixOracle()
    {
        TrcPose3D point = new TrcPose3D(3.0, -4.0, 5.0);
        double pitch = 27.0;
        double roll = -31.0;
        double yaw = 43.0;

        double[] expected = multiply(
            oracleRotationMatrix(pitch, roll, yaw),
            new double[] {point.x, point.y, point.z});

        TrcPose3D actual = point.rotate(pitch, roll, yaw);

        assertEquals(expected[0], actual.x, EPSILON, "x");
        assertEquals(expected[1], actual.y, EPSILON, "y");
        assertEquals(expected[2], actual.z, EPSILON, "z");
    }

    /**
     * Verifies rigid-transform composition against an independent SE(3)
     * matrix oracle:
     *
     *     R = Ra * Rb
     *     t = ta + Ra * tb
     */
    @Test
    public void testAddRelativePoseAgainstIndependentMatrixOracle()
    {
        TrcPose3D a =
            new TrcPose3D(11.0, -7.0, 19.0, 23.0, -34.0, 57.0);
        TrcPose3D b =
            new TrcPose3D(-5.0, 13.0, 2.0, -17.0, 29.0, -41.0);

        double[][] ra = oracleRotationMatrix(a.pitch, a.roll, a.yaw);
        double[][] rb = oracleRotationMatrix(b.pitch, b.roll, b.yaw);
        double[][] expectedRotation = multiply(ra, rb);
        double[] rotatedTranslation =
            multiply(ra, new double[] {b.x, b.y, b.z});

        double[] expectedTranslation = new double[]
        {
            a.x + rotatedTranslation[0],
            a.y + rotatedTranslation[1],
            a.z + rotatedTranslation[2]
        };

        TrcPose3D actual = a.addRelativePose(b);

        assertEquals(expectedTranslation[0], actual.x, EPSILON, "x");
        assertEquals(expectedTranslation[1], actual.y, EPSILON, "y");
        assertEquals(expectedTranslation[2], actual.z, EPSILON, "z");
        assertRotationEquals(expectedRotation, actual);
    }

    /**
     * Verifies rigid-transform inversion against an independent SE(3)
     * matrix oracle:
     *
     *     Rinv = R^T
     *     tinv = -R^T * t
     */
    @Test
    public void testInverseAgainstIndependentMatrixOracle()
    {
        TrcPose3D pose =
            new TrcPose3D(14.0, -9.0, 27.0, 32.0, -21.0, 68.0);

        double[][] rotation =
            oracleRotationMatrix(pose.pitch, pose.roll, pose.yaw);
        double[][] expectedRotation = transpose(rotation);
        double[] expectedTranslation = multiply(
            expectedRotation,
            new double[] {-pose.x, -pose.y, -pose.z});

        TrcPose3D actual = pose.inverse();

        assertEquals(expectedTranslation[0], actual.x, EPSILON, "x");
        assertEquals(expectedTranslation[1], actual.y, EPSILON, "y");
        assertEquals(expectedTranslation[2], actual.z, EPSILON, "z");
        assertRotationEquals(expectedRotation, actual);
    }

    /**
     * Verifies relativeTo() against the independent rigid-transform equation:
     *
     *     Rrel = Rref^T * Rtarget
     *     trel = Rref^T * (ttarget - tref)
     */
    @Test
    public void testRelativeToAgainstIndependentMatrixOracle()
    {
        TrcPose3D reference =
            new TrcPose3D(8.0, -12.0, 6.0, 19.0, 26.0, -37.0);
        TrcPose3D target =
            new TrcPose3D(-15.0, 31.0, 22.0, -28.0, 14.0, 73.0);

        double[][] rref = oracleRotationMatrix(
            reference.pitch, reference.roll, reference.yaw);
        double[][] rtarget = oracleRotationMatrix(
            target.pitch, target.roll, target.yaw);
        double[][] rrefInv = transpose(rref);
        double[][] expectedRotation = multiply(rrefInv, rtarget);
        double[] expectedTranslation = multiply(
            rrefInv,
            new double[]
            {
                target.x - reference.x,
                target.y - reference.y,
                target.z - reference.z
            });

        TrcPose3D actual = target.relativeTo(reference, true);

        assertEquals(expectedTranslation[0], actual.x, EPSILON, "x");
        assertEquals(expectedTranslation[1], actual.y, EPSILON, "y");
        assertEquals(expectedTranslation[2], actual.z, EPSILON, "z");
        assertRotationEquals(expectedRotation, actual);
    }

    /**
     * Verifies chained composition against a directly calculated independent
     * SE(3) result. This catches order errors that simple inverse round trips
     * can miss if composition and inverse contain matching mistakes.
     */
    @Test
    public void testChainedCompositionAgainstIndependentMatrixOracle()
    {
        TrcPose3D a =
            new TrcPose3D(4.0, -3.0, 11.0, 17.0, -24.0, 31.0);
        TrcPose3D b =
            new TrcPose3D(-6.0, 8.0, 2.0, -29.0, 13.0, 47.0);
        TrcPose3D c =
            new TrcPose3D(9.0, 1.0, -5.0, 21.0, 36.0, -18.0);

        double[][] ra = oracleRotationMatrix(a.pitch, a.roll, a.yaw);
        double[][] rb = oracleRotationMatrix(b.pitch, b.roll, b.yaw);
        double[][] rc = oracleRotationMatrix(c.pitch, c.roll, c.yaw);
        double[][] rab = multiply(ra, rb);
        double[][] expectedRotation = multiply(rab, rc);

        double[] rbTc = multiply(rb, new double[] {c.x, c.y, c.z});
        double[] tbPlusRbTc = new double[]
        {
            b.x + rbTc[0],
            b.y + rbTc[1],
            b.z + rbTc[2]
        };
        double[] raTerm = multiply(ra, tbPlusRbTc);
        double[] expectedTranslation = new double[]
        {
            a.x + raTerm[0],
            a.y + raTerm[1],
            a.z + raTerm[2]
        };

        TrcPose3D actual = a.addRelativePose(b).addRelativePose(c);

        assertEquals(expectedTranslation[0], actual.x, EPSILON, "x");
        assertEquals(expectedTranslation[1], actual.y, EPSILON, "y");
        assertEquals(expectedTranslation[2], actual.z, EPSILON, "z");
        assertRotationEquals(expectedRotation, actual);
    }

    /**
     * Exercises orientation extraction close to, but not at, the XYZ Euler
     * singularity. The oracle comparison is matrix-based because Euler angles
     * are not unique near a singularity.
     */
    @Test
    public void testNearGimbalLockAgainstIndependentMatrixOracle()
    {
        TrcPose3D a =
            new TrcPose3D(1.0, 2.0, 3.0, 12.0, 88.5, -33.0);
        TrcPose3D b =
            new TrcPose3D(-4.0, 5.0, 6.0, -7.0, 0.4, 19.0);

        double[][] ra = oracleRotationMatrix(a.pitch, a.roll, a.yaw);
        double[][] rb = oracleRotationMatrix(b.pitch, b.roll, b.yaw);
        double[][] expectedRotation = multiply(ra, rb);
        double[] rotatedTranslation =
            multiply(ra, new double[] {b.x, b.y, b.z});

        TrcPose3D actual = a.addRelativePose(b);

        assertEquals(a.x + rotatedTranslation[0], actual.x, EPSILON, "x");
        assertEquals(a.y + rotatedTranslation[1], actual.y, EPSILON, "y");
        assertEquals(a.z + rotatedTranslation[2], actual.z, EPSILON, "z");
        assertRotationEquals(expectedRotation, actual);
    }

}   //class TrcPose3DTest
