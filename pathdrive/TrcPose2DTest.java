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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package trclib.pathdrive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.apache.commons.math3.linear.RealVector;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;

/**
 * This class implements JUnit tests to verify the correctness of the
 * TrcPose2D class implementation.
 */
public class TrcPose2DTest
{
    private static final double EPSILON = 1e-3;

    /**
     * Verifies all constructors.
     */
    @Test
    public void testConstructors()
    {
        TrcPose2D pose = new TrcPose2D(1.0, 2.0, 30.0);
        assertPoseEquals(new TrcPose2D(1.0, 2.0, 30.0), pose);

        pose = new TrcPose2D(new double[] {1.0, 2.0, 30.0});
        assertPoseEquals(new TrcPose2D(1.0, 2.0, 30.0), pose);

        pose = new TrcPose2D(1.0, 2.0);
        assertPoseEquals(new TrcPose2D(1.0, 2.0, 0.0), pose);

        pose = new TrcPose2D();
        assertPoseEquals(new TrcPose2D(0.0, 0.0, 0.0), pose);
    }

    /**
     * Verifies that the array constructor requires at least three elements.
     */
    @Test
    public void testArrayConstructorInvalidInput()
    {
        assertThrows(
            ArrayIndexOutOfBoundsException.class,
            () -> new TrcPose2D(new double[] {1.0, 2.0}));
    }

    /**
     * Verifies string representation.
     */
    @Test
    public void testToString()
    {
        TrcPose2D pose = new TrcPose2D(1.25, -2.5, 37.5);

        assertEquals("(x=1.250000,y=-2.500000,angle=37.500000)", pose.toString());
    }

    /**
     * Verifies conversion of the positional components to an Apache Commons
     * Math vector.
     */
    @Test
    public void testToPosVector()
    {
        TrcPose2D pose = new TrcPose2D(1.25, -2.5, 37.5);

        RealVector vector = pose.toPosVector();

        assertEquals(2, vector.getDimension());
        assertEquals(1.25, vector.getEntry(0), EPSILON);
        assertEquals(-2.5, vector.getEntry(1), EPSILON);
    }

    /**
     * Verifies that the returned position vector does not contain the angle.
     */
    @Test
    public void testToPosVectorExcludesAngle()
    {
        TrcPose2D pose = new TrcPose2D(1.0, 2.0, 90.0);

        RealVector vector = pose.toPosVector();

        assertEquals(2, vector.getDimension());
        assertEquals(1.0, vector.getEntry(0), EPSILON);
        assertEquals(2.0, vector.getEntry(1), EPSILON);
    }

    /**
     * Verifies the basic vector operations.
     * These operations operate only on position and preserve the receiver's
     * angle according to the TrcPose2D API semantics.
     */
    @Test
    public void testLinearVectorMath()
    {
        TrcPose2D pose1 = new TrcPose2D(1.0, 2.0, 30.0);
        TrcPose2D pose2 = new TrcPose2D(4.0, 5.0, 5.0);

        TrcPose2D sum = pose1.add(pose2);
        assertPoseEquals(new TrcPose2D(5.0, 7.0, 30.0), sum);

        TrcPose2D difference = pose1.subtract(pose2);
        assertPoseEquals(new TrcPose2D(-3.0, -3.0, 30.0), difference);

        TrcPose2D negated = pose1.negate();
        assertPoseEquals(new TrcPose2D(-1.0, -2.0, 30.0), negated);

        TrcPose2D scaled = pose1.scale(2.5);
        assertPoseEquals(new TrcPose2D(2.5, 5.0, 30.0), scaled);

        // Verify that the source poses were not modified.
        assertPoseEquals(new TrcPose2D(1.0, 2.0, 30.0), pose1);
        assertPoseEquals(new TrcPose2D(4.0, 5.0, 5.0), pose2);
    }

    /**
     * Verifies linear operations with zero and negative scale factors.
     */
    @Test
    public void testLinearVectorMathEdgeCases()
    {
        TrcPose2D pose = new TrcPose2D(3.0, -4.0, 25.0);

        assertPoseEquals(
            new TrcPose2D(0.0, 0.0, 25.0),
            pose.scale(0.0));

        assertPoseEquals(
            new TrcPose2D(-3.0, 4.0, 25.0),
            pose.scale(-1.0));

        assertPoseEquals(
            new TrcPose2D(-3.0, 4.0, 25.0),
            pose.negate());
    }

    /**
     * Verifies distance calculation.
     */
    @Test
    public void testDistanceTo()
    {
        TrcPose2D origin = new TrcPose2D(0.0, 0.0, 45.0);
        TrcPose2D target = new TrcPose2D(3.0, 4.0, 0.0);

        assertEquals(5.0, origin.distanceTo(target), EPSILON);
        assertEquals(5.0, target.distanceTo(origin), EPSILON);
        assertEquals(0.0, origin.distanceTo(origin), EPSILON);
    }

    /**
     * Verifies that distanceTo ignores orientation.
     */
    @Test
    public void testDistanceToIgnoresAngle()
    {
        TrcPose2D pose1 = new TrcPose2D(3.0, 4.0, 0.0);
        TrcPose2D pose2 = new TrcPose2D(0.0, 0.0, 180.0);

        assertEquals(5.0, pose1.distanceTo(pose2), EPSILON);
    }

    /**
     * Verifies clone, setAs, equals and hashCode.
     */
    @Test
    public void testObjectContracts()
    {
        TrcPose2D original = new TrcPose2D(1.2, 3.4, 30.0);

        TrcPose2D cloned = original.clone();

        assertPoseEquals(original, cloned);
        assertEquals(original, cloned);
        assertEquals(original.hashCode(), cloned.hashCode());

        TrcPose2D copied = new TrcPose2D();
        copied.setAs(original);

        assertPoseEquals(original, copied);
        assertEquals(original, copied);

        TrcPose2D differentX = new TrcPose2D(9.9, 3.4, 30.0);
        TrcPose2D differentY = new TrcPose2D(1.2, 9.9, 30.0);
        TrcPose2D differentAngle = new TrcPose2D(1.2, 3.4, 99.9);

        assertNotEquals(original, differentX);
        assertNotEquals(original, differentY);
        assertNotEquals(original, differentAngle);

        assertNotEquals(null, original);
        assertNotEquals("not a pose", original);
    }

    /**
     * Verifies that clone() creates an independent object.
     */
    @Test
    public void testCloneIndependence()
    {
        TrcPose2D original = new TrcPose2D(1.0, 2.0, 30.0);
        TrcPose2D cloned = original.clone();

        cloned.x = 10.0;
        cloned.y = 20.0;
        cloned.angle = 90.0;

        assertPoseEquals(new TrcPose2D(1.0, 2.0, 30.0), original);
        assertPoseEquals(new TrcPose2D(10.0, 20.0, 90.0), cloned);
    }

    /**
     * Verifies setAs() copies all components.
     */
    @Test
    public void testSetAs()
    {
        TrcPose2D source = new TrcPose2D(-5.0, 8.0, -45.0);
        TrcPose2D target = new TrcPose2D(100.0, 200.0, 300.0);

        target.setAs(source);

        assertPoseEquals(source, target);
    }

    /**
     * Verifies a local translation at zero heading.
     */
    @Test
    public void testTranslatePoseZeroHeading()
    {
        TrcPose2D pose = new TrcPose2D(10.0, 20.0, 0.0);

        TrcPose2D translated = pose.translatePose(3.0, 4.0);

        assertPoseEquals(new TrcPose2D(13.0, 24.0, 0.0), translated);
    }

    /**
     * Verifies a local translation at 90 degrees clockwise.
     * With TrcLib's coordinate convention (+X right, +Y forward,
     * clockwise-positive heading), local +Y maps to global +X.
     */
    @Test
    public void testTranslatePoseCardinalHeadings()
    {
        TrcPose2D pose = new TrcPose2D(10.0, 20.0, 0.0);

        assertPoseEquals(
            new TrcPose2D(13.0, 24.0, 0.0),
            pose.translatePose(3.0, 4.0));

        pose = new TrcPose2D(10.0, 20.0, 90.0);

        assertPoseEquals(
            new TrcPose2D(14.0, 17.0, 90.0),
            pose.translatePose(3.0, 4.0));

        pose = new TrcPose2D(10.0, 20.0, 180.0);

        assertPoseEquals(
            new TrcPose2D(7.0, 16.0, 180.0),
            pose.translatePose(3.0, 4.0));

        pose = new TrcPose2D(10.0, 20.0, 270.0);

        assertPoseEquals(
            new TrcPose2D(6.0, 23.0, 270.0),
            pose.translatePose(3.0, 4.0));
    }

    /**
     * Verifies a non-cardinal local translation.
     * This calculation is deliberately performed independently rather than
     * using TrcUtil.rotateCW(), so the test does not duplicate the
     * implementation being tested.
     */
    @Test
    public void testTranslatePose()
    {
        TrcPose2D pose = new TrcPose2D(10.0, 20.0, 30.0);
        TrcPose2D translated = pose.translatePose(4.0, 6.0);

        double angleRad = Math.toRadians(30.0);
        double expectedX = 10.0 + 4.0 * Math.cos(angleRad) + 6.0 * Math.sin(angleRad);
        double expectedY = 20.0 - 4.0 * Math.sin(angleRad) + 6.0 * Math.cos(angleRad);

        assertEquals(expectedX, translated.x, EPSILON);
        assertEquals(expectedY, translated.y, EPSILON);
        assertEquals(30.0, translated.angle, EPSILON);

        // Verify that the source was not modified.
        assertPoseEquals(new TrcPose2D(10.0, 20.0, 30.0), pose);
    }

    /**
     * Verifies translation by zero offset.
     */
    @Test
    public void testTranslatePoseZeroOffset()
    {
        TrcPose2D pose = new TrcPose2D(10.0, 20.0, 37.0);

        assertPoseEquals(pose, pose.translatePose(0.0, 0.0));
    }

    @Test
    public void testRotate()
    {
        TrcPose2D pose = new TrcPose2D(10.0, 5.0, 20.0);
        TrcPose2D rotated = pose.rotate(30.0);

        double angleRad = Math.toRadians(30.0);
        double expectedX = 10.0 * Math.cos(angleRad) + 5.0 * Math.sin(angleRad);
        double expectedY = -10.0 * Math.sin(angleRad) + 5.0 * Math.cos(angleRad);

        assertEquals(expectedX, rotated.x, EPSILON);
        assertEquals(expectedY, rotated.y, EPSILON);
        assertEquals(20.0, rotated.angle, EPSILON);

        // Verify the original pose is unchanged.
        assertPoseEquals(new TrcPose2D(10.0, 5.0, 20.0), pose);
    }

    /**
     * Verifies rotation around the world origin at cardinal angles without
     * changing the pose's orientation.
     */
    @Test
    public void testRotateCardinalAngles()
    {
        TrcPose2D pose = new TrcPose2D(3.0, 4.0, 10.0);

        assertPoseEquals(
            new TrcPose2D(3.0, 4.0, 10.0),
            pose.rotate(0.0));

        assertPoseEquals(
            new TrcPose2D(4.0, -3.0, 10.0),
            pose.rotate(90.0));

        assertPoseEquals(
            new TrcPose2D(-3.0, -4.0, 10.0),
            pose.rotate(180.0));

        assertPoseEquals(
            new TrcPose2D(-4.0, 3.0, 10.0),
            pose.rotate(270.0));
    }

    /**
     * Verifies negative rotation without changing the pose's orientation.
     */
    @Test
    public void testRotateNegativeAngle()
    {
        TrcPose2D pose = new TrcPose2D(3.0, 4.0, 20.0);
        TrcPose2D rotated = pose.rotate(-90.0);

        assertPoseEquals(
            new TrcPose2D(-4.0, 3.0, 20.0),
            rotated);
    }

    @Test
    public void testRotatePose()
    {
        TrcPose2D pose = new TrcPose2D(10.0, 5.0, 20.0);
        TrcPose2D rotated = pose.rotatePose(30.0);

        double angleRad = Math.toRadians(30.0);
        double expectedX = 10.0 * Math.cos(angleRad) + 5.0 * Math.sin(angleRad);
        double expectedY = -10.0 * Math.sin(angleRad) + 5.0 * Math.cos(angleRad);

        assertEquals(expectedX, rotated.x, EPSILON);
        assertEquals(expectedY, rotated.y, EPSILON);
        assertEquals(50.0, rotated.angle, EPSILON);

        // Verify the original pose is unchanged.
        assertPoseEquals(new TrcPose2D(10.0, 5.0, 20.0), pose);
    }

    /**
     * Verifies rotation around the world origin at cardinal angles.
     */
    @Test
    public void testRotatePoseCardinalAngles()
    {
        TrcPose2D pose = new TrcPose2D(3.0, 4.0, 10.0);

        assertPoseEquals(
            new TrcPose2D(3.0, 4.0, 10.0),
            pose.rotatePose(0.0));

        assertPoseEquals(
            new TrcPose2D(4.0, -3.0, 100.0),
            pose.rotatePose(90.0));

        assertPoseEquals(
            new TrcPose2D(-3.0, -4.0, 190.0),
            pose.rotatePose(180.0));

        assertPoseEquals(
            new TrcPose2D(-4.0, 3.0, 280.0),
            pose.rotatePose(270.0));
    }

    /**
     * Verifies negative rotation.
     */
    @Test
    public void testRotatePoseNegativeAngle()
    {
        TrcPose2D pose = new TrcPose2D(3.0, 4.0, 20.0);
        TrcPose2D rotated = pose.rotatePose(-90.0);

        assertPoseEquals(
            new TrcPose2D(-4.0, 3.0, -70.0),
            rotated);
    }

    /**
     * Verifies composition of a relative pose with a global pose.
     */
    @Test
    public void testAddRelativePose()
    {
        TrcPose2D robotPose = new TrcPose2D(10.0, 5.0, 90.0);
        TrcPose2D relativePose = new TrcPose2D(0.0, 2.0, 45.0);

        TrcPose2D globalPose = robotPose.addRelativePose(relativePose);

        assertPoseEquals(
            new TrcPose2D(12.0, 5.0, 135.0),
            globalPose);

        robotPose = new TrcPose2D(10.0, 20.0, 30.0);
        relativePose = new TrcPose2D(4.0, 6.0, 15.0);

        globalPose = robotPose.addRelativePose(relativePose);

        double angleRad = Math.toRadians(30.0);
        double expectedX =
            10.0 + 4.0 * Math.cos(angleRad) + 6.0 * Math.sin(angleRad);
        double expectedY =
            20.0 - 4.0 * Math.sin(angleRad) + 6.0 * Math.cos(angleRad);

        assertEquals(expectedX, globalPose.x, EPSILON);
        assertEquals(expectedY, globalPose.y, EPSILON);
        assertEquals(45.0, globalPose.angle, EPSILON);
    }

    /**
     * Verifies that adding the identity relative pose leaves the pose
     * unchanged.
     */
    @Test
    public void testAddRelativePoseIdentity()
    {
        TrcPose2D pose = new TrcPose2D(12.0, -7.0, 35.0);

        assertPoseEquals(
            pose,
            pose.addRelativePose(new TrcPose2D()));
    }

    /**
     * Verifies that relativeTo is the inverse of addRelativePose.
     */
    @Test
    public void testRelativeToAndAddRelativePoseRoundTrip()
    {
        TrcPose2D reference = new TrcPose2D(10.0, 5.0, 37.0);
        TrcPose2D target = new TrcPose2D(16.0, 13.0, 82.0);

        TrcPose2D relative = target.relativeTo(reference, true);
        TrcPose2D reconstructed = reference.addRelativePose(relative);

        assertPoseEquals(target, reconstructed);
    }

    /**
     * Verifies the default relativeTo() overload.
     * The default behavior must transform the orientation.
     */
    @Test
    public void testRelativeToDefault()
    {
        TrcPose2D reference = new TrcPose2D(10.0, 5.0, 90.0);
        TrcPose2D target = new TrcPose2D(10.0, 10.0, 135.0);

        TrcPose2D relativePose = target.relativeTo(reference);

        assertPoseEquals(
            new TrcPose2D(-5.0, 0.0, 45.0),
            relativePose);
    }

    /**
     * Verifies the position and heading returned by relativeTo for a simple
     * cardinal-heading case.
     */
    @Test
    public void testRelativeTo()
    {
        TrcPose2D robotPose = new TrcPose2D(10.0, 5.0, 90.0);
        TrcPose2D targetPose = new TrcPose2D(10.0, 10.0, 135.0);

        TrcPose2D relativePose = targetPose.relativeTo(robotPose, true);

        assertPoseEquals(
            new TrcPose2D(-5.0, 0.0, 45.0),
            relativePose);
    }

    /**
     * Verifies the transformAngle=false behavior of relativeTo.
     * Position is still expressed in the reference frame, but the target's
     * original heading is retained.
     */
    @Test
    public void testRelativeToWithoutAngleTransform()
    {
        TrcPose2D reference = new TrcPose2D(10.0, 5.0, 90.0);
        TrcPose2D target = new TrcPose2D(10.0, 10.0, 135.0);

        TrcPose2D relativePose = target.relativeTo(reference, false);

        assertEquals(-5.0, relativePose.x, EPSILON);
        assertEquals(0.0, relativePose.y, EPSILON);
        assertEquals(135.0, relativePose.angle, EPSILON);
    }

    /**
     * Verifies relativeTo() when the reference pose is the identity.
     */
    @Test
    public void testRelativeToIdentity()
    {
        TrcPose2D target = new TrcPose2D(7.5, -4.0, 37.0);
        TrcPose2D identity = new TrcPose2D();

        assertPoseEquals(target, target.relativeTo(identity));
    }

    /**
     * Verifies that a pose is its own relative transform with respect to
     * itself.
     */
    @Test
    public void testRelativeToSelf()
    {
        TrcPose2D pose = new TrcPose2D(7.5, -4.0, 37.0);

        assertPoseEquals(
            new TrcPose2D(0.0, 0.0, 0.0),
            pose.relativeTo(pose));
    }

    /**
     * Verifies inverse() for a cardinal case.
     */
    @Test
    public void testInverse()
    {
        TrcPose2D pose = new TrcPose2D(0.0, 5.0, 90.0);
        TrcPose2D inverse = pose.inverse();

        assertPoseEquals(
            new TrcPose2D(5.0, 0.0, -90.0),
            inverse);
    }

    /**
     * Verifies inverse() for an arbitrary pose using the mathematical
     * inverse-transform formula.
     */
    @Test
    public void testInverseArbitraryPose()
    {
        TrcPose2D pose = new TrcPose2D(4.5, -2.75, 123.0);
        TrcPose2D inverse = pose.inverse();

        double angleRad = Math.toRadians(pose.angle);

        double expectedX =
            -pose.x * Math.cos(angleRad) + pose.y * Math.sin(angleRad);
        double expectedY =
            -pose.x * Math.sin(angleRad) - pose.y * Math.cos(angleRad);

        assertEquals(expectedX, inverse.x, EPSILON);
        assertEquals(expectedY, inverse.y, EPSILON);
        assertEquals(-pose.angle, inverse.angle, EPSILON);
    }

    /**
     * Verifies that inverse() of the identity is the identity.
     */
    @Test
    public void testInverseIdentity()
    {
        TrcPose2D identity = new TrcPose2D();

        assertPoseEquals(identity, identity.inverse());
    }

    /**
     * Verifies the fundamental inverse-transform identity:
     *     pose.addRelativePose(pose.inverse()) = identity
     * and the reverse composition as well.
     */
    @Test
    public void testInverseComposition()
    {
        TrcPose2D pose = new TrcPose2D(12.5, -7.25, 37.0);
        TrcPose2D inverse = pose.inverse();
        TrcPose2D identity = new TrcPose2D();

        TrcPose2D result1 = pose.addRelativePose(inverse);
        TrcPose2D result2 = inverse.addRelativePose(pose);

        assertPoseEquals(identity, result1);
        assertPoseEquals(identity, result2);
    }

    /**
     * Verifies that relativeTo() agrees with explicitly applying the inverse
     * transform.
     */
    @Test
    public void testRelativeToInverseConsistency()
    {
        TrcPose2D reference = new TrcPose2D(-25.0, 13.0, -123.0);
        TrcPose2D target = new TrcPose2D(7.5, -4.25, 67.0);

        TrcPose2D relative = target.relativeTo(reference);
        TrcPose2D viaInverse = reference.inverse().addRelativePose(target);

        assertPoseEquals(viaInverse, relative);
    }

    /**
     * Verifies that an arbitrary relative transform can be recovered after
     * composing it with a reference pose.
     */
    @Test
    public void testArbitraryRelativeTransformRoundTrip()
    {
        TrcPose2D reference = new TrcPose2D(-25.0, 13.0, -123.0);
        TrcPose2D relative = new TrcPose2D(7.5, -4.25, 67.0);

        TrcPose2D target = reference.addRelativePose(relative);
        TrcPose2D recoveredRelative = target.relativeTo(reference, true);

        assertPoseEquals(relative, recoveredRelative);
    }

    /**
     * Verifies composition associativity through the relative-pose transform.
     * (A * B) * C must equal A * (B * C).
     */
    @Test
    public void testRelativePoseCompositionAssociativity()
    {
        TrcPose2D a = new TrcPose2D(10.0, 20.0, 30.0);
        TrcPose2D b = new TrcPose2D(4.0, -3.0, 15.0);
        TrcPose2D c = new TrcPose2D(-2.0, 5.0, -20.0);

        TrcPose2D left =
            a.addRelativePose(b).addRelativePose(c);

        TrcPose2D right =
            a.addRelativePose(b.addRelativePose(c));

        assertPoseEquals(left, right);
    }

    /**
     * Verifies that rotatePose() preserves the distance from the origin.
     */
    @Test
    public void testRotatePosePreservesRadius()
    {
        TrcPose2D pose = new TrcPose2D(12.0, -5.0, 47.0);
        double originalRadius = Math.hypot(pose.x, pose.y);

        TrcPose2D rotated = pose.rotatePose(123.0);

        assertEquals(originalRadius, Math.hypot(rotated.x, rotated.y), EPSILON);
    }

    /**
     * Verifies that translating a pose does not alter its orientation.
     */
    @Test
    public void testTranslatePosePreservesAngle()
    {
        TrcPose2D pose = new TrcPose2D(3.0, 4.0, 123.0);

        assertEquals(
            pose.angle,
            pose.translatePose(-10.0, 25.0).angle,
            EPSILON);
    }

    /**
     * Verifies CSV loading from the file system.
     */
    @Test
    public void testLoadPosesFromCsvFile() throws Exception
    {
        Path tempFile = Files.createTempFile("TrcPose2DTest", ".csv");

        try
        {
            Files.write(
                tempFile,
                ("x,y,angle\n1.0,2.0,30.0\n3.5, -4.25, 90.0\n\n-5.0,6.0,-45.0\n").getBytes(StandardCharsets.UTF_8));

            TrcPose2D[] poses =
                TrcPose2D.loadPosesFromCsv(tempFile.toString(), false);

            assertEquals(3, poses.length);

            assertPoseEquals(
                new TrcPose2D(1.0, 2.0, 30.0),
                poses[0]);

            assertPoseEquals(
                new TrcPose2D(3.5, -4.25, 90.0),
                poses[1]);

            assertPoseEquals(
                new TrcPose2D(-5.0, 6.0, -45.0),
                poses[2]);
        }
        finally
        {
            Files.deleteIfExists(tempFile);
        }
    }

    /**
     * Verifies that a CSV file containing only a header produces an empty
     * pose array.
     */
    @Test
    public void testLoadPosesFromCsvEmptyFile() throws Exception
    {
        Path tempFile = Files.createTempFile("TrcPose2DTest", ".csv");

        try
        {
            Files.write(tempFile, ("x,y,angle\n").getBytes(StandardCharsets.UTF_8));

            TrcPose2D[] poses =
                TrcPose2D.loadPosesFromCsv(tempFile.toString(), false);

            assertEquals(0, poses.length);
        }
        finally
        {
            Files.deleteIfExists(tempFile);
        }
    }

    /**
     * Verifies that a CSV file containing a blank line is handled correctly.
     */
    @Test
    public void testLoadPosesFromCsvBlankLines() throws Exception
    {
        Path tempFile = Files.createTempFile("TrcPose2DTest", ".csv");

        try
        {
            Files.write(
                tempFile,
                ("x,y,angle\n\n1.0,2.0,3.0\n\n").getBytes(StandardCharsets.UTF_8));

            TrcPose2D[] poses =
                TrcPose2D.loadPosesFromCsv(tempFile.toString(), false);

            assertEquals(1, poses.length);
            assertPoseEquals(
                new TrcPose2D(1.0, 2.0, 3.0),
                poses[0]);
        }
        finally
        {
            Files.deleteIfExists(tempFile);
        }
    }

    /**
     * Verifies the CSV extension check.
     */
    @Test
    public void testLoadPosesFromCsvInvalidExtension()
    {
        IllegalArgumentException exception = assertThrows(
            IllegalArgumentException.class,
            () -> TrcPose2D.loadPosesFromCsv("poses.txt", false));

        assertEquals(
            "poses.txt is not a csv file!",
            exception.getMessage());
    }

    /**
     * Verifies malformed CSV column counts.
     */
    @Test
    public void testLoadPosesFromCsvInvalidColumnCount() throws Exception
    {
        Path tempFile = Files.createTempFile("TrcPose2DTest", ".csv");

        try
        {
            Files.write(
                tempFile,
                ("x,y,angle\n1.0,2.0\n").getBytes(StandardCharsets.UTF_8));

            assertThrows(
                IllegalArgumentException.class,
                () -> TrcPose2D.loadPosesFromCsv(tempFile.toString(), false));
        }
        finally
        {
            Files.deleteIfExists(tempFile);
        }
    }

    /**
     * Verifies malformed numeric data.
     */
    @Test
    public void testLoadPosesFromCsvInvalidNumber() throws Exception
    {
        Path tempFile = Files.createTempFile("TrcPose2DTest", ".csv");

        try
        {
            Files.write(
                tempFile,
                ("x,y,angle\n1.0,not-a-number,30.0\n").getBytes(StandardCharsets.UTF_8));

            assertThrows(
                NumberFormatException.class,
                () -> TrcPose2D.loadPosesFromCsv(tempFile.toString(), false));
        }
        finally
        {
            Files.deleteIfExists(tempFile);
        }
    }

    /**
     * Verifies the missing-resource behavior.
     */
    @Test
    public void testLoadPosesFromCsvMissingResource()
    {
        RuntimeException exception = assertThrows(
            RuntimeException.class,
            () -> TrcPose2D.loadPosesFromCsv(
                "does-not-exist.csv",
                true));

        assertNotNull(exception.getCause());
        assertEquals(IOException.class, exception.getCause().getClass());
    }

    /**
     * Compares two poses using a tolerance appropriate for floating-point
     * geometric calculations.
     */
    private static void assertPoseEquals(
        TrcPose2D expected, TrcPose2D actual)
    {
        assertEquals(expected.x, actual.x, EPSILON, "x");
        assertEquals(expected.y, actual.y, EPSILON, "y");
        assertEquals(expected.angle, actual.angle, EPSILON, "angle");
    }
}   //class TrcPose2DTest
