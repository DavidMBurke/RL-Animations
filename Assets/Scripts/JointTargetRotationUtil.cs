using UnityEngine;

public static class JointTargetRotationUtil
{
    // Unity's ConfigurableJoint targetRotation is in joint space.
    // This helper converts a desired LOCAL rotation (bone local rotation target) into joint.targetRotation.
    //
    // Works well if:
    // - Linear motions are locked
    // - Hinges are configured with AngularY/Z locked
    // - joint.axis and joint.secondaryAxis are set consistently
    public static void SetTargetRotationLocal(ConfigurableJoint joint, Quaternion targetLocalRotation, Quaternion startLocalRotation)
    {
        // Convert from bone local space to "joint space"
        // The startLocalRotation is the bone's local rotation at initialization (bind/standing pose).
        Quaternion jointSpace = Quaternion.Inverse(Quaternion.LookRotation(joint.axis, joint.secondaryAxis));
        Quaternion result = jointSpace * Quaternion.Inverse(targetLocalRotation) * startLocalRotation * Quaternion.Inverse(jointSpace);

        // Unity expects targetRotation to be the inverse of what you might expect.
        joint.targetRotation = result;
    }

    // For true hinge joints (AngularY/Z locked), it's often easiest to specify a target angle around joint.axis.
    // This makes action parameterization very stable.
    public static Quaternion HingeLocalRotationFromAngle(Quaternion baseLocalRotation, Vector3 localHingeAxis, float angleDegrees)
    {
        // Rotate around hinge axis in local bone space from a base pose.
        return Quaternion.AngleAxis(angleDegrees, localHingeAxis) * baseLocalRotation;
    }
}
