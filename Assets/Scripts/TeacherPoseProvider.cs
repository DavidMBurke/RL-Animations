using System.Collections.Generic;
using UnityEngine;

[ExecuteAlways]
public class TeacherPoseProvider : MonoBehaviour
{
    [Tooltip("Root transform that contains the teacher skeleton hierarchy (usually the FBX root).")]
    public Transform teacherRoot;

    [Header("Optional Animator (for phase randomization)")]
    public Animator animator;
    [Tooltip("Animator layer index to control.")]
    public int animatorLayer = 0;
    [Tooltip("State name to play for sampling (e.g. 'Idle_Stand'). Leave empty to not control state.")]
    public string animatorStateName;

    [Header("Animator sync")]
    [Tooltip("If true, forces the animator to update in fixed timestep (AnimatePhysics) and disables culling. This keeps teacher pose sampling aligned with physics/ML-Agents steps.")]
    public bool enforceAnimatorPhysicsSync = true;

    [Tooltip("If true, freezes the Animator (speed=0) and only changes pose when TrySetNormalizedTime is called. This prevents the teacher pose from drifting forward in real time between episodes.")]
    public bool freezeAnimatorTime = true;

    public float CurrentNormalizedTime { get; private set; }

    public Dictionary<string, Transform> BoneByName { get; private set; }

    void Awake()
    {
        ConfigureAnimator();
        EnsureBoneMap();
    }

    void OnEnable()
    {
        ConfigureAnimator();
        EnsureBoneMap();
    }

    void ConfigureAnimator()
    {
        if (animator == null) return;

        if (enforceAnimatorPhysicsSync)
        {
            // Prefer AnimatePhysics for consistency with FixedUpdate stepping.
            animator.updateMode = AnimatorUpdateMode.Fixed;
            animator.cullingMode = AnimatorCullingMode.AlwaysAnimate;
        }

        if (freezeAnimatorTime)
        {
            animator.speed = 0f;
        }
    }

#if UNITY_EDITOR
    void OnValidate()
    {
        if (Application.isPlaying) return;
        EnsureBoneMap();
    }
#endif

    void EnsureBoneMap()
    {
        if (teacherRoot == null) teacherRoot = transform;

        BoneByName ??= new Dictionary<string, Transform>(256);
        BoneByName.Clear();

        foreach (var t in teacherRoot.GetComponentsInChildren<Transform>(true))
        {
            if (!BoneByName.ContainsKey(t.name))
                BoneByName.Add(t.name, t);
        }
    }

    public bool TryGetBone(string boneName, out Transform bone)
    {
        if (BoneByName == null || BoneByName.Count == 0)
        {
            EnsureBoneMap();
        }
        return BoneByName.TryGetValue(boneName, out bone);
    }

    public bool TrySetNormalizedTime(float normalizedTime)
    {
        if (animator == null) return false;
        if (string.IsNullOrWhiteSpace(animatorStateName)) return false;

        normalizedTime = Mathf.Repeat(normalizedTime, 1f);
        CurrentNormalizedTime = normalizedTime;
        animator.Play(animatorStateName, animatorLayer, normalizedTime);
        // Ensure transforms update immediately.
        animator.Update(0f);
        return true;
    }

    void FixedUpdate()
    {
        // When frozen, keep the animator evaluated at the last requested time.
        if (!freezeAnimatorTime) return;
        if (animator == null) return;
        if (string.IsNullOrWhiteSpace(animatorStateName)) return;

        animator.speed = 0f;
        animator.Play(animatorStateName, animatorLayer, CurrentNormalizedTime);
        animator.Update(0f);
    }
}
