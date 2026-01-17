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

    public Dictionary<string, Transform> BoneByName { get; private set; }

    void Awake()
    {
        EnsureBoneMap();
    }

    void OnEnable()
    {
        EnsureBoneMap();
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
        animator.Play(animatorStateName, animatorLayer, normalizedTime);
        // Ensure transforms update immediately.
        animator.Update(0f);
        return true;
    }
}
