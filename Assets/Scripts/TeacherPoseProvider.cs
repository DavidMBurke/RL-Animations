using System.Collections.Generic;
using UnityEngine;

public class TeacherPoseProvider : MonoBehaviour
{
    [Tooltip("Root transform that contains the teacher skeleton hierarchy (usually the FBX root).")]
    public Transform teacherRoot;

    public Dictionary<string, Transform> BoneByName { get; private set; }

    void Awake()
    {
        if (teacherRoot == null) teacherRoot = transform;
        BoneByName = new Dictionary<string, Transform>(256);

        foreach (var t in teacherRoot.GetComponentsInChildren<Transform>())
        {
            if (!BoneByName.ContainsKey(t.name))
                BoneByName.Add(t.name, t);
        }
    }

    public bool TryGetBone(string boneName, out Transform bone)
    {
        return BoneByName.TryGetValue(boneName, out bone);
    }
}
