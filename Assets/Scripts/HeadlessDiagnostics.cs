using System;
using System.IO;
using Unity.MLAgents;
using UnityEngine;

public static class HeadlessDiagnostics
{
    private static string _logPath;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    private static void InitEarly()
    {
        try
        {
            var pid = System.Diagnostics.Process.GetCurrentProcess().Id;
            var port = TryGetArgValue("--mlagents-port") ?? "unknown";

            var dir = Path.Combine(Application.persistentDataPath, "mlagents_logs");
            Directory.CreateDirectory(dir);

            _logPath = Path.Combine(dir, $"player_pid{pid}_port{port}.log");
            Append($"=== Start {DateTime.Now:O} pid={pid} port={port} ===");

            Application.logMessageReceived += OnUnityLog;
        }
        catch
        {
            // Never let diagnostics break the build.
        }
    }

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    private static void ReportAfterSceneLoad()
    {
        try
        {
            Append($"AfterSceneLoad {DateTime.Now:O}");
            Append($"Academy.AutomaticSteppingEnabled={Academy.Instance.AutomaticSteppingEnabled}");
        }
        catch (Exception ex)
        {
            Append($"AfterSceneLoad exception: {ex.GetType().Name}: {ex.Message}");
        }
    }

    private static void OnUnityLog(string condition, string stackTrace, LogType type)
    {
        if (string.IsNullOrEmpty(_logPath))
            return;

        try
        {
            var msg = $"[{DateTime.Now:O}] {type}: {condition}";
            if (!string.IsNullOrWhiteSpace(stackTrace))
                msg += "\n" + stackTrace;
            Append(msg);
        }
        catch
        {
            // Ignore.
        }
    }

    private static void Append(string line)
    {
        if (string.IsNullOrEmpty(_logPath))
            return;

        try
        {
            File.AppendAllText(_logPath, line + "\n");
        }
        catch
        {
            // Ignore.
        }
    }

    private static string TryGetArgValue(string argName)
    {
        try
        {
            var args = Environment.GetCommandLineArgs();
            for (var i = 0; i < args.Length - 1; i++)
            {
                if (string.Equals(args[i], argName, StringComparison.OrdinalIgnoreCase))
                    return args[i + 1];
            }
        }
        catch
        {
            // Ignore.
        }

        return null;
    }
}
