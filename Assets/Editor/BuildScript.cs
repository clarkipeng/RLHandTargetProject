using System;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

// Batchmode build used to verify the project, e.g.
//   tools/unity.sh -quit -executeMethod BuildScript.BuildMac -logFile Logs/build.log
public static class BuildScript
{
    public static void BuildMac()
    {
        var args = Environment.GetCommandLineArgs();
        int i = Array.IndexOf(args, "-buildOutput");
        var options = new BuildPlayerOptions
        {
            scenes = new[] { "Assets/Scenes/Playground.unity" },
            locationPathName = i >= 0 && i + 1 < args.Length ? args[i + 1] : "Builds/HandTarget.app",
            target = BuildTarget.StandaloneOSX,
            options = BuildOptions.None,
        };
        BuildSummary summary = BuildPipeline.BuildPlayer(options).summary;
        Debug.Log($"BuildScript: {summary.result} -> {summary.outputPath} ({summary.totalErrors} errors, {summary.totalTime})");
        if (summary.result != BuildResult.Succeeded)
            EditorApplication.Exit(1);
    }
}
