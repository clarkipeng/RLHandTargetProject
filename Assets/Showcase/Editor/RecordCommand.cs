using System;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace Showcase.Editor
{
    /// <summary>
    /// Records a rollout of a trained policy in the training scene, in batchmode play mode:
    ///   Unity -batchmode -nographics -projectPath . -executeMethod Showcase.Editor.RecordCommand.Run
    ///     -showcaseTake out.take [-showcaseSeconds 90] [-showcaseModel "Assets/Trained Models/X.onnx"]
    /// The model, when given, replaces the policy on every agent template for this run only (the scene is not saved).
    /// With no trainer connected the agents run inference. TakeRecorder writes the take and exits.
    /// </summary>
    public static class RecordCommand
    {
        public static void Run()
        {
            try
            {
                EditorSceneManager.OpenScene("Assets/Scenes/Playground.unity", OpenSceneMode.Single);
                string modelPath = Arg("-showcaseModel");
                if (!string.IsNullOrEmpty(modelPath))
                {
                    var model = AssetDatabase.LoadMainAssetAtPath(modelPath)
                                ?? throw new ArgumentException($"no model at {modelPath}");
                    var templates = Resources.FindObjectsOfTypeAll<Unity.MLAgents.Policies.BehaviorParameters>()
                        .Where(b => b.gameObject.scene.IsValid());
                    foreach (var b in templates)
                    {
                        var so = new SerializedObject(b);
                        so.FindProperty("m_Model").objectReferenceValue = model;
                        so.ApplyModifiedPropertiesWithoutUndo();
                        Debug.Log($"[showcase] {b.gameObject.name} now runs {modelPath}");
                    }
                }
                if (string.IsNullOrEmpty(Arg("-showcaseTake"))) throw new ArgumentException("missing -showcaseTake");
                EditorApplication.isPlaying = true; // TakeRecorder attaches on scene load and exits the editor when done
            }
            catch (Exception e)
            {
                Debug.LogException(e);
                EditorApplication.Exit(1);
            }
        }

        static string Arg(string name)
        {
            var args = Environment.GetCommandLineArgs();
            int i = Array.IndexOf(args, name);
            return i >= 0 && i + 1 < args.Length ? args[i + 1] : null;
        }
    }
}
