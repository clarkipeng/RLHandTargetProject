using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace Showcase
{
    public enum ActorKind : byte { Arm = 0, Ball = 1, Target = 2, Prop = 3 }

    /// <summary>
    /// One rigid visual part of an actor, sampled once per physics step.
    /// </summary>
    public struct PartPose
    {
        public Vector3 position;
        public Quaternion rotation;
        public Vector3 scale;

        public static PartPose Lerp(in PartPose a, in PartPose b, float t) => new PartPose
        {
            position = Vector3.LerpUnclamped(a.position, b.position, t),
            rotation = Quaternion.SlerpUnclamped(a.rotation, b.rotation, t),
            scale = Vector3.LerpUnclamped(a.scale, b.scale, t),
        };
    }

    /// <summary>
    /// A spawned object (the arm, a ball, a target) and its part poses from birth to death.
    /// Part paths are relative to the actor root, so a replay can find each mesh in the actor's prefab.
    /// </summary>
    public class TakeActor
    {
        public int id;
        public ActorKind kind;
        public string[] partPaths;
        public int birthStep;
        public int deathStep = -1; // -1: alive until the end of the take
        public bool hit;           // ball: hit its target; target: was hit
        public readonly List<PartPose[]> frames = new List<PartPose[]>();

        public int LastStep => birthStep + frames.Count - 1;
        public bool AliveAt(float step) => step >= birthStep && step <= LastStep;

        public PartPose Sample(int part, float step)
        {
            float local = Mathf.Clamp(step - birthStep, 0, frames.Count - 1);
            int i = Mathf.FloorToInt(local);
            int j = Mathf.Min(i + 1, frames.Count - 1);
            return PartPose.Lerp(frames[i][part], frames[j][part], local - i);
        }
    }

    /// <summary>
    /// A recorded rollout: everything needed to re-render a policy's motion from any camera,
    /// at any frame rate and shutter, without re-running physics or the policy.
    /// </summary>
    public class Take
    {
        const string Magic = "SHOWTAKE";
        const int Version = 1;

        public float stepSeconds;
        public int stepCount;
        public string policy = "";
        public readonly List<TakeActor> actors = new List<TakeActor>();
        public readonly List<int> episodeStarts = new List<int>();
        public readonly List<int> hitSteps = new List<int>();

        public float Duration => stepCount * stepSeconds;

        public void Save(string path)
        {
            Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(path)));
            using var w = new BinaryWriter(File.Create(path));
            w.Write(Magic);
            w.Write(Version);
            w.Write(stepSeconds);
            w.Write(stepCount);
            w.Write(policy);
            WriteInts(w, episodeStarts);
            WriteInts(w, hitSteps);
            w.Write(actors.Count);
            foreach (var a in actors)
            {
                w.Write(a.id);
                w.Write((byte)a.kind);
                w.Write(a.partPaths.Length);
                foreach (var p in a.partPaths) w.Write(p);
                w.Write(a.birthStep);
                w.Write(a.deathStep);
                w.Write(a.hit);
                w.Write(a.frames.Count);
                foreach (var frame in a.frames)
                    foreach (var pose in frame)
                    {
                        WriteV3(w, pose.position);
                        w.Write(pose.rotation.x); w.Write(pose.rotation.y); w.Write(pose.rotation.z); w.Write(pose.rotation.w);
                        WriteV3(w, pose.scale);
                    }
            }
        }

        public static Take Load(string path)
        {
            using var r = new BinaryReader(File.OpenRead(path));
            if (r.ReadString() != Magic) throw new IOException($"{path} is not a showcase take");
            int version = r.ReadInt32();
            if (version != Version) throw new IOException($"{path} has take version {version}, expected {Version}");
            var take = new Take { stepSeconds = r.ReadSingle(), stepCount = r.ReadInt32(), policy = r.ReadString() };
            ReadInts(r, take.episodeStarts);
            ReadInts(r, take.hitSteps);
            int actorCount = r.ReadInt32();
            for (int i = 0; i < actorCount; i++)
            {
                var a = new TakeActor { id = r.ReadInt32(), kind = (ActorKind)r.ReadByte() };
                a.partPaths = new string[r.ReadInt32()];
                for (int p = 0; p < a.partPaths.Length; p++) a.partPaths[p] = r.ReadString();
                a.birthStep = r.ReadInt32();
                a.deathStep = r.ReadInt32();
                a.hit = r.ReadBoolean();
                int frameCount = r.ReadInt32();
                for (int f = 0; f < frameCount; f++)
                {
                    var frame = new PartPose[a.partPaths.Length];
                    for (int p = 0; p < frame.Length; p++)
                    {
                        frame[p].position = ReadV3(r);
                        frame[p].rotation = new Quaternion(r.ReadSingle(), r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
                        frame[p].scale = ReadV3(r);
                    }
                    a.frames.Add(frame);
                }
                take.actors.Add(a);
            }
            return take;
        }

        static void WriteV3(BinaryWriter w, Vector3 v) { w.Write(v.x); w.Write(v.y); w.Write(v.z); }
        static Vector3 ReadV3(BinaryReader r) => new Vector3(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
        static void WriteInts(BinaryWriter w, List<int> xs) { w.Write(xs.Count); foreach (var x in xs) w.Write(x); }
        static void ReadInts(BinaryReader r, List<int> xs) { int n = r.ReadInt32(); for (int i = 0; i < n; i++) xs.Add(r.ReadInt32()); }
    }
}
