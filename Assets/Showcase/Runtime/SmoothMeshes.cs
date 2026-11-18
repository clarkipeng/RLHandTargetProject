using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace Showcase
{
    /// <summary>
    /// Visual-only stand-ins for the low-poly training meshes. A part that is a sphere, capsule or box
    /// is rebuilt as a dense rounded shape with the same bounds; anything else keeps its mesh.
    /// Every shape is a Minkowski sum of a core box and an ellipsoid, built from a cube-sphere grid,
    /// so faces stay flat, edges and caps stay round, and the rounding is in world units.
    /// </summary>
    public static class SmoothMeshes
    {
        public enum Shape { Other, Ellipsoid, Capsule, Box }

        /// <summary>Rounding of box edges, as a fraction of the box's middle half-extent (capped by the smallest).</summary>
        public static float boxRounding = 0.22f;
        /// <summary>Grid cells per half face of the cube-sphere; detail of the round parts.</summary>
        public static int detail = 24;

        static readonly Dictionary<(Mesh, Vector3Int), Mesh> cache = new Dictionary<(Mesh, Vector3Int), Mesh>();
        static readonly Dictionary<Mesh, (Shape, int axis)> shapes = new Dictionary<Mesh, (Shape, int)>();

        /// <summary>The smooth stand-in for a mesh drawn at the given world scale (or the mesh itself).</summary>
        public static Mesh For(Mesh source, Vector3 worldScale)
        {
            if (source == null) return source;
#if !UNITY_EDITOR
            if (!source.isReadable) return source; // player builds can only rebuild meshes marked readable
#endif
            var (shape, axis) = Classify(source);
            if (shape == Shape.Other) return source;
            var s = Abs(worldScale);
            if (s.x < 1e-6f || s.y < 1e-6f || s.z < 1e-6f) return source;
            var key = (source, Vector3Int.RoundToInt(s * 1000f));
            if (cache.TryGetValue(key, out var mesh)) return mesh;

            var b = source.bounds;
            var E = Vector3.Scale(b.extents, s); // world half-extents
            Vector3 R, C;
            switch (shape)
            {
                case Shape.Ellipsoid:
                    R = E; C = Vector3.zero;
                    break;
                case Shape.Capsule:
                {
                    int i = (axis + 1) % 3, j = (axis + 2) % 3;
                    R = E; R[axis] = Mathf.Min(E[axis], 0.5f * (E[i] + E[j]));
                    C = Vector3.zero; C[axis] = E[axis] - R[axis];
                    break;
                }
                default:
                {
                    // Thin plates (fingers) round over their whole thickness instead of a sliver of it.
                    float min = Mathf.Min(E.x, Mathf.Min(E.y, E.z)), max = Mathf.Max(E.x, Mathf.Max(E.y, E.z));
                    float mid = E.x + E.y + E.z - min - max;
                    float r = Mathf.Min(0.92f * min, boxRounding * mid);
                    R = Vector3.one * r; C = E - R;
                    break;
                }
            }
            mesh = Rounded(C, R, detail);
            // Back to the part's local space: the renderer applies the scale again.
            var v = mesh.vertices; var n = mesh.normals;
            for (int k = 0; k < v.Length; k++)
            {
                v[k] = new Vector3(v[k].x / s.x, v[k].y / s.y, v[k].z / s.z) + b.center;
                n[k] = Vector3.Scale(n[k], s).normalized;
            }
            mesh.vertices = v; mesh.normals = n;
            mesh.RecalculateBounds();
            mesh.RecalculateTangents();
            mesh.name = $"{source.name} ({shape}, smooth)";
            cache[key] = mesh;
            return mesh;
        }

        /// <summary>For a capsule part: the centers of its two caps in the part's local space, and the cap radius in world units.</summary>
        public static bool CapsuleEnds(Mesh source, Vector3 worldScale, out Vector3 a, out Vector3 b, out float worldRadius)
        {
            a = b = default; worldRadius = 0f;
            if (source == null) return false;
            var (shape, axis) = Classify(source);
            if (shape != Shape.Capsule) return false;
            var s = Abs(worldScale);
            var E = Vector3.Scale(source.bounds.extents, s);
            int i = (axis + 1) % 3, j = (axis + 2) % 3;
            worldRadius = Mathf.Min(E[axis], 0.5f * (E[i] + E[j]));
            var offset = Vector3.zero; offset[axis] = (E[axis] - worldRadius) / s[axis];
            a = source.bounds.center - offset;
            b = source.bounds.center + offset;
            return true;
        }

        /// <summary>A dense sphere of radius 0.5, the size of Unity's built-in one.</summary>
        public static Mesh Sphere
        {
            get
            {
                if (sphere == null) { sphere = Rounded(Vector3.zero, Vector3.one * 0.5f, detail); sphere.RecalculateBounds(); sphere.name = "Sphere (smooth)"; }
                return sphere;
            }
        }
        static Mesh sphere;

        public static (Shape shape, int axis) Classify(Mesh m)
        {
            if (shapes.TryGetValue(m, out var known)) return known;
            var result = ClassifyVertices(m.vertices, m.bounds);
            shapes[m] = result;
            Debug.Log($"[showcase] mesh '{m.name}' ({m.vertexCount} verts, size {m.bounds.size}) -> {result.Item1}");
            return result;
        }

        static (Shape, int) ClassifyVertices(Vector3[] verts, Bounds b)
        {
            var c = b.center; var E = b.extents;
            float size = Mathf.Max(E.x, Mathf.Max(E.y, E.z));
            if (verts.Length < 8 || Mathf.Min(E.x, Mathf.Min(E.y, E.z)) < 1e-5f) return (Shape.Other, 0);
            float tol = 0.03f;

            bool ellipsoid = true;
            foreach (var v in verts)
            {
                var d = v - c;
                float q = Sq(d.x / E.x) + Sq(d.y / E.y) + Sq(d.z / E.z);
                if (Mathf.Abs(Mathf.Sqrt(q) - 1f) > tol) { ellipsoid = false; break; }
            }
            if (ellipsoid) return (Shape.Ellipsoid, 0);

            int axis = E.x >= E.y && E.x >= E.z ? 0 : E.y >= E.z ? 1 : 2;
            int i = (axis + 1) % 3, j = (axis + 2) % 3;
            if (Mathf.Abs(E[i] - E[j]) < tol * size && E[axis] > E[i] * 1.05f)
            {
                float r = 0.5f * (E[i] + E[j]), h = E[axis] - r;
                bool capsule = true, roundCap = false;
                foreach (var v in verts)
                {
                    var d = v - c;
                    float along = Mathf.Clamp(d[axis], -h, h);
                    var p = d; p[axis] -= along;
                    if (Mathf.Abs(p.magnitude - r) > tol * size) { capsule = false; break; }
                    if (Mathf.Abs(d[axis]) > h + 0.2f * r && Mathf.Abs(d[axis]) < E[axis] - 0.1f * r) roundCap = true;
                }
                if (capsule && roundCap) return (Shape.Capsule, axis);
            }

            foreach (var v in verts)
            {
                var d = v - c;
                bool onFace = Mathf.Abs(Mathf.Abs(d.x) - E.x) < tol * size
                              || Mathf.Abs(Mathf.Abs(d.y) - E.y) < tol * size
                              || Mathf.Abs(Mathf.Abs(d.z) - E.z) < tol * size;
                if (!onFace) return (Shape.Other, 0);
            }
            return (Shape.Box, 0);
        }

        /// <summary>
        /// Minkowski sum of the box [-C, C] and the ellipsoid with radii R, centered at the origin.
        /// Each cube face is a grid whose middle row and column are doubled: the doubled seam is where
        /// the support point jumps between core corners, and its zero-width quads become the flat faces.
        /// </summary>
        static Mesh Rounded(Vector3 C, Vector3 R, int k)
        {
            int n = 2 * k + 2; // vertices per face side
            var verts = new List<Vector3>(6 * n * n);
            var normals = new List<Vector3>(6 * n * n);
            var uvs = new List<Vector2>(6 * n * n);
            var tris = new List<int>(6 * (n - 1) * (n - 1) * 6);
            for (int face = 0; face < 6; face++)
            {
                int a = face / 2; float sa = face % 2 == 0 ? 1f : -1f;
                int u = (a + 1) % 3, w = (a + 2) % 3;
                int start = verts.Count;
                for (int iy = 0; iy < n; iy++)
                for (int ix = 0; ix < n; ix++)
                {
                    float gx = Coord(ix, k, out float sx), gy = Coord(iy, k, out float sy);
                    var g = Vector3.zero; g[a] = sa; g[u] = gx; g[w] = gy;
                    var dir = g.normalized;
                    var sign = Vector3.zero; sign[a] = sa; sign[u] = sx; sign[w] = sy;
                    verts.Add(Vector3.Scale(sign, C) + Vector3.Scale(R, dir));
                    normals.Add(new Vector3(dir.x / R.x, dir.y / R.y, dir.z / R.z).normalized);
                    uvs.Add(new Vector2((gx + 1f) * 0.5f, (gy + 1f) * 0.5f));
                }
                for (int iy = 0; iy < n - 1; iy++)
                for (int ix = 0; ix < n - 1; ix++)
                {
                    int i0 = start + iy * n + ix, i1 = i0 + 1, i2 = i0 + n, i3 = i2 + 1;
                    // Winding so the outward side is front-facing for every face.
                    // cross(u, w) = +a, so (i0, i1, i2) faces +a.
                    if (sa > 0) { tris.Add(i0); tris.Add(i1); tris.Add(i2); tris.Add(i1); tris.Add(i3); tris.Add(i2); }
                    else        { tris.Add(i0); tris.Add(i2); tris.Add(i1); tris.Add(i1); tris.Add(i2); tris.Add(i3); }
                }
            }
            var mesh = new Mesh { indexFormat = IndexFormat.UInt32 };
            mesh.SetVertices(verts); mesh.SetNormals(normals); mesh.SetUVs(0, uvs); mesh.SetTriangles(tris, 0);
            return mesh;
        }

        /// <summary>Grid coordinate in [-1, 1] with the middle doubled at 0; side says which core half it belongs to.</summary>
        static float Coord(int i, int k, out float side)
        {
            if (i <= k) { side = -1f; return -1f + (float)i / k; }
            side = 1f; return (float)(i - k - 1) / k;
        }

        static float Sq(float x) => x * x;
        static Vector3 Abs(Vector3 v) => new Vector3(Mathf.Abs(v.x), Mathf.Abs(v.y), Mathf.Abs(v.z));
    }
}
