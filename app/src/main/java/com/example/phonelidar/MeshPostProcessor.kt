package com.example.phonelidar

import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Turns the per-chunk live meshes into one clean model:
 *  1. welds the duplicated vertices on chunk borders,
 *  2. drops small disconnected islands (floating noise),
 *  3. flattens whole walls / floors / furniture faces: starting where ARCore detected a
 *     plane, the plane is grown across the connected surface and those vertices are
 *     projected onto it,
 *  4. Taubin-smooths the rest (no shrinking), keeping flattened vertices fixed,
 *  5. recomputes normals from the final triangles.
 */
object MeshPostProcessor {

    class Options(
        val minIslandTriangles: Int = 250,
        val preSmoothIterations: Int = 3,
        val smoothIterations: Int = 8,
        val seedDistance: Float = 0.06f,
        val growDistance: Float = 0.12f,
        val seedAlignment: Float = 0.8f,
        val growAlignment: Float = 0.7f,
        val lambda: Float = 0.5f,
        val mu: Float = -0.53f,
    )

    fun process(
        meshes: List<ChunkMesh>,
        key: Long,
        planes: PlaneSet = PlaneSet.EMPTY,
        options: Options = Options(),
    ): ChunkMesh? {
        // ---- 1. Weld ----
        val totalVerts = meshes.sumOf { it.vertexCount }
        val totalIdx = meshes.sumOf { it.indexCount }
        if (totalVerts == 0 || totalIdx == 0) return null
        val px = FloatArray(totalVerts)
        val py = FloatArray(totalVerts)
        val pz = FloatArray(totalVerts)
        val color = IntArray(totalVerts)
        val locked = BooleanArray(totalVerts)
        val remap = IntArray(totalVerts)
        val lookup = HashMap<Long, Int>(totalVerts * 2)
        var vCount = 0
        var base = 0
        for (m in meshes) {
            val vb = m.vertices
            for (i in 0 until m.vertexCount) {
                val o = i * ChunkMesh.STRIDE
                val x = vb.getFloat(o)
                val y = vb.getFloat(o + 4)
                val z = vb.getFloat(o + 8)
                val k = quantize(x, y, z)
                val existing = lookup[k]
                if (existing != null) {
                    remap[base + i] = existing
                    if (m.snapped[i]) locked[existing] = true
                    continue
                }
                lookup[k] = vCount
                remap[base + i] = vCount
                px[vCount] = x; py[vCount] = y; pz[vCount] = z
                val c = ChunkMesh.COLOR_OFFSET + o
                color[vCount] = ((vb.get(c).toInt() and 0xFF) shl 16) or
                    ((vb.get(c + 1).toInt() and 0xFF) shl 8) or (vb.get(c + 2).toInt() and 0xFF)
                locked[vCount] = m.snapped[i]
                vCount++
            }
            base += m.vertexCount
        }

        var tris = IntArray(totalIdx)
        var tCount = 0
        base = 0
        for (m in meshes) {
            var i = 0
            while (i < m.indexCount) {
                val a = remap[base + m.indices.get(i)]
                val b = remap[base + m.indices.get(i + 1)]
                val c = remap[base + m.indices.get(i + 2)]
                if (a != b && b != c && a != c) {
                    tris[tCount * 3] = a; tris[tCount * 3 + 1] = b; tris[tCount * 3 + 2] = c
                    tCount++
                }
                i += 3
            }
            base += m.vertexCount
        }

        // ---- 2. Remove small islands ----
        val parent = IntArray(vCount) { it }
        fun find(x: Int): Int {
            var r = x
            while (parent[r] != r) r = parent[r]
            var c = x
            while (parent[c] != r) { val next = parent[c]; parent[c] = r; c = next }
            return r
        }
        for (t in 0 until tCount) {
            val a = find(tris[t * 3]); val b = find(tris[t * 3 + 1]); val c = find(tris[t * 3 + 2])
            parent[b] = a
            parent[find(c)] = a
        }
        val componentTris = IntArray(vCount)
        for (t in 0 until tCount) componentTris[find(tris[t * 3])]++
        var kept = 0
        for (t in 0 until tCount) {
            if (componentTris[find(tris[t * 3])] < options.minIslandTriangles) continue
            tris[kept * 3] = tris[t * 3]; tris[kept * 3 + 1] = tris[t * 3 + 1]; tris[kept * 3 + 2] = tris[t * 3 + 2]
            kept++
        }
        tCount = kept
        if (tCount == 0) return null

        // Compact vertices still referenced, keeping their original order so the copy
        // can be done in place (a vertex only ever moves to a lower index).
        val used = BooleanArray(vCount)
        for (i in 0 until tCount * 3) used[tris[i]] = true
        val newIndex = IntArray(vCount)
        var n = 0
        for (v in 0 until vCount) {
            if (!used[v]) continue
            newIndex[v] = n
            px[n] = px[v]; py[n] = py[v]; pz[n] = pz[v]
            color[n] = color[v]; locked[n] = locked[v]
            n++
        }
        vCount = n
        tris = tris.copyOf(tCount * 3)
        for (i in tris.indices) tris[i] = newIndex[tris[i]]

        // ---- 3. Taubin smoothing over the vertex adjacency graph ----
        val adjStart = IntArray(vCount + 1)
        for (i in 0 until tCount * 3) adjStart[tris[i] + 1] += 2
        for (i in 0 until vCount) adjStart[i + 1] += adjStart[i]
        val adj = IntArray(adjStart[vCount])
        val fill = adjStart.copyOf(vCount)
        for (t in 0 until tCount) {
            val a = tris[t * 3]; val b = tris[t * 3 + 1]; val c = tris[t * 3 + 2]
            adj[fill[a]++] = b; adj[fill[a]++] = c
            adj[fill[b]++] = a; adj[fill[b]++] = c
            adj[fill[c]++] = a; adj[fill[c]++] = b
        }
        val qx = FloatArray(vCount); val qy = FloatArray(vCount); val qz = FloatArray(vCount)

        // ---- 3. Plane flattening ----
        if (!planes.isEmpty) {
            // Normals of a lightly smoothed copy decide which plane a vertex belongs to.
            val free = BooleanArray(vCount)
            val sx = px.copyOf(vCount); val sy = py.copyOf(vCount); val sz = pz.copyOf(vCount)
            repeat(options.preSmoothIterations) {
                laplacianStep(sx, sy, sz, qx, qy, qz, adj, adjStart, free, vCount, options.lambda)
                laplacianStep(sx, sy, sz, qx, qy, qz, adj, adjStart, free, vCount, options.mu)
            }
            val (nx, ny, nz) = vertexNormals(sx, sy, sz, tris, tCount, vCount)
            val owner = IntArray(vCount) { -1 }
            val queue = IntArray(vCount)
            val ordered = planes.planes.withIndex().sortedByDescending { it.value.area }
            for ((pi, plane) in ordered) {
                val pn = plane.normal
                val c = plane.center
                fun dist(v: Int) = (sx[v] - c[0]) * pn[0] + (sy[v] - c[1]) * pn[1] + (sz[v] - c[2]) * pn[2]
                fun align(v: Int) = abs(nx[v] * pn[0] + ny[v] * pn[1] + nz[v] * pn[2])
                var head = 0
                var tail = 0
                for (v in 0 until vCount) {
                    if (owner[v] >= 0 || abs(dist(v)) > options.seedDistance || align(v) < options.seedAlignment) continue
                    val dx = sx[v] - c[0]; val dy = sy[v] - c[1]; val dz = sz[v] - c[2]
                    val lx = dx * plane.axisX[0] + dy * plane.axisX[1] + dz * plane.axisX[2]
                    val lz = dx * plane.axisZ[0] + dy * plane.axisZ[1] + dz * plane.axisZ[2]
                    if (!plane.contains(lx, lz, 0.1f)) continue
                    owner[v] = pi
                    queue[tail++] = v
                }
                while (head < tail) {
                    val v = queue[head++]
                    for (k in adjStart[v] until adjStart[v + 1]) {
                        val u = adj[k]
                        if (owner[u] >= 0 || abs(dist(u)) > options.growDistance ||
                            align(u) < options.growAlignment
                        ) {
                            continue
                        }
                        owner[u] = pi
                        queue[tail++] = u
                    }
                }
            }
            // Fill pockets of noise inside a flattened surface: a vertex whose neighbours
            // mostly belong to one plane joins it, repeated to close holes up to ~30 cm.
            val pending = IntArray(vCount)
            repeat(15) {
                var changed = false
                for (v in 0 until vCount) {
                    pending[v] = -1
                    if (owner[v] >= 0) continue
                    val s = adjStart[v]
                    val e = adjStart[v + 1]
                    if (e == s) continue
                    var candidate = -1
                    var votes = 0
                    for (k in s until e) {
                        val o = owner[adj[k]]
                        if (o < 0) continue
                        if (candidate < 0) candidate = o
                        if (o == candidate) votes++
                    }
                    if (candidate < 0 || votes * 2 < e - s) continue
                    val plane = planes.planes[candidate]
                    val pn = plane.normal
                    val c = plane.center
                    val d = (sx[v] - c[0]) * pn[0] + (sy[v] - c[1]) * pn[1] + (sz[v] - c[2]) * pn[2]
                    if (abs(d) < options.growDistance * 2f) pending[v] = candidate
                }
                for (v in 0 until vCount) {
                    if (pending[v] >= 0) {
                        owner[v] = pending[v]
                        changed = true
                    }
                }
                if (!changed) return@repeat
            }

            for (v in 0 until vCount) {
                val pi = owner[v]
                if (pi < 0) continue
                val plane = planes.planes[pi]
                val pn = plane.normal
                val c = plane.center
                val d = (px[v] - c[0]) * pn[0] + (py[v] - c[1]) * pn[1] + (pz[v] - c[2]) * pn[2]
                px[v] -= pn[0] * d; py[v] -= pn[1] * d; pz[v] -= pn[2] * d
                locked[v] = true
            }
        }

        // ---- 4. Smoothing ----
        repeat(options.smoothIterations) {
            laplacianStep(px, py, pz, qx, qy, qz, adj, adjStart, locked, vCount, options.lambda)
            laplacianStep(px, py, pz, qx, qy, qz, adj, adjStart, locked, vCount, options.mu)
        }

        // ---- 5. Normals, area, bounds ----
        val (nx, ny, nz) = vertexNormals(px, py, pz, tris, tCount, vCount)
        var area = 0f
        for (t in 0 until tCount) {
            val a = tris[t * 3]; val b = tris[t * 3 + 1]; val c = tris[t * 3 + 2]
            val ux = px[b] - px[a]; val uy = py[b] - py[a]; val uz = pz[b] - pz[a]
            val wx = px[c] - px[a]; val wy = py[c] - py[a]; val wz = pz[c] - pz[a]
            val cx = uy * wz - uz * wy
            val cy = uz * wx - ux * wz
            val cz = ux * wy - uy * wx
            area += 0.5f * sqrt(cx * cx + cy * cy + cz * cz)
        }

        val min = floatArrayOf(Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE)
        val max = floatArrayOf(-Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE)
        val vb = ByteBuffer.allocateDirect(vCount * ChunkMesh.STRIDE).order(ByteOrder.nativeOrder())
        for (i in 0 until vCount) {
            vb.putFloat(px[i]); vb.putFloat(py[i]); vb.putFloat(pz[i])
            if (px[i] < min[0]) min[0] = px[i]; if (px[i] > max[0]) max[0] = px[i]
            if (py[i] < min[1]) min[1] = py[i]; if (py[i] > max[1]) max[1] = py[i]
            if (pz[i] < min[2]) min[2] = pz[i]; if (pz[i] > max[2]) max[2] = pz[i]
            vb.putFloat(nx[i]); vb.putFloat(ny[i]); vb.putFloat(nz[i])
            val c = color[i]
            vb.put(((c shr 16) and 0xFF).toByte())
            vb.put(((c shr 8) and 0xFF).toByte())
            vb.put((c and 0xFF).toByte())
            vb.put(0xFF.toByte())
        }
        vb.position(0)
        val ib = ByteBuffer.allocateDirect(tCount * 3 * 4).order(ByteOrder.nativeOrder()).asIntBuffer()
        ib.put(tris, 0, tCount * 3)
        ib.position(0)
        return ChunkMesh(key, vCount, tCount * 3, vb, ib, area, min, max, locked.copyOf(vCount))
    }

    /** Area-weighted unit vertex normals. */
    private fun vertexNormals(
        px: FloatArray, py: FloatArray, pz: FloatArray, tris: IntArray, tCount: Int, vCount: Int,
    ): Triple<FloatArray, FloatArray, FloatArray> {
        val nx = FloatArray(vCount); val ny = FloatArray(vCount); val nz = FloatArray(vCount)
        for (t in 0 until tCount) {
            val a = tris[t * 3]; val b = tris[t * 3 + 1]; val c = tris[t * 3 + 2]
            val ux = px[b] - px[a]; val uy = py[b] - py[a]; val uz = pz[b] - pz[a]
            val wx = px[c] - px[a]; val wy = py[c] - py[a]; val wz = pz[c] - pz[a]
            val cx = uy * wz - uz * wy
            val cy = uz * wx - ux * wz
            val cz = ux * wy - uy * wx
            nx[a] += cx; ny[a] += cy; nz[a] += cz
            nx[b] += cx; ny[b] += cy; nz[b] += cz
            nx[c] += cx; ny[c] += cy; nz[c] += cz
        }
        for (i in 0 until vCount) {
            val len = sqrt(nx[i] * nx[i] + ny[i] * ny[i] + nz[i] * nz[i])
            if (len > 1e-12f) {
                nx[i] /= len; ny[i] /= len; nz[i] /= len
            } else {
                nx[i] = 0f; ny[i] = 1f; nz[i] = 0f
            }
        }
        return Triple(nx, ny, nz)
    }

    private fun laplacianStep(
        px: FloatArray, py: FloatArray, pz: FloatArray,
        qx: FloatArray, qy: FloatArray, qz: FloatArray,
        adj: IntArray, adjStart: IntArray, locked: BooleanArray, count: Int, factor: Float,
    ) {
        for (i in 0 until count) {
            val s = adjStart[i]
            val e = adjStart[i + 1]
            if (locked[i] || e == s) {
                qx[i] = px[i]; qy[i] = py[i]; qz[i] = pz[i]
                continue
            }
            var sx = 0f; var sy = 0f; var sz = 0f
            for (k in s until e) {
                val j = adj[k]
                sx += px[j]; sy += py[j]; sz += pz[j]
            }
            val inv = 1f / (e - s)
            qx[i] = px[i] + factor * (sx * inv - px[i])
            qy[i] = py[i] + factor * (sy * inv - py[i])
            qz[i] = pz[i] + factor * (sz * inv - pz[i])
        }
        System.arraycopy(qx, 0, px, 0, count)
        System.arraycopy(qy, 0, py, 0, count)
        System.arraycopy(qz, 0, pz, 0, count)
    }

    /** Positions on chunk borders are bit-identical; 0.1 mm buckets merge them. */
    private fun quantize(x: Float, y: Float, z: Float): Long {
        val mask = 0x1FFFFFL
        val ix = Math.round(x * 10000.0).toLong() and mask
        val iy = Math.round(y * 10000.0).toLong() and mask
        val iz = Math.round(z * 10000.0).toLong() and mask
        return (ix shl 42) or (iy shl 21) or iz
    }
}
