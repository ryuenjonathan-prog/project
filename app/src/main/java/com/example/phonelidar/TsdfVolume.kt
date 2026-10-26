package com.example.phonelidar

import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.math.abs
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Sparse truncated signed distance field (TSDF) built from depth frames, meshed with
 * Surface Nets.
 *
 * Space is split into blocks of 8³ voxels that are allocated only near observed surfaces.
 * Blocks are grouped into chunks of 4³ blocks; a chunk is the unit of meshing, so only
 * chunks touched by new depth data are re-meshed.
 *
 * Not thread-safe: use from a single thread.
 */
class TsdfVolume(
    val voxelSize: Float = 0.02f,
    private val maxBlocks: Int = 25_000,
) {
    private class Block {
        val tsdf = FloatArray(BLOCK_VOXELS) { 1f }
        val weight = FloatArray(BLOCK_VOXELS)
        val color = IntArray(BLOCK_VOXELS)
    }

    private val blocks = HashMap<Long, Block>()
    private val dirtyChunks = HashSet<Long>()
    private val touched = HashSet<Long>()
    private var frameCount = 0

    val truncation = voxelSize * 4f
    private val blockSize = voxelSize * BLOCK_DIM
    private val maxWeight = 100f
    private val minMeshWeight = 1.5f

    val blockCount: Int get() = blocks.size
    val isFull: Boolean get() = blocks.size >= maxBlocks

    fun clear() {
        blocks.clear()
        dirtyChunks.clear()
    }

    /** Fuses one depth frame into the volume. */
    fun integrate(f: DepthFrame) {
        val m = f.pose
        val tx = m[12]
        val ty = m[13]
        val tz = m[14]

        // 1. Allocate blocks along the truncation band around every observed surface point.
        touched.clear()
        for (v in 0 until f.height step 2) {
            for (u in 0 until f.width step 2) {
                val d = f.depth[v * f.width + u]
                if (d <= 0f) continue
                val rx = (u - f.cx) / f.fx
                val ry = (v - f.cy) / f.fy
                for (s in -1..1) {
                    val dd = d + s * truncation
                    val px = rx * dd
                    val py = -ry * dd
                    val pz = -dd
                    val wx = m[0] * px + m[4] * py + m[8] * pz + tx
                    val wy = m[1] * px + m[5] * py + m[9] * pz + ty
                    val wz = m[2] * px + m[6] * py + m[10] * pz + tz
                    touched.add(
                        pack(
                            floor(wx / blockSize).toInt(),
                            floor(wy / blockSize).toInt(),
                            floor(wz / blockSize).toInt(),
                        ),
                    )
                }
            }
        }

        // 2. Existing blocks that the camera now sees through are free space: integrating
        // them carves away floating noise left by earlier bad measurements.
        // Done every other frame: it visits every block, and carving needs no hurry.
        if (++frameCount % 2 == 0) {
            for ((key, block) in blocks) {
                if (key !in touched && isInFreeSpace(key, f)) {
                    if (integrateBlock(block, key, f)) markBlockDirty(key)
                }
            }
        }

        // 3. Update every voxel of the surface blocks by projecting it into the depth image.
        for (key in touched) {
            var block = blocks[key]
            if (block == null) {
                if (blocks.size >= maxBlocks) continue
                block = Block()
                blocks[key] = block
            }
            if (integrateBlock(block, key, f)) markBlockDirty(key)
        }
    }

    /** True if the whole block lies in front of the observed surface along its view ray. */
    private fun isInFreeSpace(key: Long, f: DepthFrame): Boolean {
        val m = f.pose
        val half = blockSize * 0.5f
        val px = (unpackX(key) * blockSize + half) - m[12]
        val py = (unpackY(key) * blockSize + half) - m[13]
        val pz = (unpackZ(key) * blockSize + half) - m[14]
        val camX = m[0] * px + m[1] * py + m[2] * pz
        val camY = m[4] * px + m[5] * py + m[6] * pz
        val zs = -(m[8] * px + m[9] * py + m[10] * pz)
        if (zs < 0.3f) return false
        val u = (f.fx * camX / zs + f.cx).toInt()
        val v = (f.fy * -camY / zs + f.cy).toInt()
        if (u < 0 || v < 0 || u >= f.width || v >= f.height) return false
        val d = f.depth[v * f.width + u]
        return d > 0f && zs + blockSize < d - truncation
    }

    private fun integrateBlock(block: Block, key: Long, f: DepthFrame): Boolean {
        val m = f.pose
        val tx = m[12]
        val ty = m[13]
        val tz = m[14]
        val bx = unpackX(key) * BLOCK_DIM
        val by = unpackY(key) * BLOCK_DIM
        val bz = unpackZ(key) * BLOCK_DIM
        val w = f.width
        val h = f.height
        val trunc = truncation
        val tsdf = block.tsdf
        val weight = block.weight
        val color = block.color
        var changed = false

        for (z in 0 until BLOCK_DIM) {
            val pz = (bz + z + 0.5f) * voxelSize - tz
            for (y in 0 until BLOCK_DIM) {
                val py = (by + y + 0.5f) * voxelSize - ty
                for (x in 0 until BLOCK_DIM) {
                    val px = (bx + x + 0.5f) * voxelSize - tx
                    // World -> camera (transpose of the rotation part).
                    val camX = m[0] * px + m[1] * py + m[2] * pz
                    val camY = m[4] * px + m[5] * py + m[6] * pz
                    val camZ = m[8] * px + m[9] * py + m[10] * pz
                    val zs = -camZ
                    if (zs < 0.1f) continue
                    val u = (f.fx * camX / zs + f.cx + 0.5f).toInt()
                    val v = (f.fy * -camY / zs + f.cy + 0.5f).toInt()
                    if (u < 0 || v < 0 || u >= w || v >= h) continue
                    val pix = v * w + u
                    val d = f.depth[pix]
                    if (d <= 0f) continue
                    val sdf = d - zs
                    if (sdf < -trunc) continue
                    val t = if (sdf > trunc) 1f else sdf / trunc

                    // Depth error grows with distance squared, so near readings count more.
                    val wObs = (1f / (d * d)).coerceIn(0.2f, 1f)
                    val i = (z * BLOCK_DIM + y) * BLOCK_DIM + x
                    val wOld = weight[i]
                    tsdf[i] = (tsdf[i] * wOld + t * wObs) / (wOld + wObs)
                    weight[i] = min(wOld + wObs, maxWeight)
                    if (sdf < trunc * 0.5f) {
                        color[i] = blendColor(color[i], f.color[pix], wOld / wObs)
                    }
                    changed = true
                }
            }
        }
        return changed
    }

    private fun blendColor(old: Int, new: Int, wOld: Float): Int {
        if (wOld <= 0f) return new
        val a = wOld / (wOld + 1f)
        val b = 1f - a
        val r = (((old shr 16) and 0xFF) * a + ((new shr 16) and 0xFF) * b).toInt()
        val g = (((old shr 8) and 0xFF) * a + ((new shr 8) and 0xFF) * b).toInt()
        val bl = ((old and 0xFF) * a + (new and 0xFF) * b).toInt()
        return (r shl 16) or (g shl 8) or bl
    }

    /**
     * Marks the block's chunk dirty, plus neighbouring chunks whose meshing reads this
     * block's boundary voxels.
     */
    private fun markBlockDirty(key: Long) {
        val bx = unpackX(key)
        val by = unpackY(key)
        val bz = unpackZ(key)
        val cx = bx shr 2
        val cy = by shr 2
        val cz = bz shr 2
        val lx = bx and 3
        val ly = by and 3
        val lz = bz and 3
        for (dz in -1..1) {
            if (!affects(lz, dz)) continue
            for (dy in -1..1) {
                if (!affects(ly, dy)) continue
                for (dx in -1..1) {
                    if (!affects(lx, dx)) continue
                    dirtyChunks.add(pack(cx + dx, cy + dy, cz + dz))
                }
            }
        }
    }

    private fun affects(local: Int, d: Int): Boolean = when (d) {
        0 -> true
        -1 -> local == 0
        else -> local == CHUNK_BLOCKS - 1
    }

    /** Returns and forgets the chunks that need re-meshing. */
    fun takeDirtyChunks(): LongArray {
        val out = dirtyChunks.toLongArray()
        dirtyChunks.clear()
        return out
    }

    fun markChunkDirty(key: Long) {
        dirtyChunks.add(key)
    }

    // ---- Surface Nets meshing ----

    private val gTsdf = FloatArray(G * G * G)
    private val gWeight = FloatArray(G * G * G)
    private val gColor = IntArray(G * G * G)
    private val rTsdf = FloatArray(R * R * R)
    private val rWeight = FloatArray(R * R * R)
    private val rColor = IntArray(R * R * R)
    private val num = FloatArray(R * R * R)
    private val den = FloatArray(R * R * R)
    private val tmp = FloatArray(R * R * R)
    private val cellVertex = IntArray(C * C * C)
    private val pos = FloatList()
    private val nrm = FloatList()
    private val col = IntList()
    private val idx = IntList()

    private val snappedFlags = BoolList()
    private val vp = FloatArray(3)
    private val vn = FloatArray(3)

    /** Every chunk that contains at least one allocated block. */
    fun allChunkKeys(): LongArray {
        val keys = HashSet<Long>()
        for (key in blocks.keys) keys.add(pack(unpackX(key) shr 2, unpackY(key) shr 2, unpackZ(key) shr 2))
        return keys.toLongArray()
    }

    /** Builds the mesh of one chunk, or null if it contains no surface. */
    fun meshChunk(key: Long, planes: PlaneSet = PlaneSet.EMPTY): ChunkMesh? {
        val baseX = unpackX(key) * CHUNK_VOXELS - 1
        val baseY = unpackY(key) * CHUNK_VOXELS - 1
        val baseZ = unpackZ(key) * CHUNK_VOXELS - 1
        if (!gather(baseX - BORDER, baseY - BORDER, baseZ - BORDER)) return null
        smooth()

        pos.clear(); nrm.clear(); col.clear(); idx.clear(); snappedFlags.clear()
        val vs = voxelSize
        val minW = minMeshWeight
        val sx = 1
        val sy = G
        val sz = G * G
        val v = FloatArray(8)

        // One vertex per cell (cube of 8 voxels) that the surface passes through.
        for (k in 0 until C) {
            for (j in 0 until C) {
                for (i in 0 until C) {
                    val ci = (k * C + j) * C + i
                    cellVertex[ci] = -1
                    val l0 = (k * G + j) * G + i
                    var valid = true
                    var mask = 0
                    for (c in 0 until 8) {
                        val l = l0 + (c and 1) * sx + ((c shr 1) and 1) * sy + ((c shr 2) and 1) * sz
                        if (gWeight[l] < minW) { valid = false; break }
                        val value = gTsdf[l]
                        v[c] = value
                        if (value < 0f) mask = mask or (1 shl c)
                    }
                    if (!valid || mask == 0 || mask == 0xFF) continue

                    var ox = 0f; var oy = 0f; var oz = 0f; var n = 0
                    for (e in 0 until 12) {
                        val a = EDGES[e * 2]
                        val b = EDGES[e * 2 + 1]
                        val va = v[a]
                        val vb = v[b]
                        if ((va < 0f) == (vb < 0f)) continue
                        val t = va / (va - vb)
                        ox += (a and 1) + t * ((b and 1) - (a and 1))
                        oy += ((a shr 1) and 1) + t * (((b shr 1) and 1) - ((a shr 1) and 1))
                        oz += ((a shr 2) and 1) + t * (((b shr 2) and 1) - ((a shr 2) and 1))
                        n++
                    }
                    ox /= n; oy /= n; oz /= n

                    // Gradient of the TSDF points away from the surface, i.e. the normal.
                    var gx = (v[1] + v[3] + v[5] + v[7]) - (v[0] + v[2] + v[4] + v[6])
                    var gy = (v[2] + v[3] + v[6] + v[7]) - (v[0] + v[1] + v[4] + v[5])
                    var gz = (v[4] + v[5] + v[6] + v[7]) - (v[0] + v[1] + v[2] + v[3])
                    val len = sqrt(gx * gx + gy * gy + gz * gz)
                    if (len > 1e-6f) { gx /= len; gy /= len; gz /= len } else { gx = 0f; gy = 1f; gz = 0f }

                    // Color: corners nearest the surface count the most.
                    var r = 0f; var g = 0f; var bl = 0f; var wsum = 0f
                    for (c in 0 until 8) {
                        val l = l0 + (c and 1) * sx + ((c shr 1) and 1) * sy + ((c shr 2) and 1) * sz
                        val wc = 1.05f - min(1f, abs(v[c]))
                        val cc = gColor[l]
                        r += ((cc shr 16) and 0xFF) * wc
                        g += ((cc shr 8) and 0xFF) * wc
                        bl += (cc and 0xFF) * wc
                        wsum += wc
                    }

                    vp[0] = (baseX + i + ox + 0.5f) * vs
                    vp[1] = (baseY + j + oy + 0.5f) * vs
                    vp[2] = (baseZ + k + oz + 0.5f) * vs
                    vn[0] = gx; vn[1] = gy; vn[2] = gz
                    snappedFlags.add(!planes.isEmpty && planes.snap(vp, vn))

                    cellVertex[ci] = pos.size / 3
                    pos.add(vp[0]); pos.add(vp[1]); pos.add(vp[2])
                    nrm.add(vn[0]); nrm.add(vn[1]); nrm.add(vn[2])
                    col.add(
                        ((r / wsum).toInt() shl 16) or ((g / wsum).toInt() shl 8) or (bl / wsum).toInt(),
                    )
                }
            }
        }
        if (pos.size == 0) return null

        // One quad per sign-changing voxel edge owned by this chunk.
        var area = 0f
        for (k in 1 until G - 1) {
            for (j in 1 until G - 1) {
                for (i in 1 until G - 1) {
                    val l = (k * G + j) * G + i
                    if (gWeight[l] < minW) continue
                    val v0 = gTsdf[l]
                    for (axis in 0 until 3) {
                        val ln = l + when (axis) { 0 -> sx; 1 -> sy; else -> sz }
                        if (gWeight[ln] < minW) continue
                        val v1 = gTsdf[ln]
                        if ((v0 < 0f) == (v1 < 0f)) continue
                        if (abs(v0) >= 0.99f && abs(v1) >= 0.99f) continue

                        // The four cells sharing this edge, counter-clockwise around +axis.
                        val c0: Int; val c1: Int; val c2: Int; val c3: Int
                        when (axis) {
                            0 -> { // u = y, v = z
                                c0 = cell(i, j - 1, k - 1); c1 = cell(i, j, k - 1)
                                c2 = cell(i, j, k); c3 = cell(i, j - 1, k)
                            }
                            1 -> { // u = z, v = x
                                c0 = cell(i - 1, j, k - 1); c1 = cell(i - 1, j, k)
                                c2 = cell(i, j, k); c3 = cell(i, j, k - 1)
                            }
                            else -> { // u = x, v = y
                                c0 = cell(i - 1, j - 1, k); c1 = cell(i, j - 1, k)
                                c2 = cell(i, j, k); c3 = cell(i - 1, j, k)
                            }
                        }
                        if (c0 < 0 || c1 < 0 || c2 < 0 || c3 < 0) continue
                        if (v0 < 0f) {
                            idx.add(c0); idx.add(c1); idx.add(c2)
                            idx.add(c0); idx.add(c2); idx.add(c3)
                        } else {
                            idx.add(c0); idx.add(c2); idx.add(c1)
                            idx.add(c0); idx.add(c3); idx.add(c2)
                        }
                        area += triangleArea(c0, c1, c2) + triangleArea(c0, c2, c3)
                    }
                }
            }
        }
        if (idx.size == 0) return null
        return buildMesh(key, area)
    }

    private fun cell(i: Int, j: Int, k: Int): Int = cellVertex[(k * C + j) * C + i]

    private fun triangleArea(a: Int, b: Int, c: Int): Float {
        val p = pos.data
        val ax = p[b * 3] - p[a * 3]; val ay = p[b * 3 + 1] - p[a * 3 + 1]; val az = p[b * 3 + 2] - p[a * 3 + 2]
        val bx = p[c * 3] - p[a * 3]; val by = p[c * 3 + 1] - p[a * 3 + 1]; val bz = p[c * 3 + 2] - p[a * 3 + 2]
        val cx = ay * bz - az * by
        val cy = az * bx - ax * bz
        val cz = ax * by - ay * bx
        return 0.5f * sqrt(cx * cx + cy * cy + cz * cz)
    }

    /** Copies the voxels of the chunk plus a border into the raw grid. */
    private fun gather(baseX: Int, baseY: Int, baseZ: Int): Boolean {
        rWeight.fill(0f)
        var any = false
        for (bz in (baseZ shr 3)..((baseZ + R - 1) shr 3)) {
            for (by in (baseY shr 3)..((baseY + R - 1) shr 3)) {
                for (bx in (baseX shr 3)..((baseX + R - 1) shr 3)) {
                    val block = blocks[pack(bx, by, bz)] ?: continue
                    any = true
                    val x0 = max(baseX, bx * BLOCK_DIM); val x1 = min(baseX + R, bx * BLOCK_DIM + BLOCK_DIM)
                    val y0 = max(baseY, by * BLOCK_DIM); val y1 = min(baseY + R, by * BLOCK_DIM + BLOCK_DIM)
                    val z0 = max(baseZ, bz * BLOCK_DIM); val z1 = min(baseZ + R, bz * BLOCK_DIM + BLOCK_DIM)
                    for (gz in z0 until z1) {
                        for (gy in y0 until y1) {
                            var bi = ((gz - bz * BLOCK_DIM) * BLOCK_DIM + (gy - by * BLOCK_DIM)) * BLOCK_DIM +
                                (x0 - bx * BLOCK_DIM)
                            var li = ((gz - baseZ) * R + (gy - baseY)) * R + (x0 - baseX)
                            for (gx in x0 until x1) {
                                rTsdf[li] = block.tsdf[bi]
                                rWeight[li] = block.weight[bi]
                                rColor[li] = block.color[bi]
                                bi++
                                li++
                            }
                        }
                    }
                }
            }
        }
        return any
    }

    /**
     * Weighted Gaussian-like blur ([1,4,6,4,1] per axis) of the raw grid into the meshing
     * grid. Averaging only observed voxels smooths away depth noise without growing the
     * surface into unobserved space.
     */
    private fun smooth() {
        val n = R * R * R
        for (i in 0 until n) {
            val w = rWeight[i]
            num[i] = rTsdf[i] * w
            den[i] = w
        }
        blurAxis(num, 1)
        blurAxis(num, R)
        blurAxis(num, R * R)
        blurAxis(den, 1)
        blurAxis(den, R)
        blurAxis(den, R * R)
        for (k in 0 until G) {
            for (j in 0 until G) {
                var gi = (k * G + j) * G
                var ri = ((k + BORDER) * R + (j + BORDER)) * R + BORDER
                for (i in 0 until G) {
                    val w = rWeight[ri]
                    gWeight[gi] = w
                    gTsdf[gi] = if (den[ri] > 0f) num[ri] / den[ri] else rTsdf[ri]
                    gColor[gi] = rColor[ri]
                    gi++
                    ri++
                }
            }
        }
    }

    private fun blurAxis(a: FloatArray, stride: Int) {
        val n = R * R * R
        val lo = 2 * stride
        val hi = n - 2 * stride
        for (i in lo until hi) {
            tmp[i] = (a[i - 2 * stride] + 4f * a[i - stride] + 6f * a[i] +
                4f * a[i + stride] + a[i + 2 * stride]) * (1f / 16f)
        }
        // Values that wrap across rows are only ever read inside the cropped interior,
        // where every tap is a true neighbour.
        System.arraycopy(tmp, lo, a, lo, hi - lo)
    }

    private fun buildMesh(key: Long, area: Float): ChunkMesh {
        val vertexCount = pos.size / 3
        val vb = ByteBuffer.allocateDirect(vertexCount * ChunkMesh.STRIDE).order(ByteOrder.nativeOrder())
        val min = floatArrayOf(Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE)
        val max = floatArrayOf(-Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE)
        for (i in 0 until vertexCount) {
            for (a in 0 until 3) {
                val p = pos.data[i * 3 + a]
                vb.putFloat(p)
                if (p < min[a]) min[a] = p
                if (p > max[a]) max[a] = p
            }
            vb.putFloat(nrm.data[i * 3]); vb.putFloat(nrm.data[i * 3 + 1]); vb.putFloat(nrm.data[i * 3 + 2])
            val c = col.data[i]
            vb.put(((c shr 16) and 0xFF).toByte())
            vb.put(((c shr 8) and 0xFF).toByte())
            vb.put((c and 0xFF).toByte())
            vb.put(0xFF.toByte())
        }
        vb.position(0)
        val ib = ByteBuffer.allocateDirect(idx.size * 4).order(ByteOrder.nativeOrder()).asIntBuffer()
        ib.put(idx.data, 0, idx.size)
        ib.position(0)
        return ChunkMesh(key, vertexCount, idx.size, vb, ib, area, min, max, snappedFlags.toArray())
    }

    private class FloatList {
        var data = FloatArray(4096)
        var size = 0
        fun clear() { size = 0 }
        fun add(value: Float) {
            if (size == data.size) data = data.copyOf(size * 2)
            data[size++] = value
        }
    }

    private class BoolList {
        var data = BooleanArray(4096)
        var size = 0
        fun clear() { size = 0 }
        fun add(value: Boolean) {
            if (size == data.size) data = data.copyOf(size * 2)
            data[size++] = value
        }
        fun toArray(): BooleanArray = data.copyOf(size)
    }

    private class IntList {
        var data = IntArray(4096)
        var size = 0
        fun clear() { size = 0 }
        fun add(value: Int) {
            if (size == data.size) data = data.copyOf(size * 2)
            data[size++] = value
        }
    }

    companion object {
        const val BLOCK_DIM = 8
        const val BLOCK_VOXELS = BLOCK_DIM * BLOCK_DIM * BLOCK_DIM
        const val CHUNK_BLOCKS = 4
        const val CHUNK_VOXELS = BLOCK_DIM * CHUNK_BLOCKS

        // Local meshing grid: the chunk's voxels plus one voxel on every side.
        private const val G = CHUNK_VOXELS + 2
        private const val C = G - 1

        // Raw grid gathered for smoothing: meshing grid plus the blur radius on every side.
        private const val BORDER = 2
        private const val R = G + 2 * BORDER

        // Cube corners are numbered c = x + 2y + 4z; these are the 12 edges.
        private val EDGES = intArrayOf(
            0, 1, 2, 3, 4, 5, 6, 7,
            0, 2, 1, 3, 4, 6, 5, 7,
            0, 4, 1, 5, 2, 6, 3, 7,
        )

        private const val MASK21 = 0x1FFFFFL

        fun pack(x: Int, y: Int, z: Int): Long =
            ((x.toLong() and MASK21) shl 42) or ((y.toLong() and MASK21) shl 21) or (z.toLong() and MASK21)

        private fun signExtend(v: Long): Int = ((v shl 43) shr 43).toInt()
        fun unpackX(key: Long): Int = signExtend(key ushr 42)
        fun unpackY(key: Long): Int = signExtend(key ushr 21)
        fun unpackZ(key: Long): Int = signExtend(key)
    }
}
