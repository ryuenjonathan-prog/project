package com.example.phonelidar

import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import kotlin.math.floor

/**
 * Accumulated world-space point cloud. Points are deduplicated on a voxel grid so that
 * scanning the same surface again does not grow the cloud forever.
 *
 * Layout per point: x, y, z, r, g, b (colors in 0..1).
 */
class PointCloud(val capacity: Int, private val voxelSize: Float) {

    val data: FloatBuffer = ByteBuffer
        .allocateDirect(capacity * FLOATS_PER_POINT * 4)
        .order(ByteOrder.nativeOrder())
        .asFloatBuffer()

    @Volatile
    var size = 0
        private set

    // Open-addressing hash set of occupied voxels.
    private val tableMask: Int
    private val keys: LongArray

    private var sumX = 0.0
    private var sumY = 0.0
    private var sumZ = 0.0

    init {
        var tableSize = 1
        while (tableSize < capacity * 2) tableSize = tableSize shl 1
        tableMask = tableSize - 1
        keys = LongArray(tableSize) { EMPTY }
    }

    val isFull: Boolean get() = size >= capacity

    /** Adds a point unless its voxel is already occupied. Returns true if added. */
    @Synchronized
    fun add(x: Float, y: Float, z: Float, r: Float, g: Float, b: Float): Boolean {
        if (size >= capacity) return false
        val key = voxelKey(x, y, z)
        var slot = mix(key) and tableMask
        while (true) {
            val k = keys[slot]
            if (k == key) return false
            if (k == EMPTY) break
            slot = (slot + 1) and tableMask
        }
        keys[slot] = key
        val base = size * FLOATS_PER_POINT
        data.put(base, x)
        data.put(base + 1, y)
        data.put(base + 2, z)
        data.put(base + 3, r)
        data.put(base + 4, g)
        data.put(base + 5, b)
        sumX += x
        sumY += y
        sumZ += z
        size++
        return true
    }

    @Synchronized
    fun clear() {
        keys.fill(EMPTY)
        size = 0
        sumX = 0.0
        sumY = 0.0
        sumZ = 0.0
    }

    @Synchronized
    fun centroid(out: FloatArray) {
        if (size == 0) {
            out[0] = 0f; out[1] = 0f; out[2] = 0f
            return
        }
        out[0] = (sumX / size).toFloat()
        out[1] = (sumY / size).toFloat()
        out[2] = (sumZ / size).toFloat()
    }

    /** Copies the current points so they can be used off the GL thread. */
    @Synchronized
    fun snapshot(): FloatArray {
        val out = FloatArray(size * FLOATS_PER_POINT)
        val dup = data.duplicate()
        dup.position(0)
        dup.get(out)
        return out
    }

    private fun voxelKey(x: Float, y: Float, z: Float): Long {
        val ix = floor(x / voxelSize).toLong() and MASK21
        val iy = floor(y / voxelSize).toLong() and MASK21
        val iz = floor(z / voxelSize).toLong() and MASK21
        return (ix shl 42) or (iy shl 21) or iz
    }

    private fun mix(key: Long): Int {
        var h = key * -0x61c8864680b583ebL
        h = h xor (h ushr 29)
        return h.toInt()
    }

    companion object {
        const val FLOATS_PER_POINT = 6
        private const val EMPTY = Long.MIN_VALUE
        private const val MASK21 = 0x1FFFFFL
    }
}
