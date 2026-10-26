package com.example.phonelidar

import java.nio.ByteBuffer
import java.nio.IntBuffer

/**
 * Triangle mesh of one chunk of the volume. Immutable once built, so it can be read
 * by the GL thread and the exporter at the same time (use absolute reads only).
 *
 * Vertex layout ([STRIDE] bytes): position 3×float, normal 3×float, color 4×ubyte.
 */
class ChunkMesh(
    val key: Long,
    val vertexCount: Int,
    val indexCount: Int,
    val vertices: ByteBuffer,
    val indices: IntBuffer,
    val area: Float,
    val min: FloatArray,
    val max: FloatArray,
    /** Vertices that were snapped onto a detected plane; post-processing keeps them fixed. */
    val snapped: BooleanArray,
) {
    companion object {
        const val STRIDE = 28
        const val NORMAL_OFFSET = 12
        const val COLOR_OFFSET = 24
    }
}
