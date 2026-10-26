package com.example.phonelidar

/**
 * One depth observation handed from the GL thread to the reconstruction thread.
 *
 * [depth] is in meters (0 = no data), [color] is packed 0xRRGGBB per depth pixel.
 * Intrinsics are in depth-image pixels; [pose] is the camera-to-world matrix
 * (column-major, OpenGL camera convention: -Z forward, +Y up).
 */
class DepthFrame(
    val width: Int,
    val height: Int,
    val depth: FloatArray,
    val color: IntArray,
    val fx: Float,
    val fy: Float,
    val cx: Float,
    val cy: Float,
    val pose: FloatArray,
) {
    companion object {
        /**
         * Drops pixels on depth discontinuities. Smoothed depth blends foreground and
         * background there, producing "flying" points that smear object edges.
         */
        fun removeEdges(depth: FloatArray, w: Int, h: Int, relThreshold: Float = 0.08f): FloatArray {
            val out = FloatArray(w * h)
            for (v in 1 until h - 1) {
                for (u in 1 until w - 1) {
                    val i = v * w + u
                    val d = depth[i]
                    if (d <= 0f) continue
                    val limit = d * relThreshold
                    if (jump(d, depth[i - 1], limit) || jump(d, depth[i + 1], limit) ||
                        jump(d, depth[i - w], limit) || jump(d, depth[i + w], limit)
                    ) {
                        continue
                    }
                    out[i] = d
                }
            }
            return out
        }

        private fun jump(d: Float, n: Float, limit: Float): Boolean = n > 0f && kotlin.math.abs(d - n) > limit
    }
}
