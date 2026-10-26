package com.example.phonelidar

/**
 * One depth observation handed from the GL thread to the reconstruction thread.
 *
 * [depth] is in meters (0 = no data), [color] is packed 0xRRGGBB per depth pixel and
 * [weight] (0..1) says how much each reading can be trusted.
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
    val weight: FloatArray,
) {
    companion object {
        /**
         * Per-pixel trust: readings that hit a surface at a grazing angle are much less
         * accurate, so they are down-weighted (normal estimated from neighbouring pixels).
         * [confidence] (0..1, optional) multiplies in ARCore's own confidence.
         */
        fun computeWeights(
            depth: FloatArray, w: Int, h: Int, fx: Float, fy: Float, cx: Float, cy: Float,
            confidence: FloatArray? = null,
        ): FloatArray {
            val out = FloatArray(w * h)
            for (v in 0 until h) {
                for (u in 0 until w) {
                    val i = v * w + u
                    val d = depth[i]
                    if (d <= 0f) continue
                    val conf = confidence?.get(i) ?: 1f
                    val un = if (u + 1 < w && depth[i + 1] > 0f) i + 1 else if (u > 0 && depth[i - 1] > 0f) i - 1 else -1
                    val vn = if (v + 1 < h && depth[i + w] > 0f) i + w else if (v > 0 && depth[i - w] > 0f) i - w else -1
                    if (un < 0 || vn < 0) {
                        out[i] = 0.5f * conf
                        continue
                    }
                    // 3D points in the sensor frame.
                    val x0 = (u - cx) * d / fx; val y0 = (v - cy) * d / fy
                    val du = depth[un]; val uu = un % w
                    val x1 = (uu - cx) * du / fx - x0; val y1 = (v - cy) * du / fy - y0; val z1 = du - d
                    val dv = depth[vn]; val vv = vn / w
                    val x2 = (u - cx) * dv / fx - x0; val y2 = (vv - cy) * dv / fy - y0; val z2 = dv - d
                    val nx = y1 * z2 - z1 * y2
                    val ny = z1 * x2 - x1 * z2
                    val nz = x1 * y2 - y1 * x2
                    val nl = kotlin.math.sqrt(nx * nx + ny * ny + nz * nz)
                    val rl = kotlin.math.sqrt(x0 * x0 + y0 * y0 + d * d)
                    if (nl < 1e-12f) {
                        out[i] = 0.5f * conf
                        continue
                    }
                    val cos = kotlin.math.abs(nx * x0 + ny * y0 + nz * d) / (nl * rl)
                    out[i] = ((cos - 0.15f) / 0.5f).coerceIn(0f, 1f) * conf
                }
            }
            return out
        }

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
