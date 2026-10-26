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
)
