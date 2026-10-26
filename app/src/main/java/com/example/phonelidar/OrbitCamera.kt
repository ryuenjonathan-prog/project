package com.example.phonelidar

import android.opengl.Matrix
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin
import kotlin.math.sqrt

/** Turntable camera for inspecting the model: rotate, zoom and pan around a target. */
class OrbitCamera {
    private var yaw = 0.6f
    private var pitch = 0.6f
    private var distance = 3f
    private val target = FloatArray(3)

    @Synchronized
    fun rotate(dxPx: Float, dyPx: Float) {
        yaw -= dxPx * 0.008f
        pitch = (pitch + dyPx * 0.008f).coerceIn(-1.5f, 1.5f)
    }

    @Synchronized
    fun zoom(scaleFactor: Float) {
        distance = (distance / scaleFactor).coerceIn(0.2f, 60f)
    }

    @Synchronized
    fun pan(dxPx: Float, dyPx: Float, viewHeightPx: Int) {
        val k = distance * 1.15f / max(1, viewHeightPx)
        val f = forward()
        // right = forward × worldUp, up = right × forward
        var rx = -f[2]; var rz = f[0]
        val rl = sqrt(rx * rx + rz * rz).coerceAtLeast(1e-4f)
        rx /= rl; rz /= rl
        val ux = -rz * f[1]
        val uy = rz * f[0] - rx * f[2]
        val uz = rx * f[1]
        target[0] += -rx * dxPx * k + ux * dyPx * k
        target[1] += uy * dyPx * k
        target[2] += -rz * dxPx * k + uz * dyPx * k
    }

    /** Centers the view on a bounding box. */
    @Synchronized
    fun frame(min: FloatArray, max: FloatArray) {
        for (a in 0 until 3) target[a] = (min[a] + max[a]) * 0.5f
        val dx = max[0] - min[0]; val dy = max[1] - min[1]; val dz = max[2] - min[2]
        distance = (sqrt(dx * dx + dy * dy + dz * dz) * 0.9f).coerceIn(0.8f, 30f)
        pitch = 0.6f
    }

    @Synchronized
    fun viewMatrix(out: FloatArray, lightDir: FloatArray) {
        val f = forward()
        val ex = target[0] - f[0] * distance
        val ey = target[1] - f[1] * distance
        val ez = target[2] - f[2] * distance
        Matrix.setLookAtM(out, 0, ex, ey, ez, target[0], target[1], target[2], 0f, 1f, 0f)
        lightDir[0] = f[0]; lightDir[1] = f[1]; lightDir[2] = f[2]
    }

    /** Unit vector from the eye towards the target. */
    private fun forward(): FloatArray = floatArrayOf(
        -cos(pitch) * sin(yaw),
        -sin(pitch),
        -cos(pitch) * cos(yaw),
    )
}
