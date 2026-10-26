package com.example.phonelidar

import android.media.Image
import com.google.ar.core.CameraIntrinsics
import com.google.ar.core.Frame
import com.google.ar.core.exceptions.NotYetAvailableException
import java.nio.ByteOrder
import kotlin.math.abs

/**
 * Copies ARCore raw depth (plus confidence and camera color) out of a [Frame] into a
 * [DepthFrame]. Runs on the GL thread, so it only does cheap per-pixel work.
 */
class DepthCapture {

    var minConfidence = 100
    var minDepthMeters = 0.25f
    var maxDepthMeters = 4.5f

    private var lastDepthTimestamp = 0L

    /** Returns a new frame, or null if no new depth image is available. */
    fun capture(frame: Frame): DepthFrame? {
        val depth = try {
            frame.acquireRawDepthImage16Bits()
        } catch (e: NotYetAvailableException) {
            return null
        }
        depth.use {
            if (depth.timestamp == lastDepthTimestamp) return null
            lastDepthTimestamp = depth.timestamp
            val confidence = try {
                frame.acquireRawDepthConfidenceImage()
            } catch (e: NotYetAvailableException) {
                return null
            }
            confidence.use {
                val color = try {
                    frame.acquireCameraImage()
                } catch (e: NotYetAvailableException) {
                    null
                }
                try {
                    return convert(frame, depth, confidence, color)
                } finally {
                    color?.close()
                }
            }
        }
    }

    private fun convert(frame: Frame, depth: Image, confidence: Image, color: Image?): DepthFrame {
        val camera = frame.camera
        val w = depth.width
        val h = depth.height

        // Depth is aligned with either the GPU texture or the CPU image depending on the
        // device; use whichever intrinsics share the depth image's aspect ratio.
        val intr = pickIntrinsics(w, h, camera.textureIntrinsics, camera.imageIntrinsics)
        val dims = intr.imageDimensions
        val focal = intr.focalLength
        val pp = intr.principalPoint
        val fx = focal[0] * w / dims[0]
        val fy = focal[1] * h / dims[1]
        val cx = pp[0] * w / dims[0]
        val cy = pp[1] * h / dims[1]

        val pose = FloatArray(16)
        camera.pose.toMatrix(pose, 0)

        val depthPlane = depth.planes[0]
        val depthBuf = depthPlane.buffer.order(ByteOrder.LITTLE_ENDIAN)
        val confPlane = confidence.planes[0]
        val confBuf = confPlane.buffer
        val sampler = color?.let { ColorSampler(it, camera.imageIntrinsics) }

        val meters = FloatArray(w * h)
        val colors = IntArray(w * h)
        for (v in 0 until h) {
            for (u in 0 until w) {
                val conf = confBuf.get(v * confPlane.rowStride + u * confPlane.pixelStride).toInt() and 0xFF
                if (conf < minConfidence) continue
                val mm = depthBuf.getShort(v * depthPlane.rowStride + u * depthPlane.pixelStride)
                    .toInt() and 0xFFFF
                val d = mm * 0.001f
                if (d < minDepthMeters || d > maxDepthMeters) continue
                val i = v * w + u
                meters[i] = d
                colors[i] = sampler?.sample((u - cx) * d / fx, (v - cy) * d / fy, d) ?: DEFAULT_COLOR
            }
        }
        return DepthFrame(w, h, meters, colors, fx, fy, cx, cy, pose)
    }

    private fun pickIntrinsics(w: Int, h: Int, a: CameraIntrinsics, b: CameraIntrinsics): CameraIntrinsics {
        val aspect = w.toFloat() / h
        val ad = a.imageDimensions
        val bd = b.imageDimensions
        val errA = abs(ad[0].toFloat() / ad[1] - aspect)
        val errB = abs(bd[0].toFloat() / bd[1] - aspect)
        return if (errA <= errB) a else b
    }

    /** Samples RGB from a YUV_420_888 camera image by projecting a sensor-frame point into it. */
    private class ColorSampler(image: Image, intrinsics: CameraIntrinsics) {
        private val width = image.width
        private val height = image.height
        private val yPlane = image.planes[0]
        private val uPlane = image.planes[1]
        private val vPlane = image.planes[2]
        private val yBuf = yPlane.buffer
        private val uBuf = uPlane.buffer
        private val vBuf = vPlane.buffer
        private val fx: Float
        private val fy: Float
        private val cx: Float
        private val cy: Float

        init {
            val dims = intrinsics.imageDimensions
            val focal = intrinsics.focalLength
            val pp = intrinsics.principalPoint
            fx = focal[0] * width / dims[0]
            fy = focal[1] * height / dims[1]
            cx = pp[0] * width / dims[0]
            cy = pp[1] * height / dims[1]
        }

        fun sample(xs: Float, ys: Float, z: Float): Int {
            val u = (fx * xs / z + cx).toInt()
            val v = (fy * ys / z + cy).toInt()
            if (u < 0 || v < 0 || u >= width || v >= height) return DEFAULT_COLOR

            val y = yBuf.get(v * yPlane.rowStride + u * yPlane.pixelStride).toInt() and 0xFF
            val uvRow = v / 2
            val uvCol = u / 2
            val cu = (uBuf.get(uvRow * uPlane.rowStride + uvCol * uPlane.pixelStride).toInt() and 0xFF) - 128
            val cv = (vBuf.get(uvRow * vPlane.rowStride + uvCol * vPlane.pixelStride).toInt() and 0xFF) - 128

            val r = clamp(y + 1.402f * cv)
            val g = clamp(y - 0.344136f * cu - 0.714136f * cv)
            val b = clamp(y + 1.772f * cu)
            return (r shl 16) or (g shl 8) or b
        }

        private fun clamp(c: Float): Int = c.toInt().coerceIn(0, 255)
    }

    companion object {
        private const val DEFAULT_COLOR = 0x9A9A9A
    }
}
