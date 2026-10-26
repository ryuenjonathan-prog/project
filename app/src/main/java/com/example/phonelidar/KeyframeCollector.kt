package com.example.phonelidar

import android.graphics.ImageFormat
import android.graphics.Rect
import android.graphics.YuvImage
import android.media.Image
import android.util.Log
import com.google.ar.core.Frame
import com.google.ar.core.exceptions.NotYetAvailableException
import java.io.ByteArrayOutputStream
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicInteger
import kotlin.math.acos
import kotlin.math.sqrt

/**
 * Picks sharp, well-spaced camera photos during the scan for texturing. Also tracks how fast
 * the phone is turning, so blurry frames can be skipped for depth fusion too.
 */
class KeyframeCollector(private val maxKeyframes: Int = 90) {

    private val encoder = Executors.newSingleThreadExecutor()
    private val keyframes = ArrayList<Keyframe>()
    private val pending = AtomicInteger(0)

    private val pose = FloatArray(16)
    private val prevPose = FloatArray(16)
    private var prevTimestamp = 0L
    private var lastKeyPose: FloatArray? = null
    private var lastKeyTime = 0L

    /** Rotation speed of the camera in rad/s (smoothed). */
    @Volatile var angularSpeed = 0f
        private set

    val count: Int get() = synchronized(keyframes) { keyframes.size }

    fun snapshot(): List<Keyframe> = synchronized(keyframes) { ArrayList(keyframes) }

    fun clear() {
        synchronized(keyframes) { keyframes.clear() }
        lastKeyPose = null
    }

    /** Updates motion state; call on the GL thread for every tracked frame. */
    fun updateMotion(frame: Frame) {
        frame.camera.pose.toMatrix(pose, 0)
        val ts = frame.timestamp
        if (prevTimestamp != 0L && ts > prevTimestamp) {
            val dt = (ts - prevTimestamp) * 1e-9f
            val speed = angleBetween(pose, prevPose) / dt
            angularSpeed = angularSpeed * 0.6f + speed * 0.4f
        }
        if (ts != prevTimestamp) {
            System.arraycopy(pose, 0, prevPose, 0, 16)
            prevTimestamp = ts
        }
    }

    /** Captures a keyframe if the phone is steady and has moved enough since the last one. */
    fun maybeCapture(frame: Frame) {
        if (count >= maxKeyframes || pending.get() >= 2 || angularSpeed > MAX_SPEED) return
        val now = frame.timestamp
        val last = lastKeyPose
        if (last != null) {
            if (now - lastKeyTime < MIN_INTERVAL_NS) return
            val dx = pose[12] - last[12]; val dy = pose[13] - last[13]; val dz = pose[14] - last[14]
            val moved = sqrt(dx * dx + dy * dy + dz * dz)
            if (moved < MIN_TRANSLATION && angleBetween(pose, last) < MIN_ROTATION) return
        }
        val image = try {
            frame.acquireCameraImage()
        } catch (e: NotYetAvailableException) {
            return
        }
        val kfPose = pose.copyOf()
        val intr = frame.camera.imageIntrinsics
        val nv21: ByteArray
        val w: Int
        val h: Int
        try {
            w = image.width
            h = image.height
            nv21 = toNv21(image)
        } finally {
            image.close()
        }
        val dims = intr.imageDimensions
        val focal = intr.focalLength
        val pp = intr.principalPoint
        val sx = w.toFloat() / dims[0]
        val sy = h.toFloat() / dims[1]
        lastKeyPose = kfPose
        lastKeyTime = now
        pending.incrementAndGet()
        encoder.execute {
            try {
                val out = ByteArrayOutputStream(w * h / 4)
                YuvImage(nv21, ImageFormat.NV21, w, h, null).compressToJpeg(Rect(0, 0, w, h), 88, out)
                val kf = Keyframe(
                    out.toByteArray(), w, h,
                    focal[0] * sx, focal[1] * sy, pp[0] * sx, pp[1] * sy, kfPose,
                )
                synchronized(keyframes) { keyframes.add(kf) }
            } catch (e: Exception) {
                Log.e(TAG, "Keyframe encoding failed", e)
            } finally {
                pending.decrementAndGet()
            }
        }
    }

    private fun toNv21(image: Image): ByteArray {
        val w = image.width
        val h = image.height
        val out = ByteArray(w * h * 3 / 2)
        val yPlane = image.planes[0]
        val yBuf = yPlane.buffer.duplicate()
        for (row in 0 until h) {
            yBuf.position(row * yPlane.rowStride)
            yBuf.get(out, row * w, w)
        }
        val uPlane = image.planes[1]
        val vPlane = image.planes[2]
        val uBuf = uPlane.buffer
        val vBuf = vPlane.buffer
        var o = w * h
        for (row in 0 until h / 2) {
            val ur = row * uPlane.rowStride
            val vr = row * vPlane.rowStride
            for (col in 0 until w / 2) {
                out[o++] = vBuf.get(vr + col * vPlane.pixelStride)
                out[o++] = uBuf.get(ur + col * uPlane.pixelStride)
            }
        }
        return out
    }

    /** Angle between the viewing directions (camera -Z axes) of two poses. */
    private fun angleBetween(a: FloatArray, b: FloatArray): Float {
        val dot = a[8] * b[8] + a[9] * b[9] + a[10] * b[10]
        return acos(dot.coerceIn(-1f, 1f))
    }

    fun shutdown() {
        encoder.shutdown()
    }

    companion object {
        private const val TAG = "Keyframes"
        private const val MAX_SPEED = 0.6f
        private const val MIN_INTERVAL_NS = 400_000_000L
        private const val MIN_TRANSLATION = 0.25f
        private const val MIN_ROTATION = 0.26f // ~15 degrees
    }
}
