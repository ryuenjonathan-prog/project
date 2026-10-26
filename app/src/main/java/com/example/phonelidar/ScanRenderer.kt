package com.example.phonelidar

import android.opengl.GLES30
import android.opengl.GLSurfaceView
import android.opengl.Matrix
import android.os.SystemClock
import android.util.Log
import com.google.ar.core.Session
import com.google.ar.core.TrackingFailureReason
import com.google.ar.core.TrackingState
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10

/**
 * GL-thread side of the app: drives the ARCore session, draws the camera and the live mesh,
 * and feeds depth frames to the [ReconstructionWorker]. Heavy work never runs here, so the
 * preview stays at the camera frame rate.
 */
class ScanRenderer(
    private val worker: ReconstructionWorker,
    private val orbit: OrbitCamera,
    private val displayRotation: () -> Int,
    private val onStats: (Stats) -> Unit,
) : GLSurfaceView.Renderer {

    class Stats(
        val cameraFps: Int,
        val tracking: Boolean,
        val failure: TrackingFailureReason,
        val triangles: Int,
        val area: Float,
        val full: Boolean,
    )

    @Volatile var session: Session? = null
    @Volatile var scanning = false
    @Volatile var view3d = false
    @Volatile var style = MeshRenderer.Style.NORMALS
    @Volatile var reframeRequested = true
    @Volatile var useFullDepth = true

    private val background = BackgroundRenderer()
    private val meshRenderer = MeshRenderer()
    private val depthCapture = DepthCapture()

    private var viewportWidth = 1
    private var viewportHeight = 1
    private var viewportChanged = false
    private var textureSession: Session? = null

    private val view = FloatArray(16)
    private val proj = FloatArray(16)
    private val mvp = FloatArray(16)
    private val lightDir = FloatArray(3)

    private var lastCameraTimestamp = 0L
    private var cameraFrames = 0
    private var fpsWindowStart = 0L
    private var cameraFps = 0
    private var lastStatsTime = 0L

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        GLES30.glEnable(GLES30.GL_DEPTH_TEST)
        background.createOnGlThread()
        meshRenderer.createOnGlThread()
        textureSession = null
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        GLES30.glViewport(0, 0, width, height)
        viewportWidth = width
        viewportHeight = height
        viewportChanged = true
    }

    override fun onDrawFrame(gl: GL10?) {
        val show3d = view3d
        if (show3d) GLES30.glClearColor(0.07f, 0.08f, 0.10f, 1f) else GLES30.glClearColor(0f, 0f, 0f, 1f)
        GLES30.glClear(GLES30.GL_COLOR_BUFFER_BIT or GLES30.GL_DEPTH_BUFFER_BIT)

        meshRenderer.applyUpdates(worker.updates)
        if (reframeRequested && meshRenderer.hasMesh) {
            orbit.frame(meshRenderer.boundsMin, meshRenderer.boundsMax)
            reframeRequested = false
        }

        val session = session
        if (session == null) {
            if (show3d) drawOrbit()
            return
        }
        if (textureSession !== session) {
            session.setCameraTextureName(background.textureId)
            textureSession = session
            viewportChanged = true
        }
        if (viewportChanged) {
            session.setDisplayGeometry(displayRotation(), viewportWidth, viewportHeight)
            viewportChanged = false
        }

        val frame = try {
            session.update()
        } catch (e: Exception) {
            Log.e(TAG, "session.update failed", e)
            return
        }
        val camera = frame.camera
        countCameraFps(frame.timestamp)

        val tracking = camera.trackingState == TrackingState.TRACKING
        if (tracking && scanning && !worker.isBusy && !worker.isFull) {
            try {
                depthCapture.capture(frame, useFullDepth)?.let { worker.submit(it) }
            } catch (e: Exception) {
                Log.e(TAG, "Depth capture failed", e)
            }
        }

        background.draw(frame, visible = !show3d)
        if (show3d) {
            drawOrbit()
        } else {
            camera.getViewMatrix(view, 0)
            camera.getProjectionMatrix(proj, 0, 0.05f, 100f)
            Matrix.multiplyMM(mvp, 0, proj, 0, view, 0)
            // Camera looks down -Z: its forward vector is the negated third row of the view matrix.
            lightDir[0] = -view[2]; lightDir[1] = -view[6]; lightDir[2] = -view[10]
            meshRenderer.draw(mvp, lightDir, style, OVERLAY_ALPHA)
        }

        val now = SystemClock.uptimeMillis()
        if (now - lastStatsTime > 200) {
            lastStatsTime = now
            onStats(
                Stats(
                    cameraFps, tracking, camera.trackingFailureReason,
                    worker.triangleCount, worker.surfaceArea, worker.isFull,
                ),
            )
        }
    }

    private fun drawOrbit() {
        orbit.viewMatrix(view, lightDir)
        val aspect = viewportWidth.toFloat() / viewportHeight
        Matrix.perspectiveM(proj, 0, 60f, aspect, 0.03f, 200f)
        Matrix.multiplyMM(mvp, 0, proj, 0, view, 0)
        meshRenderer.draw(mvp, lightDir, style, 1f)
    }

    private fun countCameraFps(timestamp: Long) {
        if (timestamp != lastCameraTimestamp) {
            lastCameraTimestamp = timestamp
            cameraFrames++
        }
        val now = SystemClock.uptimeMillis()
        if (fpsWindowStart == 0L) fpsWindowStart = now
        if (now - fpsWindowStart >= 1000) {
            cameraFps = (cameraFrames * 1000 / (now - fpsWindowStart)).toInt()
            cameraFrames = 0
            fpsWindowStart = now
        }
    }

    companion object {
        private const val TAG = "ScanRenderer"
        private const val OVERLAY_ALPHA = 0.6f
    }
}
