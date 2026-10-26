package com.example.phonelidar

import android.Manifest
import android.app.Activity
import android.content.pm.PackageManager
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.opengl.Matrix
import android.os.Build
import android.os.Bundle
import android.os.SystemClock
import android.util.Log
import android.view.MotionEvent
import android.view.ScaleGestureDetector
import android.view.WindowManager
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import com.google.ar.core.ArCoreApk
import com.google.ar.core.Config
import com.google.ar.core.Session
import com.google.ar.core.TrackingFailureReason
import com.google.ar.core.TrackingState
import com.google.ar.core.exceptions.CameraNotAvailableException
import com.google.ar.core.exceptions.UnavailableException
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10
import kotlin.math.cos
import kotlin.math.sin

class MainActivity : Activity(), GLSurfaceView.Renderer {

    private lateinit var surfaceView: GLSurfaceView
    private lateinit var statusView: TextView
    private lateinit var scanButton: Button
    private lateinit var viewButton: Button

    private var session: Session? = null
    private var installRequested = false

    private val background = BackgroundRenderer()
    private val cloudRenderer = PointCloudRenderer()
    private val cloud = PointCloud(capacity = 1_500_000, voxelSize = 0.015f)
    private val depthProcessor = DepthProcessor()

    // GL-thread state.
    private var viewportWidth = 1
    private var viewportHeight = 1
    private var viewportChanged = false
    private var cameraTextureSet = false
    private val view = FloatArray(16)
    private val proj = FloatArray(16)
    private val mvp = FloatArray(16)
    private val target = FloatArray(3)
    private var lastStatusTime = 0L

    // Shared between UI and GL threads.
    @Volatile private var scanning = true
    @Volatile private var orbitView = false
    @Volatile private var clearRequested = false
    @Volatile private var orbitYaw = 0f
    @Volatile private var orbitPitch = 0.5f
    @Volatile private var orbitDistance = 3f

    private var lastTouchX = 0f
    private var lastTouchY = 0f
    private lateinit var scaleDetector: ScaleGestureDetector

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        statusView = findViewById(R.id.status)
        scanButton = findViewById(R.id.btn_scan)
        viewButton = findViewById(R.id.btn_view)
        surfaceView = findViewById(R.id.surface)

        surfaceView.preserveEGLContextOnPause = true
        surfaceView.setEGLContextClientVersion(2)
        surfaceView.setEGLConfigChooser(8, 8, 8, 8, 16, 0)
        surfaceView.setRenderer(this)
        surfaceView.renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY

        scaleDetector = ScaleGestureDetector(
            this,
            object : ScaleGestureDetector.SimpleOnScaleGestureListener() {
                override fun onScale(detector: ScaleGestureDetector): Boolean {
                    orbitDistance = (orbitDistance / detector.scaleFactor).coerceIn(0.3f, 50f)
                    return true
                }
            },
        )
        surfaceView.setOnTouchListener { _, event -> onSurfaceTouch(event) }

        scanButton.setOnClickListener {
            scanning = !scanning
            updateButtons()
        }
        viewButton.setOnClickListener {
            orbitView = !orbitView
            updateButtons()
        }
        findViewById<Button>(R.id.btn_clear).setOnClickListener { clearRequested = true }
        findViewById<Button>(R.id.btn_save).setOnClickListener { savePly() }
        updateButtons()
    }

    private fun updateButtons() {
        scanButton.text = if (scanning) "Пауза" else "Скан"
        viewButton.text = if (orbitView) "Камера" else "3D вид"
    }

    private fun onSurfaceTouch(event: MotionEvent): Boolean {
        if (!orbitView) return true
        scaleDetector.onTouchEvent(event)
        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN -> {
                lastTouchX = event.x
                lastTouchY = event.y
            }
            MotionEvent.ACTION_MOVE -> {
                if (event.pointerCount == 1 && !scaleDetector.isInProgress) {
                    val dx = event.x - lastTouchX
                    val dy = event.y - lastTouchY
                    orbitYaw -= dx * 0.008f
                    orbitPitch = (orbitPitch + dy * 0.008f).coerceIn(-1.5f, 1.5f)
                }
                lastTouchX = event.x
                lastTouchY = event.y
            }
            MotionEvent.ACTION_POINTER_UP -> {
                // Avoid a jump when one finger of a pinch is lifted.
                val remaining = if (event.actionIndex == 0) 1 else 0
                lastTouchX = event.getX(remaining)
                lastTouchY = event.getY(remaining)
            }
        }
        return true
    }

    override fun onResume() {
        super.onResume()

        val needed = mutableListOf(Manifest.permission.CAMERA)
        if (Build.VERSION.SDK_INT <= Build.VERSION_CODES.P) {
            needed += Manifest.permission.WRITE_EXTERNAL_STORAGE
        }
        val missing = needed.filter { checkSelfPermission(it) != PackageManager.PERMISSION_GRANTED }
        if (missing.isNotEmpty()) {
            requestPermissions(missing.toTypedArray(), PERMISSION_REQUEST)
            return
        }

        if (session == null) {
            try {
                when (ArCoreApk.getInstance().requestInstall(this, !installRequested)) {
                    ArCoreApk.InstallStatus.INSTALL_REQUESTED -> {
                        installRequested = true
                        return
                    }
                    ArCoreApk.InstallStatus.INSTALLED -> Unit
                    else -> Unit
                }
                session = createSession()
            } catch (e: UnavailableException) {
                showError("ARCore недоступен: ${e.javaClass.simpleName}")
                return
            } catch (e: Exception) {
                showError("Ошибка запуска ARCore: ${e.message}")
                return
            }
        }

        val session = session ?: return
        try {
            session.resume()
        } catch (e: CameraNotAvailableException) {
            showError("Камера недоступна. Перезапустите приложение.")
            session.close()
            this.session = null
            return
        }
        surfaceView.onResume()
    }

    private fun createSession(): Session? {
        val s = Session(this)
        val config = Config(s)
        val depthMode = when {
            s.isDepthModeSupported(Config.DepthMode.RAW_DEPTH_ONLY) -> Config.DepthMode.RAW_DEPTH_ONLY
            s.isDepthModeSupported(Config.DepthMode.AUTOMATIC) -> Config.DepthMode.AUTOMATIC
            else -> {
                s.close()
                showError("Это устройство не поддерживает ARCore Depth API")
                return null
            }
        }
        config.setDepthMode(depthMode)
        config.setFocusMode(Config.FocusMode.AUTO)
        config.setPlaneFindingMode(Config.PlaneFindingMode.DISABLED)
        config.setLightEstimationMode(Config.LightEstimationMode.DISABLED)
        config.setUpdateMode(Config.UpdateMode.LATEST_CAMERA_IMAGE)
        s.configure(config)
        return s
    }

    override fun onPause() {
        super.onPause()
        if (session != null) {
            surfaceView.onPause()
            session?.pause()
        }
    }

    override fun onDestroy() {
        session?.close()
        session = null
        super.onDestroy()
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray,
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (grantResults.any { it != PackageManager.PERMISSION_GRANTED }) {
            Toast.makeText(this, "Без доступа к камере сканирование невозможно", Toast.LENGTH_LONG)
                .show()
            finish()
        }
    }

    private fun showError(message: String) {
        Log.e(TAG, message)
        runOnUiThread {
            statusView.text = message
            Toast.makeText(this, message, Toast.LENGTH_LONG).show()
        }
    }

    private fun savePly() {
        val points = cloud.snapshot()
        if (points.isEmpty()) {
            Toast.makeText(this, "Облако точек пустое", Toast.LENGTH_SHORT).show()
            return
        }
        Toast.makeText(this, "Сохраняю...", Toast.LENGTH_SHORT).show()
        Thread {
            val message = try {
                val count = points.size / PointCloud.FLOATS_PER_POINT
                "Сохранено $count точек: ${PlyExporter.save(this, points)}"
            } catch (e: Exception) {
                Log.e(TAG, "Export failed", e)
                "Ошибка сохранения: ${e.message}"
            }
            runOnUiThread { Toast.makeText(this, message, Toast.LENGTH_LONG).show() }
        }.start()
    }

    // ---- GLSurfaceView.Renderer (GL thread) ----

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        GLES20.glClearColor(0f, 0f, 0f, 1f)
        GLES20.glEnable(GLES20.GL_DEPTH_TEST)
        background.createOnGlThread()
        cloudRenderer.createOnGlThread(cloud)
        cameraTextureSet = false
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        GLES20.glViewport(0, 0, width, height)
        viewportWidth = width
        viewportHeight = height
        viewportChanged = true
    }

    override fun onDrawFrame(gl: GL10?) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)
        val session = session ?: return

        if (!cameraTextureSet) {
            session.setCameraTextureName(background.textureId)
            cameraTextureSet = true
        }
        if (viewportChanged) {
            @Suppress("DEPRECATION")
            val rotation = windowManager.defaultDisplay.rotation
            session.setDisplayGeometry(rotation, viewportWidth, viewportHeight)
            viewportChanged = false
        }

        val frame = try {
            session.update()
        } catch (e: Exception) {
            Log.e(TAG, "session.update failed", e)
            return
        }
        val camera = frame.camera

        if (clearRequested) {
            cloud.clear()
            depthProcessor.reset()
            clearRequested = false
        }

        val tracking = camera.trackingState == TrackingState.TRACKING
        if (tracking && scanning && !cloud.isFull) {
            try {
                depthProcessor.process(frame, cloud)
            } catch (e: Exception) {
                Log.e(TAG, "Depth processing failed", e)
            }
        }

        background.draw(frame, visible = !orbitView)
        if (orbitView) {
            cloud.centroid(target)
            val pitch = orbitPitch
            val yaw = orbitYaw
            val dist = orbitDistance
            val eyeX = target[0] + dist * cos(pitch) * sin(yaw)
            val eyeY = target[1] + dist * sin(pitch)
            val eyeZ = target[2] + dist * cos(pitch) * cos(yaw)
            Matrix.setLookAtM(view, 0, eyeX, eyeY, eyeZ, target[0], target[1], target[2], 0f, 1f, 0f)
            val aspect = viewportWidth.toFloat() / viewportHeight
            Matrix.perspectiveM(proj, 0, 60f, aspect, 0.05f, 200f)
        } else {
            camera.getViewMatrix(view, 0)
            camera.getProjectionMatrix(proj, 0, 0.05f, 200f)
        }
        Matrix.multiplyMM(mvp, 0, proj, 0, view, 0)
        cloudRenderer.draw(cloud, mvp)

        val now = SystemClock.uptimeMillis()
        if (now - lastStatusTime > 250) {
            lastStatusTime = now
            val status = buildStatus(tracking, camera.trackingFailureReason)
            runOnUiThread { statusView.text = status }
        }
    }

    private fun buildStatus(tracking: Boolean, reason: TrackingFailureReason): String {
        val sb = StringBuilder()
        sb.append("Точек: ").append(cloud.size)
        if (cloud.isFull) sb.append(" (лимит)")
        sb.append('\n')
        if (!tracking) {
            sb.append(
                when (reason) {
                    TrackingFailureReason.INSUFFICIENT_LIGHT -> "Слишком темно"
                    TrackingFailureReason.EXCESSIVE_MOTION -> "Двигайте телефон медленнее"
                    TrackingFailureReason.INSUFFICIENT_FEATURES -> "Мало деталей в кадре"
                    TrackingFailureReason.CAMERA_UNAVAILABLE -> "Камера недоступна"
                    else -> "Медленно поводите телефоном для инициализации"
                },
            )
        } else if (!scanning) {
            sb.append("Пауза")
        } else {
            sb.append("Сканирование… плавно двигайте телефон")
        }
        if (orbitView) sb.append("\n3D вид: тяните пальцем — вращение, щипок — масштаб")
        return sb.toString()
    }

    companion object {
        private const val TAG = "PhoneLidar"
        private const val PERMISSION_REQUEST = 1
    }
}
