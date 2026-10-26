package com.example.phonelidar

import android.Manifest
import android.app.Activity
import android.app.AlertDialog
import android.content.ActivityNotFoundException
import android.content.Intent
import android.content.pm.PackageManager
import android.opengl.GLSurfaceView
import android.os.Build
import android.os.Bundle
import android.os.SystemClock
import android.text.SpannableString
import android.text.Spanned
import android.text.style.ForegroundColorSpan
import android.util.Log
import android.view.GestureDetector
import android.view.MotionEvent
import android.view.ScaleGestureDetector
import android.view.View
import android.view.WindowManager
import android.widget.ImageButton
import android.widget.TextView
import android.widget.Toast
import com.google.ar.core.ArCoreApk
import com.google.ar.core.CameraConfig
import com.google.ar.core.CameraConfigFilter
import com.google.ar.core.Config
import com.google.ar.core.Session
import com.google.ar.core.TrackingFailureReason
import com.google.ar.core.exceptions.CameraNotAvailableException
import com.google.ar.core.exceptions.UnavailableException
import java.text.SimpleDateFormat
import java.util.Date
import java.util.EnumSet
import java.util.Locale

class MainActivity : Activity() {

    private lateinit var surfaceView: GLSurfaceView
    private lateinit var statusPill: TextView
    private lateinit var fpsPill: TextView
    private lateinit var statsPill: TextView
    private lateinit var hintView: TextView
    private lateinit var recordButton: RecordButton
    private lateinit var viewButton: ImageButton
    private lateinit var viewLabel: TextView
    private lateinit var styleLabel: TextView
    private lateinit var progressOverlay: View

    private val worker = ReconstructionWorker()
    private val orbit = OrbitCamera()
    private lateinit var renderer: ScanRenderer

    private var session: Session? = null
    private var installRequested = false
    private var cameraTargetFps = 30

    private var scanning = false
    private var view3d = false
    private var scanMillis = 0L
    private var scanStartedAt = 0L
    private var lastStats: ScanRenderer.Stats? = null
    private var errorText: String? = null

    private lateinit var scaleDetector: ScaleGestureDetector
    private lateinit var gestureDetector: GestureDetector
    private var lastTouchX = 0f
    private var lastTouchY = 0f
    private var lastFocusX = 0f
    private var lastFocusY = 0f

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        surfaceView = findViewById(R.id.surface)
        statusPill = findViewById(R.id.status_pill)
        fpsPill = findViewById(R.id.fps_pill)
        statsPill = findViewById(R.id.stats_pill)
        hintView = findViewById(R.id.hint)
        recordButton = findViewById(R.id.btn_record)
        viewButton = findViewById(R.id.btn_view)
        viewLabel = findViewById(R.id.label_view)
        styleLabel = findViewById(R.id.label_style)
        progressOverlay = findViewById(R.id.progress_overlay)

        renderer = ScanRenderer(worker, orbit, ::displayRotation) { stats ->
            runOnUiThread { onStats(stats) }
        }
        surfaceView.preserveEGLContextOnPause = true
        surfaceView.setEGLContextClientVersion(3)
        surfaceView.setEGLConfigChooser(8, 8, 8, 8, 24, 0)
        surfaceView.setRenderer(renderer)
        surfaceView.renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY

        setupGestures()
        recordButton.setOnClickListener { setScanning(!scanning) }
        viewButton.setOnClickListener { setView3d(!view3d) }
        findViewById<View>(R.id.btn_style).setOnClickListener { cycleStyle() }
        findViewById<View>(R.id.btn_reset).setOnClickListener { confirmReset() }
        findViewById<View>(R.id.btn_export).setOnClickListener { chooseExportFormat() }
        refreshUi()
    }

    @Suppress("DEPRECATION")
    private fun displayRotation(): Int = windowManager.defaultDisplay.rotation

    // ---- Scan state ----

    private fun setScanning(value: Boolean) {
        if (value == scanning) return
        if (value && session == null) {
            toast(errorText ?: "Камера ещё не готова")
            return
        }
        scanning = value
        renderer.scanning = value
        if (value) {
            scanStartedAt = SystemClock.uptimeMillis()
            if (view3d) setView3d(false)
        } else {
            scanMillis += SystemClock.uptimeMillis() - scanStartedAt
            if ((lastStats?.triangles ?: 0) > 0) {
                // Replace the live mesh with the cleaned-up model and show it off.
                beginBusy("Обработка модели…")
                worker.finalizeModel {
                    runOnUiThread {
                        endBusy()
                        if (!scanning) {
                            renderer.reframeRequested = true
                            setView3d(true)
                        }
                    }
                }
            }
        }
        refreshUi()
    }

    private fun setView3d(value: Boolean) {
        view3d = value
        renderer.view3d = value
        if (value) {
            renderer.style = if (renderer.style == MeshRenderer.Style.NORMALS) {
                MeshRenderer.Style.COLOR
            } else {
                renderer.style
            }
            if (scanning) setScanning(false)
        }
        refreshUi()
    }

    private fun cycleStyle() {
        val styles = MeshRenderer.Style.values()
        renderer.style = styles[(renderer.style.ordinal + 1) % styles.size]
        refreshUi()
    }

    private fun confirmReset() {
        if ((lastStats?.triangles ?: 0) == 0) {
            reset()
            return
        }
        AlertDialog.Builder(this)
            .setTitle("Начать заново?")
            .setMessage("Текущий скан будет удалён.")
            .setPositiveButton("Удалить") { _, _ -> reset() }
            .setNegativeButton("Отмена", null)
            .show()
    }

    private fun reset() {
        setScanning(false)
        worker.reset()
        scanMillis = 0L
        renderer.reframeRequested = true
        renderer.style = MeshRenderer.Style.NORMALS
        setView3d(false)
    }

    // ---- UI ----

    private fun onStats(stats: ScanRenderer.Stats) {
        lastStats = stats
        if (stats.full && scanning) {
            setScanning(false)
            toast("Достигнут лимит памяти. Сохраните модель.")
        }
        refreshUi()
    }

    private fun refreshUi() {
        val stats = lastStats
        recordButton.recording = scanning
        viewLabel.text = if (view3d) "Камера" else "3D"
        viewButton.setImageResource(if (view3d) R.drawable.ic_camera else R.drawable.ic_cube)
        styleLabel.text = renderer.style.label

        val elapsed = scanMillis + if (scanning) SystemClock.uptimeMillis() - scanStartedAt else 0L
        val time = String.format(Locale.US, "%02d:%02d", elapsed / 60000, elapsed / 1000 % 60)
        val (dotColor, label) = when {
            session == null -> 0xFFFFB020.toInt() to "Подготовка"
            scanning -> 0xFFFF3B30.toInt() to "Запись  $time"
            (stats?.triangles ?: 0) > 0 -> 0xFF34C759.toInt() to "Пауза  $time"
            else -> 0xFF7CF3FF.toInt() to "Готов"
        }
        val status = SpannableString("●  $label")
        status.setSpan(ForegroundColorSpan(dotColor), 0, 1, Spanned.SPAN_EXCLUSIVE_EXCLUSIVE)
        statusPill.text = status

        val fps = stats?.cameraFps ?: 0
        fpsPill.text = if (fps > 0) "$fps FPS" else "$cameraTargetFps FPS"

        val triangles = stats?.triangles ?: 0
        if (triangles > 0) {
            statsPill.visibility = View.VISIBLE
            statsPill.text = String.format(
                Locale.US, "▲ %s   ◻ %.1f м²", formatCount(triangles), stats?.area ?: 0f,
            )
        } else {
            statsPill.visibility = View.GONE
        }

        val hint = hintText(stats)
        hintView.text = hint
        hintView.visibility = if (hint.isEmpty()) View.GONE else View.VISIBLE
    }

    private fun hintText(stats: ScanRenderer.Stats?): String {
        errorText?.let { return it }
        if (session == null) return "Запуск камеры…"
        if (view3d) {
            return if ((stats?.triangles ?: 0) > 0) {
                "Палец — вращать · щипок — зум · два пальца — сдвиг\nДвойной тап — центрировать"
            } else {
                "Модель пока пустая — отсканируйте комнату"
            }
        }
        if (stats != null && !stats.tracking) {
            return when (stats.failure) {
                TrackingFailureReason.INSUFFICIENT_LIGHT -> "Слишком темно — включите свет"
                TrackingFailureReason.EXCESSIVE_MOTION -> "Двигайте телефон медленнее"
                TrackingFailureReason.INSUFFICIENT_FEATURES -> "Мало деталей — наведите на предметы"
                TrackingFailureReason.CAMERA_UNAVAILABLE -> "Камера занята другим приложением"
                else -> "Плавно поводите телефоном из стороны в сторону"
            }
        }
        if (!scanning) {
            return if ((stats?.triangles ?: 0) > 0) "Нажмите ● чтобы продолжить" else "Наведите камеру на комнату и нажмите ●"
        }
        return if ((stats?.triangles ?: 0) == 0) {
            "Медленно ведите телефон по стенам и полу"
        } else {
            "Обходите комнату, пока всё не покроется сеткой"
        }
    }

    private fun formatCount(n: Int): String = when {
        n >= 1_000_000 -> String.format(Locale.US, "%.1fM", n / 1_000_000f)
        n >= 1_000 -> String.format(Locale.US, "%.0fK", n / 1_000f)
        else -> n.toString()
    }

    private var busyCount = 0

    private fun beginBusy(text: String) {
        busyCount++
        findViewById<TextView>(R.id.progress_text).text = text
        progressOverlay.visibility = View.VISIBLE
    }

    private fun endBusy() {
        busyCount = (busyCount - 1).coerceAtLeast(0)
        if (busyCount == 0) progressOverlay.visibility = View.GONE
    }

    private fun toast(text: String) = Toast.makeText(this, text, Toast.LENGTH_LONG).show()

    // ---- Gestures (3D view) ----

    private fun setupGestures() {
        scaleDetector = ScaleGestureDetector(
            this,
            object : ScaleGestureDetector.SimpleOnScaleGestureListener() {
                override fun onScaleBegin(detector: ScaleGestureDetector): Boolean {
                    lastFocusX = detector.focusX
                    lastFocusY = detector.focusY
                    return true
                }

                override fun onScale(detector: ScaleGestureDetector): Boolean {
                    orbit.zoom(detector.scaleFactor)
                    orbit.pan(detector.focusX - lastFocusX, detector.focusY - lastFocusY, surfaceView.height)
                    lastFocusX = detector.focusX
                    lastFocusY = detector.focusY
                    return true
                }
            },
        )
        gestureDetector = GestureDetector(
            this,
            object : GestureDetector.SimpleOnGestureListener() {
                override fun onDoubleTap(e: MotionEvent): Boolean {
                    renderer.reframeRequested = true
                    return true
                }
            },
        )
        surfaceView.setOnTouchListener { _, event ->
            if (view3d) handleOrbitTouch(event)
            true
        }
    }

    private fun handleOrbitTouch(event: MotionEvent) {
        scaleDetector.onTouchEvent(event)
        gestureDetector.onTouchEvent(event)
        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN -> {
                lastTouchX = event.x
                lastTouchY = event.y
            }
            MotionEvent.ACTION_MOVE -> {
                if (event.pointerCount == 1 && !scaleDetector.isInProgress) {
                    orbit.rotate(event.x - lastTouchX, event.y - lastTouchY)
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
    }

    // ---- Export ----

    private fun chooseExportFormat() {
        if ((lastStats?.triangles ?: 0) == 0) {
            toast("Сначала отсканируйте комнату")
            return
        }
        setScanning(false)
        val formats = MeshFormat.values()
        AlertDialog.Builder(this)
            .setTitle("Скачать 3D-модель")
            .setItems(formats.map { it.title }.toTypedArray()) { _, which -> export(formats[which]) }
            .setNegativeButton("Отмена", null)
            .show()
    }

    private fun export(format: MeshFormat) {
        beginBusy("Сохраняю 3D-модель…")
        val stamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        val name = "room_$stamp.${format.extension}"
        worker.withFinalMesh { meshes ->
            val result: Result<MediaSaver.Saved> = runCatching {
                check(meshes.isNotEmpty()) { "Модель пустая" }
                MediaSaver.save(applicationContext, name) { out -> format.write(out, meshes) }
            }
            runOnUiThread {
                endBusy()
                result.fold(
                    onSuccess = { showSaved(it, format) },
                    onFailure = {
                        Log.e(TAG, "Export failed", it)
                        toast("Ошибка сохранения: ${it.message}")
                    },
                )
            }
        }
    }

    private fun showSaved(saved: MediaSaver.Saved, format: MeshFormat) {
        val builder = AlertDialog.Builder(this)
            .setTitle("Модель сохранена")
            .setMessage("Файл: ${saved.displayPath}")
            .setNegativeButton("Готово", null)
        val uri = saved.uri
        if (uri != null) {
            builder.setPositiveButton("Поделиться") { _, _ ->
                val send = Intent(Intent.ACTION_SEND)
                    .setType(format.mimeType)
                    .putExtra(Intent.EXTRA_STREAM, uri)
                    .addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
                startActivity(Intent.createChooser(send, "Отправить модель"))
            }
            builder.setNeutralButton("Открыть") { _, _ ->
                val view = Intent(Intent.ACTION_VIEW)
                    .setDataAndType(uri, format.mimeType)
                    .addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
                try {
                    startActivity(view)
                } catch (e: ActivityNotFoundException) {
                    toast("Нет приложения для ${format.extension.uppercase()} — используйте «Поделиться»")
                }
            }
        }
        builder.show()
    }

    // ---- Lifecycle / ARCore ----

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
                if (ArCoreApk.getInstance().requestInstall(this, !installRequested) ==
                    ArCoreApk.InstallStatus.INSTALL_REQUESTED
                ) {
                    installRequested = true
                    return
                }
                session = createSession() ?: return
            } catch (e: UnavailableException) {
                showError("ARCore недоступен на этом устройстве (${e.javaClass.simpleName})")
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
            renderer.session = null
            return
        }
        renderer.session = session
        surfaceView.onResume()
        refreshUi()
    }

    private fun createSession(): Session? {
        val s = Session(this)

        // Prefer a 60 FPS camera stream when the device offers one together with depth.
        val default = s.cameraConfig
        val fast = s.getSupportedCameraConfigs(
            CameraConfigFilter(s).setTargetFps(EnumSet.of(CameraConfig.TargetFps.TARGET_FPS_60)),
        )
        if (fast.isNotEmpty()) {
            s.setCameraConfig(fast[0])
            if (pickDepthMode(s) == null) s.setCameraConfig(default)
        }

        val depthMode = pickDepthMode(s)
        if (depthMode == null) {
            s.close()
            showError("Телефон не поддерживает ARCore Depth API")
            return null
        }
        cameraTargetFps = s.cameraConfig.fpsRange.upper

        val config = Config(s)
        config.setDepthMode(depthMode)
        renderer.useFullDepth = depthMode == Config.DepthMode.AUTOMATIC
        config.setFocusMode(Config.FocusMode.AUTO)
        config.setPlaneFindingMode(Config.PlaneFindingMode.HORIZONTAL_AND_VERTICAL)
        config.setLightEstimationMode(Config.LightEstimationMode.DISABLED)
        config.setUpdateMode(Config.UpdateMode.LATEST_CAMERA_IMAGE)
        s.configure(config)
        errorText = null
        return s
    }

    // Full (smoothed) depth needs AUTOMATIC; raw-only is the fallback.
    private fun pickDepthMode(s: Session): Config.DepthMode? = when {
        s.isDepthModeSupported(Config.DepthMode.AUTOMATIC) -> Config.DepthMode.AUTOMATIC
        s.isDepthModeSupported(Config.DepthMode.RAW_DEPTH_ONLY) -> Config.DepthMode.RAW_DEPTH_ONLY
        else -> null
    }

    override fun onPause() {
        super.onPause()
        if (scanning) setScanning(false)
        if (session != null) {
            surfaceView.onPause()
            session?.pause()
        }
    }

    override fun onDestroy() {
        renderer.session = null
        session?.close()
        session = null
        worker.quit()
        super.onDestroy()
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray,
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (grantResults.any { it != PackageManager.PERMISSION_GRANTED }) {
            Toast.makeText(this, "Без доступа к камере сканирование невозможно", Toast.LENGTH_LONG).show()
            finish()
        }
    }

    private fun showError(message: String) {
        Log.e(TAG, message)
        errorText = message
        refreshUi()
    }

    companion object {
        private const val TAG = "PhoneLidar"
        private const val PERMISSION_REQUEST = 1
    }
}
