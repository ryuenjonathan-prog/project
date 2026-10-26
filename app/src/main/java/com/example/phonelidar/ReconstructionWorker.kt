package com.example.phonelidar

import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import android.util.Log
import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Owns the [TsdfVolume] and runs depth fusion and meshing on a background thread so the
 * camera preview keeps its frame rate. Mesh changes are published through [updates].
 *
 * While scanning, the published meshes are the live per-chunk meshes. When scanning stops,
 * [finalizeModel] replaces them with one cleaned-up model (plane snapping, island removal,
 * smoothing, photo textures from [keyframes]); resuming the scan switches back to the
 * live chunks.
 */
class ReconstructionWorker(private val keyframes: KeyframeCollector) {

    /**
     * A replaced chunk mesh; [mesh] == null removes the chunk, [CLEAR] removes everything.
     * [textured] carries the textured final model.
     */
    class MeshUpdate(val key: Long, val mesh: ChunkMesh?, val textured: TexturedMesh? = null)

    private val thread = HandlerThread("reconstruction").apply { start() }
    private val handler = Handler(thread.looper)
    private val volume = TsdfVolume()
    private val meshes = HashMap<Long, ChunkMesh>()
    private val busy = AtomicBoolean(false)
    private var lastMeshTime = 0L
    private var chunkTriangles = 0
    private var chunkArea = 0f
    private var finalMesh: ChunkMesh? = null
    private var finalTextured: TexturedMesh? = null
    private var finalStale = true
    private var showingFinal = false

    val updates = ConcurrentLinkedQueue<MeshUpdate>()

    /** Latest planes detected by ARCore; used to flatten the mesh. */
    @Volatile var planes: PlaneSet = PlaneSet.EMPTY

    @Volatile var triangleCount = 0
        private set
    @Volatile var surfaceArea = 0f
        private set
    @Volatile var isFull = false
        private set

    val isBusy: Boolean get() = busy.get()

    /** Queues a frame for fusion; returns false (frame dropped) if the worker is busy. */
    fun submit(frame: DepthFrame): Boolean {
        if (!busy.compareAndSet(false, true)) return false
        handler.post {
            try {
                if (showingFinal) showLiveChunks()
                volume.integrate(frame)
                finalStale = true
                isFull = volume.isFull
                val now = SystemClock.uptimeMillis()
                if (now - lastMeshTime > MESH_INTERVAL_MS) {
                    remesh(MESH_BUDGET_MS)
                    lastMeshTime = SystemClock.uptimeMillis()
                }
            } finally {
                busy.set(false)
            }
        }
        return true
    }

    /** Builds the cleaned-up model and shows it instead of the live chunks. */
    fun finalizeModel(onDone: () -> Unit) {
        handler.post {
            if (finalStale || finalMesh == null) buildFinal()
            if (!showingFinal) showFinal()
            onDone()
        }
    }

    fun reset() {
        handler.post {
            volume.clear()
            meshes.clear()
            finalMesh = null
            finalTextured = null
            finalStale = true
            showingFinal = false
            chunkTriangles = 0
            chunkArea = 0f
            triangleCount = 0
            surfaceArea = 0f
            isFull = false
            updates.add(MeshUpdate(CLEAR, null))
        }
    }

    /** Hands the finished model to [block] on the worker thread. */
    fun withFinalModel(block: (FinalModel) -> Unit) {
        handler.post {
            if (finalStale || finalMesh == null) buildFinal()
            block(FinalModel(finalMesh?.let { listOf(it) } ?: meshes.values.toList(), finalTextured))
        }
    }

    private fun buildFinal() {
        // Re-mesh everything so all chunks use the latest (most complete) planes.
        for (key in volume.allChunkKeys()) volume.markChunkDirty(key)
        remesh(Long.MAX_VALUE, publish = !showingFinal)
        val mesh = MeshPostProcessor.process(meshes.values.toList(), FINAL, planes)
        finalMesh = mesh
        finalTextured = null
        val photos = keyframes.snapshot()
        if (mesh != null && photos.isNotEmpty()) {
            finalTextured = try {
                TextureMapper.map(mesh, photos, BitmapAtlasCanvas())
            } catch (e: OutOfMemoryError) {
                Log.e(TAG, "Not enough memory for the texture atlas", e)
                null
            } catch (e: Exception) {
                Log.e(TAG, "Texturing failed", e)
                null
            }
        }
        finalStale = false
        if (showingFinal) showFinal()
    }

    private fun showFinal() {
        val mesh = finalMesh ?: return
        showingFinal = true
        updates.add(MeshUpdate(CLEAR, null))
        updates.add(MeshUpdate(FINAL, mesh))
        finalTextured?.let { updates.add(MeshUpdate(FINAL_TEXTURED, null, it)) }
        triangleCount = mesh.indexCount / 3
        surfaceArea = mesh.area
    }

    private fun showLiveChunks() {
        showingFinal = false
        updates.add(MeshUpdate(CLEAR, null))
        for ((key, mesh) in meshes) updates.add(MeshUpdate(key, mesh))
        triangleCount = chunkTriangles
        surfaceArea = chunkArea
    }

    fun quit() {
        thread.quitSafely()
    }

    private fun remesh(budgetMs: Long, publish: Boolean = true) {
        val planes = planes
        val start = SystemClock.uptimeMillis()
        val dirty = volume.takeDirtyChunks()
        for (i in dirty.indices) {
            if (SystemClock.uptimeMillis() - start > budgetMs) {
                // Out of time: keep the rest for the next round.
                for (j in i until dirty.size) volume.markChunkDirty(dirty[j])
                break
            }
            val key = dirty[i]
            val mesh = volume.meshChunk(key, planes)
            val old = if (mesh != null) meshes.put(key, mesh) else meshes.remove(key)
            if (old == null && mesh == null) continue
            if (old != null) {
                chunkTriangles -= old.indexCount / 3
                chunkArea -= old.area
            }
            if (mesh != null) {
                chunkTriangles += mesh.indexCount / 3
                chunkArea += mesh.area
            }
            if (publish) {
                updates.add(MeshUpdate(key, mesh))
                triangleCount = chunkTriangles
                surfaceArea = chunkArea
            }
        }
    }

    companion object {
        const val CLEAR = Long.MIN_VALUE
        const val FINAL = Long.MIN_VALUE + 1
        const val FINAL_TEXTURED = Long.MIN_VALUE + 2
        private const val TAG = "Reconstruction"
        private const val MESH_INTERVAL_MS = 250L
        private const val MESH_BUDGET_MS = 40L
    }
}
