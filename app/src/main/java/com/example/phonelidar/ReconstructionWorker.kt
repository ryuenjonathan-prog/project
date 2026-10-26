package com.example.phonelidar

import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Owns the [TsdfVolume] and runs depth fusion and meshing on a background thread so the
 * camera preview keeps its frame rate. Mesh changes are published through [updates].
 */
class ReconstructionWorker {

    /** A replaced chunk mesh; [mesh] == null removes the chunk, [CLEAR] removes everything. */
    class MeshUpdate(val key: Long, val mesh: ChunkMesh?)

    private val thread = HandlerThread("reconstruction").apply { start() }
    private val handler = Handler(thread.looper)
    private val volume = TsdfVolume()
    private val meshes = HashMap<Long, ChunkMesh>()
    private val busy = AtomicBoolean(false)
    private var lastMeshTime = 0L

    val updates = ConcurrentLinkedQueue<MeshUpdate>()

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
                volume.integrate(frame)
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

    /** Meshes everything that is still pending. */
    fun flush() {
        handler.post { remesh(Long.MAX_VALUE) }
    }

    fun reset() {
        handler.post {
            volume.clear()
            meshes.clear()
            triangleCount = 0
            surfaceArea = 0f
            isFull = false
            updates.add(MeshUpdate(CLEAR, null))
        }
    }

    /** Finishes meshing and hands the complete model to [block] on the worker thread. */
    fun withFinalMesh(block: (List<ChunkMesh>) -> Unit) {
        handler.post {
            remesh(Long.MAX_VALUE)
            block(meshes.values.toList())
        }
    }

    fun quit() {
        thread.quitSafely()
    }

    private fun remesh(budgetMs: Long) {
        val start = SystemClock.uptimeMillis()
        val dirty = volume.takeDirtyChunks()
        for (i in dirty.indices) {
            if (SystemClock.uptimeMillis() - start > budgetMs) {
                // Out of time: keep the rest for the next round.
                for (j in i until dirty.size) volume.markChunkDirty(dirty[j])
                break
            }
            val key = dirty[i]
            val mesh = volume.meshChunk(key)
            val old = if (mesh != null) meshes.put(key, mesh) else meshes.remove(key)
            if (old == null && mesh == null) continue
            if (old != null) {
                triangleCount -= old.indexCount / 3
                surfaceArea -= old.area
            }
            if (mesh != null) {
                triangleCount += mesh.indexCount / 3
                surfaceArea += mesh.area
            }
            updates.add(MeshUpdate(key, mesh))
        }
    }

    companion object {
        const val CLEAR = Long.MIN_VALUE
        private const val MESH_INTERVAL_MS = 250L
        private const val MESH_BUDGET_MS = 40L
    }
}
