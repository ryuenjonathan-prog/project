package com.example.phonelidar

import android.opengl.GLES30
import java.util.Queue

/**
 * Draws the reconstructed mesh. Each chunk lives in its own vertex/index buffer so a
 * chunk update only re-uploads that chunk.
 */
class MeshRenderer {

    enum class Style(val label: String) { COLOR("Цвет"), CLAY("Глина"), NORMALS("Нормали") }

    private class GpuChunk(val vbo: Int, val ibo: Int, val indexCount: Int)

    private val meshes = HashMap<Long, ChunkMesh>()
    private val gpu = HashMap<Long, GpuChunk>()
    private val pendingUpload = HashSet<Long>()

    private var program = 0
    private var positionAttrib = 0
    private var normalAttrib = 0
    private var colorAttrib = 0
    private var mvpUniform = 0
    private var lightUniform = 0
    private var modeUniform = 0
    private var alphaUniform = 0

    val boundsMin = FloatArray(3)
    val boundsMax = FloatArray(3)
    var hasMesh = false
        private set

    fun createOnGlThread() {
        program = GlUtil.createProgram(VERTEX_SHADER, FRAGMENT_SHADER)
        positionAttrib = GLES30.glGetAttribLocation(program, "a_Position")
        normalAttrib = GLES30.glGetAttribLocation(program, "a_Normal")
        colorAttrib = GLES30.glGetAttribLocation(program, "a_Color")
        mvpUniform = GLES30.glGetUniformLocation(program, "u_Mvp")
        lightUniform = GLES30.glGetUniformLocation(program, "u_LightDir")
        modeUniform = GLES30.glGetUniformLocation(program, "u_Mode")
        alphaUniform = GLES30.glGetUniformLocation(program, "u_Alpha")
        // A new GL context has none of the old buffers: upload everything again.
        gpu.clear()
        pendingUpload.addAll(meshes.keys)
    }

    /** Applies mesh changes published by the reconstruction thread. */
    fun applyUpdates(queue: Queue<ReconstructionWorker.MeshUpdate>) {
        var changed = false
        while (true) {
            val update = queue.poll() ?: break
            changed = true
            if (update.key == ReconstructionWorker.CLEAR) {
                for (key in gpu.keys.toList()) deleteGpu(key)
                meshes.clear()
                pendingUpload.clear()
                continue
            }
            val mesh = update.mesh
            if (mesh == null) {
                meshes.remove(update.key)
                pendingUpload.remove(update.key)
                deleteGpu(update.key)
            } else {
                meshes[update.key] = mesh
                pendingUpload.add(update.key)
            }
        }
        for (key in pendingUpload) {
            val mesh = meshes[key] ?: continue
            upload(key, mesh)
        }
        pendingUpload.clear()
        if (changed) updateBounds()
    }

    private fun upload(key: Long, mesh: ChunkMesh) {
        val existing = gpu[key]
        val vbo: Int
        val ibo: Int
        if (existing != null) {
            vbo = existing.vbo
            ibo = existing.ibo
        } else {
            val ids = IntArray(2)
            GLES30.glGenBuffers(2, ids, 0)
            vbo = ids[0]
            ibo = ids[1]
        }
        GLES30.glBindBuffer(GLES30.GL_ARRAY_BUFFER, vbo)
        GLES30.glBufferData(
            GLES30.GL_ARRAY_BUFFER,
            mesh.vertexCount * ChunkMesh.STRIDE,
            mesh.vertices.duplicate().position(0),
            GLES30.GL_STATIC_DRAW,
        )
        GLES30.glBindBuffer(GLES30.GL_ELEMENT_ARRAY_BUFFER, ibo)
        GLES30.glBufferData(
            GLES30.GL_ELEMENT_ARRAY_BUFFER,
            mesh.indexCount * 4,
            mesh.indices.duplicate().position(0),
            GLES30.GL_STATIC_DRAW,
        )
        GLES30.glBindBuffer(GLES30.GL_ARRAY_BUFFER, 0)
        GLES30.glBindBuffer(GLES30.GL_ELEMENT_ARRAY_BUFFER, 0)
        gpu[key] = GpuChunk(vbo, ibo, mesh.indexCount)
    }

    private fun deleteGpu(key: Long) {
        val chunk = gpu.remove(key) ?: return
        GLES30.glDeleteBuffers(2, intArrayOf(chunk.vbo, chunk.ibo), 0)
    }

    private fun updateBounds() {
        hasMesh = meshes.isNotEmpty()
        if (!hasMesh) return
        boundsMin.fill(Float.MAX_VALUE)
        boundsMax.fill(-Float.MAX_VALUE)
        for (mesh in meshes.values) {
            for (a in 0 until 3) {
                if (mesh.min[a] < boundsMin[a]) boundsMin[a] = mesh.min[a]
                if (mesh.max[a] > boundsMax[a]) boundsMax[a] = mesh.max[a]
            }
        }
    }

    fun draw(mvp: FloatArray, lightDir: FloatArray, style: Style, alpha: Float) {
        if (gpu.isEmpty()) return
        GLES30.glUseProgram(program)
        GLES30.glUniformMatrix4fv(mvpUniform, 1, false, mvp, 0)
        GLES30.glUniform3f(lightUniform, lightDir[0], lightDir[1], lightDir[2])
        GLES30.glUniform1f(modeUniform, style.ordinal.toFloat())
        GLES30.glUniform1f(alphaUniform, alpha)

        if (alpha < 1f) {
            GLES30.glEnable(GLES30.GL_BLEND)
            GLES30.glBlendFunc(GLES30.GL_SRC_ALPHA, GLES30.GL_ONE_MINUS_SRC_ALPHA)
        }
        GLES30.glEnableVertexAttribArray(positionAttrib)
        GLES30.glEnableVertexAttribArray(normalAttrib)
        GLES30.glEnableVertexAttribArray(colorAttrib)
        for (chunk in gpu.values) {
            GLES30.glBindBuffer(GLES30.GL_ARRAY_BUFFER, chunk.vbo)
            GLES30.glVertexAttribPointer(positionAttrib, 3, GLES30.GL_FLOAT, false, ChunkMesh.STRIDE, 0)
            GLES30.glVertexAttribPointer(
                normalAttrib, 3, GLES30.GL_FLOAT, false, ChunkMesh.STRIDE, ChunkMesh.NORMAL_OFFSET,
            )
            GLES30.glVertexAttribPointer(
                colorAttrib, 4, GLES30.GL_UNSIGNED_BYTE, true, ChunkMesh.STRIDE, ChunkMesh.COLOR_OFFSET,
            )
            GLES30.glBindBuffer(GLES30.GL_ELEMENT_ARRAY_BUFFER, chunk.ibo)
            GLES30.glDrawElements(GLES30.GL_TRIANGLES, chunk.indexCount, GLES30.GL_UNSIGNED_INT, 0)
        }
        GLES30.glDisableVertexAttribArray(positionAttrib)
        GLES30.glDisableVertexAttribArray(normalAttrib)
        GLES30.glDisableVertexAttribArray(colorAttrib)
        GLES30.glBindBuffer(GLES30.GL_ARRAY_BUFFER, 0)
        GLES30.glBindBuffer(GLES30.GL_ELEMENT_ARRAY_BUFFER, 0)
        if (alpha < 1f) GLES30.glDisable(GLES30.GL_BLEND)
    }

    companion object {
        private val VERTEX_SHADER = """
            uniform mat4 u_Mvp;
            attribute vec3 a_Position;
            attribute vec3 a_Normal;
            attribute vec4 a_Color;
            varying vec3 v_Normal;
            varying vec4 v_Color;
            void main() {
                gl_Position = u_Mvp * vec4(a_Position, 1.0);
                v_Normal = a_Normal;
                v_Color = a_Color;
            }
        """.trimIndent()

        private val FRAGMENT_SHADER = """
            precision mediump float;
            uniform vec3 u_LightDir;
            uniform float u_Mode;
            uniform float u_Alpha;
            varying vec3 v_Normal;
            varying vec4 v_Color;
            void main() {
                vec3 n = normalize(v_Normal);
                float diff = abs(dot(n, u_LightDir));
                vec3 c;
                if (u_Mode < 0.5) {
                    c = v_Color.rgb * (0.6 + 0.4 * diff);
                } else if (u_Mode < 1.5) {
                    c = vec3(0.88, 0.86, 0.82) * (0.22 + 0.78 * diff);
                } else {
                    c = (n * 0.5 + 0.5) * (0.55 + 0.45 * diff);
                }
                gl_FragColor = vec4(c, u_Alpha);
            }
        """.trimIndent()
    }
}
