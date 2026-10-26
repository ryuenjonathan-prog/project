package com.example.phonelidar

import android.opengl.GLES20

/**
 * Draws a [PointCloud] from a GPU buffer. Points are only ever appended, so each frame
 * uploads just the points added since the previous frame.
 */
class PointCloudRenderer {

    var pointSize = 12f

    private var program = 0
    private var positionAttrib = 0
    private var colorAttrib = 0
    private var mvpUniform = 0
    private var pointSizeUniform = 0
    private var vbo = 0
    private var uploadedPoints = 0

    fun createOnGlThread(cloud: PointCloud) {
        program = GlUtil.createProgram(VERTEX_SHADER, FRAGMENT_SHADER)
        positionAttrib = GLES20.glGetAttribLocation(program, "a_Position")
        colorAttrib = GLES20.glGetAttribLocation(program, "a_Color")
        mvpUniform = GLES20.glGetUniformLocation(program, "u_Mvp")
        pointSizeUniform = GLES20.glGetUniformLocation(program, "u_PointSize")

        val buffers = IntArray(1)
        GLES20.glGenBuffers(1, buffers, 0)
        vbo = buffers[0]
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vbo)
        GLES20.glBufferData(
            GLES20.GL_ARRAY_BUFFER,
            cloud.capacity * STRIDE_BYTES,
            null,
            GLES20.GL_DYNAMIC_DRAW,
        )
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
        uploadedPoints = 0
    }

    fun draw(cloud: PointCloud, mvp: FloatArray) {
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vbo)
        upload(cloud)
        val count = uploadedPoints
        if (count > 0) {
            GLES20.glUseProgram(program)
            GLES20.glUniformMatrix4fv(mvpUniform, 1, false, mvp, 0)
            GLES20.glUniform1f(pointSizeUniform, pointSize)
            GLES20.glVertexAttribPointer(
                positionAttrib, 3, GLES20.GL_FLOAT, false, STRIDE_BYTES, 0,
            )
            GLES20.glVertexAttribPointer(
                colorAttrib, 3, GLES20.GL_FLOAT, false, STRIDE_BYTES, 3 * 4,
            )
            GLES20.glEnableVertexAttribArray(positionAttrib)
            GLES20.glEnableVertexAttribArray(colorAttrib)
            GLES20.glDrawArrays(GLES20.GL_POINTS, 0, count)
            GLES20.glDisableVertexAttribArray(positionAttrib)
            GLES20.glDisableVertexAttribArray(colorAttrib)
        }
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
    }

    private fun upload(cloud: PointCloud) {
        synchronized(cloud) {
            val size = cloud.size
            if (size < uploadedPoints) uploadedPoints = 0 // cloud was cleared
            if (size == uploadedPoints) return
            val data = cloud.data
            data.position(uploadedPoints * PointCloud.FLOATS_PER_POINT)
            GLES20.glBufferSubData(
                GLES20.GL_ARRAY_BUFFER,
                uploadedPoints * STRIDE_BYTES,
                (size - uploadedPoints) * STRIDE_BYTES,
                data,
            )
            data.position(0)
            uploadedPoints = size
        }
    }

    companion object {
        private const val STRIDE_BYTES = PointCloud.FLOATS_PER_POINT * 4

        private val VERTEX_SHADER = """
            uniform mat4 u_Mvp;
            uniform float u_PointSize;
            attribute vec3 a_Position;
            attribute vec3 a_Color;
            varying vec3 v_Color;
            void main() {
                gl_Position = u_Mvp * vec4(a_Position, 1.0);
                gl_PointSize = clamp(u_PointSize / gl_Position.w, 1.0, 16.0);
                v_Color = a_Color;
            }
        """.trimIndent()

        private val FRAGMENT_SHADER = """
            precision mediump float;
            varying vec3 v_Color;
            void main() {
                gl_FragColor = vec4(v_Color, 1.0);
            }
        """.trimIndent()
    }
}
