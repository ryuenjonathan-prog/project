package com.example.phonelidar

import android.content.ContentValues
import android.content.Context
import android.os.Build
import android.os.Environment
import android.provider.MediaStore
import java.io.BufferedOutputStream
import java.io.File
import java.io.FileOutputStream
import java.io.OutputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * Writes points as a binary PLY file (opens in MeshLab, CloudCompare, Blender, etc.)
 * into the public Downloads folder.
 */
object PlyExporter {

    /** Returns a human-readable location of the saved file. */
    fun save(context: Context, points: FloatArray): String {
        val stamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        val name = "scan_$stamp.ply"

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            val values = ContentValues().apply {
                put(MediaStore.Downloads.DISPLAY_NAME, name)
                put(MediaStore.Downloads.MIME_TYPE, "application/octet-stream")
                put(MediaStore.Downloads.RELATIVE_PATH, Environment.DIRECTORY_DOWNLOADS)
            }
            val resolver = context.contentResolver
            val uri = resolver.insert(MediaStore.Downloads.EXTERNAL_CONTENT_URI, values)
                ?: throw IllegalStateException("Не удалось создать файл")
            resolver.openOutputStream(uri)?.use { write(it, points) }
                ?: throw IllegalStateException("Не удалось открыть файл")
            return "Download/$name"
        }

        @Suppress("DEPRECATION")
        val dir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS)
        dir.mkdirs()
        val file = File(dir, name)
        FileOutputStream(file).use { write(it, points) }
        return file.absolutePath
    }

    private fun write(stream: OutputStream, points: FloatArray) {
        val count = points.size / PointCloud.FLOATS_PER_POINT
        val out = BufferedOutputStream(stream, 1 shl 16)
        val header = buildString {
            append("ply\n")
            append("format binary_little_endian 1.0\n")
            append("comment Phone LiDAR (ARCore depth)\n")
            append("element vertex $count\n")
            append("property float x\nproperty float y\nproperty float z\n")
            append("property uchar red\nproperty uchar green\nproperty uchar blue\n")
            append("end_header\n")
        }
        out.write(header.toByteArray(Charsets.US_ASCII))

        val record = ByteBuffer.allocate(15).order(ByteOrder.LITTLE_ENDIAN)
        for (i in 0 until count) {
            val base = i * PointCloud.FLOATS_PER_POINT
            record.clear()
            record.putFloat(points[base])
            record.putFloat(points[base + 1])
            record.putFloat(points[base + 2])
            record.put(toByte(points[base + 3]))
            record.put(toByte(points[base + 4]))
            record.put(toByte(points[base + 5]))
            out.write(record.array(), 0, 15)
        }
        out.flush()
    }

    private fun toByte(c: Float): Byte = (c * 255f).toInt().coerceIn(0, 255).toByte()
}
