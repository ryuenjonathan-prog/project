package com.example.phonelidar

import android.content.ContentValues
import android.content.Context
import android.net.Uri
import android.os.Build
import android.os.Environment
import android.provider.MediaStore
import java.io.BufferedOutputStream
import java.io.File
import java.io.FileOutputStream
import java.io.OutputStream

/** Saves files into the public Downloads/PhoneLidar folder. */
object MediaSaver {

    class Saved(val uri: Uri?, val displayPath: String)

    fun save(context: Context, name: String, writer: (OutputStream) -> Unit): Saved {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            val resolver = context.contentResolver
            val values = ContentValues().apply {
                put(MediaStore.Downloads.DISPLAY_NAME, name)
                put(MediaStore.Downloads.MIME_TYPE, "application/octet-stream")
                put(MediaStore.Downloads.RELATIVE_PATH, "${Environment.DIRECTORY_DOWNLOADS}/$FOLDER")
                put(MediaStore.Downloads.IS_PENDING, 1)
            }
            val uri = resolver.insert(MediaStore.Downloads.EXTERNAL_CONTENT_URI, values)
                ?: throw IllegalStateException("Не удалось создать файл")
            try {
                val stream = resolver.openOutputStream(uri)
                    ?: throw IllegalStateException("Не удалось открыть файл")
                BufferedOutputStream(stream, 1 shl 16).use(writer)
                values.clear()
                values.put(MediaStore.Downloads.IS_PENDING, 0)
                resolver.update(uri, values, null, null)
            } catch (e: Exception) {
                resolver.delete(uri, null, null)
                throw e
            }
            return Saved(uri, "Загрузки/$FOLDER/$name")
        }

        @Suppress("DEPRECATION")
        val dir = File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS), FOLDER)
        dir.mkdirs()
        val file = File(dir, name)
        BufferedOutputStream(FileOutputStream(file), 1 shl 16).use(writer)
        return Saved(null, file.absolutePath)
    }

    private const val FOLDER = "PhoneLidar"
}
