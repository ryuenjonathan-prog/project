package com.example.phonelidar

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.Canvas
import android.graphics.Paint
import android.graphics.Rect
import java.io.ByteArrayOutputStream

/** [AtlasCanvas] backed by an Android Bitmap. */
class BitmapAtlasCanvas : AtlasCanvas {
    private var atlas: Bitmap? = null
    private var canvas: Canvas? = null
    private var source: Bitmap? = null
    private var sourceFrame: Keyframe? = null
    private val paint = Paint(Paint.FILTER_BITMAP_FLAG)
    private val src = Rect()
    private val dst = Rect()

    override fun create(size: Int) {
        val bitmap = Bitmap.createBitmap(size, size, Bitmap.Config.ARGB_8888)
        bitmap.eraseColor(0xFF808080.toInt())
        atlas = bitmap
        canvas = Canvas(bitmap)
    }

    override fun fillRect(x: Int, y: Int, w: Int, h: Int, rgb: Int) {
        val fill = Paint().apply { color = 0xFF000000.toInt() or rgb }
        canvas?.drawRect(x.toFloat(), y.toFloat(), (x + w).toFloat(), (y + h).toFloat(), fill)
    }

    override fun drawRegion(kf: Keyframe, sx: Int, sy: Int, sw: Int, sh: Int, dx: Int, dy: Int, dw: Int, dh: Int) {
        if (kf !== sourceFrame) {
            source?.recycle()
            source = BitmapFactory.decodeByteArray(kf.jpeg, 0, kf.jpeg.size)
            sourceFrame = kf
        }
        val bitmap = source ?: return
        src.set(sx, sy, sx + sw, sy + sh)
        dst.set(dx, dy, dx + dw, dy + dh)
        canvas?.drawBitmap(bitmap, src, dst, paint)
    }

    override fun encodeJpeg(quality: Int): ByteArray {
        source?.recycle()
        source = null
        sourceFrame = null
        val bitmap = atlas ?: return ByteArray(0)
        val out = ByteArrayOutputStream(4 shl 20)
        bitmap.compress(Bitmap.CompressFormat.JPEG, quality, out)
        bitmap.recycle()
        atlas = null
        canvas = null
        return out.toByteArray()
    }
}
