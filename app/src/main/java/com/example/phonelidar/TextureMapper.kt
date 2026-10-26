package com.example.phonelidar

import kotlin.math.ceil
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/** A sharp camera photo with the pose and intrinsics it was taken with. */
class Keyframe(
    val jpeg: ByteArray,
    val width: Int,
    val height: Int,
    val fx: Float,
    val fy: Float,
    val cx: Float,
    val cy: Float,
    /** Camera-to-world, column-major, OpenGL camera convention (-Z forward, +Y up). */
    val pose: FloatArray,
)

/** Platform image backend used to bake the texture atlas (Bitmap on Android). */
interface AtlasCanvas {
    fun create(size: Int)
    fun fillRect(x: Int, y: Int, w: Int, h: Int, rgb: Int)

    /** Copies a region of a keyframe photo, scaled, into the atlas. Calls come grouped by keyframe. */
    fun drawRegion(kf: Keyframe, sx: Int, sy: Int, sw: Int, sh: Int, dx: Int, dy: Int, dw: Int, dh: Int)
    fun encodeJpeg(quality: Int): ByteArray
}

/** Final model with a texture atlas. UVs have their origin at the top-left of the atlas. */
class TexturedMesh(
    val vertexCount: Int,
    val positions: FloatArray,
    val normals: FloatArray,
    val uvs: FloatArray,
    /** Packed 0xRRGGBB, multiplied with the texture: white where textured. */
    val colors: IntArray,
    val indices: IntArray,
    val atlasJpeg: ByteArray,
    val atlasSize: Int,
    val min: FloatArray,
    val max: FloatArray,
) {
    val indexCount: Int get() = indices.size
}

/**
 * Projective texturing: every triangle takes its pixels from the keyframe that sees it best
 * (frontal, close, not occluded). Neighbouring triangles with the same keyframe form charts;
 * each chart's image region is cut out of its photo and packed into one atlas.
 */
object TextureMapper {

    private const val ZBUFFER_WIDTH = 240
    private const val MIN_COS = 0.25f
    private const val EDGE_MARGIN = 4f
    private const val SOURCE_PADDING = 2
    private const val GUTTER = 2
    private const val FALLBACK_BLOCK = 8

    private class View(kf: Keyframe) {
        val fx = kf.fx; val fy = kf.fy; val cx = kf.cx; val cy = kf.cy
        val w = kf.width; val h = kf.height
        val m = kf.pose
        val tx = m[12]; val ty = m[13]; val tz = m[14]
        val zw = ZBUFFER_WIDTH
        val zh = max(1, ZBUFFER_WIDTH * h / w)
        val zbuf = FloatArray(zw * zh) { Float.MAX_VALUE }

        // Last projection result.
        var u = 0f; var v = 0f; var z = 0f

        fun project(x: Float, y: Float, zz: Float): Boolean {
            val dx = x - tx; val dy = y - ty; val dz = zz - tz
            val camX = m[0] * dx + m[1] * dy + m[2] * dz
            val camY = m[4] * dx + m[5] * dy + m[6] * dz
            z = -(m[8] * dx + m[9] * dy + m[10] * dz)
            if (z < 0.1f) return false
            u = fx * camX / z + cx
            v = fy * -camY / z + cy
            return true
        }

        fun inside(margin: Float) = u >= margin && v >= margin && u < w - margin && v < h - margin

        fun zIndex(): Int {
            val zx = (u * zw / w).toInt().coerceIn(0, zw - 1)
            val zy = (v * zh / h).toInt().coerceIn(0, zh - 1)
            return zy * zw + zx
        }
    }

    fun map(mesh: ChunkMesh, keyframes: List<Keyframe>, canvas: AtlasCanvas, atlasSize: Int = 4096): TexturedMesh? {
        if (keyframes.isEmpty() || mesh.indexCount == 0) return null
        val vCount = mesh.vertexCount
        val tCount = mesh.indexCount / 3
        val px = FloatArray(vCount); val py = FloatArray(vCount); val pz = FloatArray(vCount)
        val nrm = FloatArray(vCount * 3)
        val col = IntArray(vCount)
        for (i in 0 until vCount) {
            val o = i * ChunkMesh.STRIDE
            px[i] = mesh.vertices.getFloat(o)
            py[i] = mesh.vertices.getFloat(o + 4)
            pz[i] = mesh.vertices.getFloat(o + 8)
            for (a in 0 until 3) nrm[i * 3 + a] = mesh.vertices.getFloat(o + ChunkMesh.NORMAL_OFFSET + a * 4)
            val c = o + ChunkMesh.COLOR_OFFSET
            col[i] = ((mesh.vertices.get(c).toInt() and 0xFF) shl 16) or
                ((mesh.vertices.get(c + 1).toInt() and 0xFF) shl 8) or (mesh.vertices.get(c + 2).toInt() and 0xFF)
        }
        val tris = IntArray(tCount * 3) { mesh.indices.get(it) }

        // Face centroids and unit normals.
        val fc = FloatArray(tCount * 3)
        val fn = FloatArray(tCount * 3)
        for (t in 0 until tCount) {
            val a = tris[t * 3]; val b = tris[t * 3 + 1]; val c = tris[t * 3 + 2]
            fc[t * 3] = (px[a] + px[b] + px[c]) / 3f
            fc[t * 3 + 1] = (py[a] + py[b] + py[c]) / 3f
            fc[t * 3 + 2] = (pz[a] + pz[b] + pz[c]) / 3f
            val ux = px[b] - px[a]; val uy = py[b] - py[a]; val uz = pz[b] - pz[a]
            val wx = px[c] - px[a]; val wy = py[c] - py[a]; val wz = pz[c] - pz[a]
            var nx = uy * wz - uz * wy; var ny = uz * wx - ux * wz; var nz = ux * wy - uy * wx
            val len = sqrt(nx * nx + ny * ny + nz * nz)
            if (len > 1e-12f) { nx /= len; ny /= len; nz /= len }
            fn[t * 3] = nx; fn[t * 3 + 1] = ny; fn[t * 3 + 2] = nz
        }

        // Occlusion buffers: nearest surface depth per coarse pixel for each keyframe.
        val views = keyframes.map { View(it) }
        for (view in views) {
            for (t in 0 until tCount) {
                if (!view.project(fc[t * 3], fc[t * 3 + 1], fc[t * 3 + 2]) || !view.inside(0f)) continue
                val zi = view.zIndex()
                if (view.z < view.zbuf[zi]) view.zbuf[zi] = view.z
            }
        }

        fun score(t: Int, k: Int, strict: Boolean): Float {
            val view = views[k]
            val cxw = fc[t * 3]; val cyw = fc[t * 3 + 1]; val czw = fc[t * 3 + 2]
            val ex = view.tx - cxw; val ey = view.ty - cyw; val ez = view.tz - czw
            val dist = sqrt(ex * ex + ey * ey + ez * ez)
            val cos = (fn[t * 3] * ex + fn[t * 3 + 1] * ey + fn[t * 3 + 2] * ez) / dist
            if (cos < if (strict) MIN_COS else 0.02f) return 0f
            if (!view.project(cxw, cyw, czw) || !view.inside(EDGE_MARGIN)) return 0f
            val z = view.z
            if (strict && z > view.zbuf[view.zIndex()] + 0.03f + 0.02f * z) return 0f
            for (i in 0 until 3) {
                val vi = tris[t * 3 + i]
                if (!view.project(px[vi], py[vi], pz[vi]) || !view.inside(EDGE_MARGIN)) return 0f
            }
            // Frontal and close views give the most pixels per square metre.
            val s = cos * cos / (z * z)
            return if (strict) s else s * 1e-3f
        }

        // Best keyframe per triangle.
        val label = IntArray(tCount) { -1 }
        val best = FloatArray(tCount)
        for (k in views.indices) {
            for (t in 0 until tCount) {
                val s = score(t, k, true)
                if (s > best[t]) { best[t] = s; label[t] = k }
            }
        }
        // Triangles no keyframe sees cleanly: accept any view that contains them.
        for (t in 0 until tCount) {
            if (label[t] >= 0) continue
            for (k in views.indices) {
                val s = score(t, k, false)
                if (s > best[t]) { best[t] = s; label[t] = k }
            }
        }

        // Triangle adjacency through shared edges.
        val adj = IntArray(tCount * 3) { -1 }
        run {
            val edges = HashMap<Long, Int>(tCount * 2)
            for (t in 0 until tCount) {
                for (e in 0 until 3) {
                    val a = tris[t * 3 + e]; val b = tris[t * 3 + (e + 1) % 3]
                    val key = (min(a, b).toLong() shl 32) or max(a, b).toLong()
                    val other = edges.remove(key)
                    if (other == null) {
                        edges[key] = t * 3 + e
                    } else {
                        adj[t * 3 + e] = other / 3
                        adj[other] = t
                    }
                }
            }
        }

        // Smooth labels so charts are large and seams few.
        repeat(4) {
            for (t in 0 until tCount) {
                val n0 = adj[t * 3]; val n1 = adj[t * 3 + 1]; val n2 = adj[t * 3 + 2]
                val l0 = if (n0 >= 0) label[n0] else -2
                val l1 = if (n1 >= 0) label[n1] else -3
                val l2 = if (n2 >= 0) label[n2] else -4
                val major = when {
                    l0 == l1 || l0 == l2 -> l0
                    l1 == l2 -> l1
                    else -> continue
                }
                if (major < 0 || major == label[t]) continue
                if (score(t, major, true) > 0f) label[t] = major
            }
        }

        // Charts: connected triangles sharing a keyframe.
        val chartOf = IntArray(tCount) { -1 }
        val chartLabel = ArrayList<Int>()
        val chartTris = ArrayList<IntArray>()
        val stack = IntArray(tCount)
        val members = IntArray(tCount)
        for (seed in 0 until tCount) {
            if (chartOf[seed] >= 0) continue
            val id = chartLabel.size
            val l = label[seed]
            var sp = 0
            var count = 0
            stack[sp++] = seed
            chartOf[seed] = id
            while (sp > 0) {
                val t = stack[--sp]
                members[count++] = t
                for (e in 0 until 3) {
                    val n = adj[t * 3 + e]
                    if (n >= 0 && chartOf[n] < 0 && label[n] == l) {
                        chartOf[n] = id
                        stack[sp++] = n
                    }
                }
            }
            chartLabel.add(l)
            chartTris.add(members.copyOf(count))
        }

        // Source rectangle of every textured chart inside its keyframe photo.
        val charts = chartLabel.size
        val sx = IntArray(charts); val sy = IntArray(charts); val sw = IntArray(charts); val sh = IntArray(charts)
        var sourceArea = 0.0
        for (c in 0 until charts) {
            val k = chartLabel[c]
            if (k < 0) continue
            val view = views[k]
            var u0 = Float.MAX_VALUE; var v0 = Float.MAX_VALUE; var u1 = -Float.MAX_VALUE; var v1 = -Float.MAX_VALUE
            for (t in chartTris[c]) {
                for (i in 0 until 3) {
                    val vi = tris[t * 3 + i]
                    view.project(px[vi], py[vi], pz[vi])
                    u0 = min(u0, view.u); v0 = min(v0, view.v); u1 = max(u1, view.u); v1 = max(v1, view.v)
                }
            }
            val x0 = (u0.toInt() - SOURCE_PADDING).coerceIn(0, view.w - 1)
            val y0 = (v0.toInt() - SOURCE_PADDING).coerceIn(0, view.h - 1)
            val x1 = (ceil(u1).toInt() + SOURCE_PADDING).coerceIn(x0 + 1, view.w)
            val y1 = (ceil(v1).toInt() + SOURCE_PADDING).coerceIn(y0 + 1, view.h)
            sx[c] = x0; sy[c] = y0; sw[c] = x1 - x0; sh[c] = y1 - y0
            sourceArea += (sw[c] + GUTTER).toDouble() * (sh[c] + GUTTER)
        }

        // Shelf-pack the charts, shrinking them until everything fits.
        val dx = IntArray(charts); val dy = IntArray(charts); val dw = IntArray(charts); val dh = IntArray(charts)
        val order = (0 until charts).filter { chartLabel[it] >= 0 }.sortedByDescending { sh[it] }
        var scale = min(1.0, sqrt(0.8 * atlasSize.toDouble() * atlasSize / max(1.0, sourceArea)))
        while (true) {
            if (pack(order, sw, sh, scale, atlasSize, dx, dy, dw, dh)) break
            scale *= 0.92
        }

        // Output vertices: one per (mesh vertex, chart) pair, so charts get their own UVs.
        val outIndex = HashMap<Long, Int>(vCount * 2)
        val oPos = FloatList(); val oNrm = FloatList(); val oUv = FloatList()
        val oCol = ArrayList<Int>()
        val oIdx = IntArray(tCount * 3)
        val fallbackU = (FALLBACK_BLOCK / 2f) / atlasSize
        val fallbackV = (atlasSize - FALLBACK_BLOCK / 2f) / atlasSize
        for (t in 0 until tCount) {
            val c = chartOf[t]
            val k = chartLabel[c]
            for (i in 0 until 3) {
                val vi = tris[t * 3 + i]
                val key = (vi.toLong() shl 32) or c.toLong()
                var out = outIndex[key]
                if (out == null) {
                    out = oCol.size
                    outIndex[key] = out
                    oPos.add(px[vi]); oPos.add(py[vi]); oPos.add(pz[vi])
                    oNrm.add(nrm[vi * 3]); oNrm.add(nrm[vi * 3 + 1]); oNrm.add(nrm[vi * 3 + 2])
                    if (k >= 0) {
                        val view = views[k]
                        view.project(px[vi], py[vi], pz[vi])
                        val fu = dx[c] + (view.u - sx[c]) * dw[c] / sw[c]
                        val fv = dy[c] + (view.v - sy[c]) * dh[c] / sh[c]
                        oUv.add(fu / atlasSize); oUv.add(fv / atlasSize)
                        oCol.add(0xFFFFFF)
                    } else {
                        oUv.add(fallbackU); oUv.add(fallbackV)
                        oCol.add(col[vi])
                    }
                }
                oIdx[t * 3 + i] = out
            }
        }

        // Bake: white block for untextured triangles, then every chart from its photo.
        canvas.create(atlasSize)
        canvas.fillRect(0, atlasSize - FALLBACK_BLOCK, FALLBACK_BLOCK, FALLBACK_BLOCK, 0xFFFFFF)
        val byKeyframe = order.groupBy { chartLabel[it] }
        for ((k, list) in byKeyframe) {
            for (c in list) {
                canvas.drawRegion(keyframes[k], sx[c], sy[c], sw[c], sh[c], dx[c], dy[c], dw[c], dh[c])
            }
        }
        val jpeg = canvas.encodeJpeg(90)

        val outCount = oCol.size
        return TexturedMesh(
            vertexCount = outCount,
            positions = oPos.toArray(),
            normals = oNrm.toArray(),
            uvs = oUv.toArray(),
            colors = oCol.toIntArray(),
            indices = oIdx,
            atlasJpeg = jpeg,
            atlasSize = atlasSize,
            min = mesh.min.copyOf(),
            max = mesh.max.copyOf(),
        )
    }

    /** Shelf packing; the bottom-left corner is reserved for the fallback block. */
    private fun pack(
        order: List<Int>, sw: IntArray, sh: IntArray, scale: Double, size: Int,
        dx: IntArray, dy: IntArray, dw: IntArray, dh: IntArray,
    ): Boolean {
        var x = 0
        var y = 0
        var shelf = 0
        val limit = size - FALLBACK_BLOCK - GUTTER
        for (c in order) {
            val w = max(1, ceil(sw[c] * scale).toInt())
            val h = max(1, ceil(sh[c] * scale).toInt())
            if (w > size) return false
            if (x + w > size) {
                x = 0
                y += shelf + GUTTER
                shelf = 0
            }
            if (y + h > limit) return false
            dx[c] = x; dy[c] = y; dw[c] = w; dh[c] = h
            x += w + GUTTER
            if (h > shelf) shelf = h
        }
        return true
    }

    private class FloatList {
        var data = FloatArray(1 shl 16)
        var size = 0
        fun add(v: Float) {
            if (size == data.size) data = data.copyOf(size * 2)
            data[size++] = v
        }
        fun toArray(): FloatArray = data.copyOf(size)
    }
}
