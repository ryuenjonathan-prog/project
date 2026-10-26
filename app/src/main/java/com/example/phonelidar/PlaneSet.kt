package com.example.phonelidar

import kotlin.math.abs
import kotlin.math.sqrt

/**
 * A detected planar surface (floor, wall, table, cabinet side) in world space.
 * [polygon] is a convex outline as (x, z) pairs in the plane's local frame
 * spanned by [axisX] and [axisZ]; [normal] is the plane's Y axis.
 */
class SnapPlane(
    val center: FloatArray,
    val normal: FloatArray,
    val axisX: FloatArray,
    val axisZ: FloatArray,
    polygon: FloatArray,
) {
    private val edgeCount = polygon.size / 2
    private val ax = FloatArray(edgeCount)
    private val az = FloatArray(edgeCount)
    private val nx = FloatArray(edgeCount)
    private val nz = FloatArray(edgeCount)

    init {
        // Orientation of the outline, so edge normals can be made to point inwards.
        var area2 = 0f
        for (i in 0 until edgeCount) {
            val j = (i + 1) % edgeCount
            area2 += polygon[i * 2] * polygon[j * 2 + 1] - polygon[j * 2] * polygon[i * 2 + 1]
        }
        val sign = if (area2 >= 0f) 1f else -1f
        for (i in 0 until edgeCount) {
            val j = (i + 1) % edgeCount
            ax[i] = polygon[i * 2]
            az[i] = polygon[i * 2 + 1]
            val ex = polygon[j * 2] - ax[i]
            val ez = polygon[j * 2 + 1] - az[i]
            val len = sqrt(ex * ex + ez * ez).coerceAtLeast(1e-6f)
            // Inward normal of edge (ex, ez) for this orientation.
            nx[i] = -ez / len * sign
            nz[i] = ex / len * sign
        }
    }

    val area: Float = run {
        var a = 0f
        for (i in 0 until polygon.size / 2) {
            val j = (i + 1) % (polygon.size / 2)
            a += polygon[i * 2] * polygon[j * 2 + 1] - polygon[j * 2] * polygon[i * 2 + 1]
        }
        abs(a) * 0.5f
    }

    /** True if local point (lx, lz) is inside the outline grown by [margin]. */
    fun contains(lx: Float, lz: Float, margin: Float): Boolean {
        for (i in 0 until edgeCount) {
            if ((lx - ax[i]) * nx[i] + (lz - az[i]) * nz[i] < -margin) return false
        }
        return edgeCount >= 3
    }
}

/**
 * Pulls mesh vertices that lie close to a detected plane exactly onto it, which turns
 * noisy depth into perfectly flat walls, floors and furniture faces.
 */
class PlaneSet(
    planes: List<SnapPlane>,
    private val snapDistance: Float = 0.05f,
    private val margin: Float = 0.15f,
    private val minAlignment: Float = 0.8f,
    minArea: Float = 0.08f,
) {
    val planes: List<SnapPlane> = planes.filter { it.area >= minArea }

    val isEmpty: Boolean get() = planes.isEmpty()

    /**
     * Snaps the vertex at p (position) with normal n in place.
     * Returns true if it was moved onto a plane.
     */
    fun snap(p: FloatArray, n: FloatArray): Boolean {
        var best: SnapPlane? = null
        var bestDist = snapDistance
        var bestSigned = 0f
        for (plane in planes) {
            val pn = plane.normal
            val align = n[0] * pn[0] + n[1] * pn[1] + n[2] * pn[2]
            if (abs(align) < minAlignment) continue
            val dx = p[0] - plane.center[0]
            val dy = p[1] - plane.center[1]
            val dz = p[2] - plane.center[2]
            val dist = dx * pn[0] + dy * pn[1] + dz * pn[2]
            if (abs(dist) >= bestDist) continue
            val lx = dx * plane.axisX[0] + dy * plane.axisX[1] + dz * plane.axisX[2]
            val lz = dx * plane.axisZ[0] + dy * plane.axisZ[1] + dz * plane.axisZ[2]
            if (!plane.contains(lx, lz, margin)) continue
            best = plane
            bestDist = abs(dist)
            bestSigned = dist
        }
        val plane = best ?: return false
        val pn = plane.normal
        p[0] -= pn[0] * bestSigned
        p[1] -= pn[1] * bestSigned
        p[2] -= pn[2] * bestSigned
        val s = if (n[0] * pn[0] + n[1] * pn[1] + n[2] * pn[2] >= 0f) 1f else -1f
        n[0] = pn[0] * s
        n[1] = pn[1] * s
        n[2] = pn[2] * s
        return true
    }

    companion object {
        val EMPTY = PlaneSet(emptyList())
    }
}
