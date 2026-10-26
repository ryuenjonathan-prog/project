package com.example.phonelidar

import java.io.BufferedWriter
import java.io.OutputStream
import java.io.OutputStreamWriter
import java.nio.ByteBuffer
import java.nio.ByteOrder

/** The finished scan: the mesh, plus a textured version when keyframes were available. */
class FinalModel(val meshes: List<ChunkMesh>, val textured: TexturedMesh?)

/** One output file of an export. */
class ExportFile(val name: String, val mimeType: String, val write: (OutputStream) -> Unit)

/** Export formats for the scanned mesh. */
enum class MeshFormat(val extension: String, val mimeType: String, val title: String) {
    GLB("glb", "model/gltf-binary", "GLB — 3D-модель с текстурой (рекомендуется)"),
    OBJ("obj", "model/obj", "OBJ + MTL + JPG — для Blender, 3ds Max, Unity"),
    PLY("ply", "application/octet-stream", "PLY — сетка с цветами вершин (MeshLab)");

    /** Files to write for [model]; the first one is the main file. */
    fun files(baseName: String, model: FinalModel): List<ExportFile> {
        val tm = model.textured
        return when (this) {
            GLB -> listOf(
                ExportFile("$baseName.glb", mimeType) { out ->
                    if (tm != null) MeshWriters.writeGlbTextured(out, tm) else MeshWriters.writeGlb(out, model.meshes)
                },
            )
            OBJ -> if (tm != null) {
                listOf(
                    ExportFile("$baseName.obj", mimeType) { MeshWriters.writeObjTextured(it, tm, "$baseName.mtl") },
                    ExportFile("$baseName.mtl", "text/plain") { MeshWriters.writeMtl(it, "$baseName.jpg") },
                    ExportFile("$baseName.jpg", "image/jpeg") { it.write(tm.atlasJpeg) },
                )
            } else {
                listOf(ExportFile("$baseName.obj", mimeType) { MeshWriters.writeObj(it, model.meshes) })
            }
            PLY -> listOf(ExportFile("$baseName.ply", mimeType) { MeshWriters.writePly(it, model.meshes) })
        }
    }
}

/**
 * Serializes chunk meshes into one model. Coordinates are ARCore world space: meters, Y up,
 * which matches glTF's convention.
 */
object MeshWriters {

    fun writeGlb(out: OutputStream, meshes: List<ChunkMesh>) {
        val vertexCount = meshes.sumOf { it.vertexCount }
        val indexCount = meshes.sumOf { it.indexCount }
        val min = floatArrayOf(Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE)
        val max = floatArrayOf(-Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE)
        for (m in meshes) {
            for (a in 0 until 3) {
                if (m.min[a] < min[a]) min[a] = m.min[a]
                if (m.max[a] > max[a]) max[a] = m.max[a]
            }
        }

        val posBytes = vertexCount * 12
        val nrmBytes = vertexCount * 12
        val colBytes = vertexCount * 4
        val idxBytes = indexCount * 4
        val binLength = posBytes + nrmBytes + colBytes + idxBytes

        val json = buildString {
            append("{\"asset\":{\"version\":\"2.0\",\"generator\":\"Phone LiDAR\"},")
            append("\"scene\":0,\"scenes\":[{\"nodes\":[0]}],")
            append("\"nodes\":[{\"mesh\":0,\"name\":\"Scan\"}],")
            append("\"meshes\":[{\"name\":\"Scan\",\"primitives\":[{")
            append("\"attributes\":{\"POSITION\":0,\"NORMAL\":1,\"COLOR_0\":2},")
            append("\"indices\":3,\"material\":0,\"mode\":4}]}],")
            append("\"materials\":[{\"name\":\"VertexColor\",\"doubleSided\":true,")
            append("\"pbrMetallicRoughness\":{\"baseColorFactor\":[1,1,1,1],")
            append("\"metallicFactor\":0,\"roughnessFactor\":1}}],")
            append("\"buffers\":[{\"byteLength\":$binLength}],")
            append("\"bufferViews\":[")
            append("{\"buffer\":0,\"byteOffset\":0,\"byteLength\":$posBytes,\"target\":34962},")
            append("{\"buffer\":0,\"byteOffset\":$posBytes,\"byteLength\":$nrmBytes,\"target\":34962},")
            append("{\"buffer\":0,\"byteOffset\":${posBytes + nrmBytes},\"byteLength\":$colBytes,\"target\":34962},")
            append("{\"buffer\":0,\"byteOffset\":${posBytes + nrmBytes + colBytes},\"byteLength\":$idxBytes,\"target\":34963}],")
            append("\"accessors\":[")
            append("{\"bufferView\":0,\"componentType\":5126,\"count\":$vertexCount,\"type\":\"VEC3\",")
            append("\"min\":[${min[0]},${min[1]},${min[2]}],\"max\":[${max[0]},${max[1]},${max[2]}]},")
            append("{\"bufferView\":1,\"componentType\":5126,\"count\":$vertexCount,\"type\":\"VEC3\"},")
            append("{\"bufferView\":2,\"componentType\":5121,\"normalized\":true,\"count\":$vertexCount,\"type\":\"VEC4\"},")
            append("{\"bufferView\":3,\"componentType\":5125,\"count\":$indexCount,\"type\":\"SCALAR\"}]}")
        }
        val jsonBytes = json.toByteArray(Charsets.UTF_8)
        val jsonPadded = (jsonBytes.size + 3) / 4 * 4
        val total = 12 + 8 + jsonPadded + 8 + binLength

        val w = LeWriter(out)
        w.int(0x46546C67) // "glTF"
        w.int(2)
        w.int(total)
        w.int(jsonPadded)
        w.int(0x4E4F534A) // "JSON"
        w.bytes(jsonBytes)
        repeat(jsonPadded - jsonBytes.size) { w.byte(0x20) }
        w.int(binLength)
        w.int(0x004E4942) // "BIN\0"

        for (m in meshes) {
            for (i in 0 until m.vertexCount) {
                val o = i * ChunkMesh.STRIDE
                w.float(m.vertices.getFloat(o))
                w.float(m.vertices.getFloat(o + 4))
                w.float(m.vertices.getFloat(o + 8))
            }
        }
        for (m in meshes) {
            for (i in 0 until m.vertexCount) {
                val o = i * ChunkMesh.STRIDE + ChunkMesh.NORMAL_OFFSET
                w.float(m.vertices.getFloat(o))
                w.float(m.vertices.getFloat(o + 4))
                w.float(m.vertices.getFloat(o + 8))
            }
        }
        for (m in meshes) {
            for (i in 0 until m.vertexCount) {
                val o = i * ChunkMesh.STRIDE + ChunkMesh.COLOR_OFFSET
                // glTF vertex colors are linear; camera colors are sRGB.
                for (c in 0 until 3) w.byte(SRGB_TO_LINEAR[m.vertices.get(o + c).toInt() and 0xFF])
                w.byte(0xFF)
            }
        }
        var base = 0
        for (m in meshes) {
            for (i in 0 until m.indexCount) w.int(base + m.indices.get(i))
            base += m.vertexCount
        }
        w.flush()
    }

    fun writeGlbTextured(out: OutputStream, m: TexturedMesh) {
        val n = m.vertexCount
        val posBytes = n * 12
        val nrmBytes = n * 12
        val uvBytes = n * 8
        val colBytes = n * 4
        val idxBytes = m.indexCount * 4
        val imgBytes = m.atlasJpeg.size
        val imgPadded = (imgBytes + 3) / 4 * 4
        val o1 = posBytes
        val o2 = o1 + nrmBytes
        val o3 = o2 + uvBytes
        val o4 = o3 + colBytes
        val o5 = o4 + idxBytes
        val binLength = o5 + imgPadded

        val json = buildString {
            append("{\"asset\":{\"version\":\"2.0\",\"generator\":\"Phone LiDAR\"},")
            append("\"scene\":0,\"scenes\":[{\"nodes\":[0]}],")
            append("\"nodes\":[{\"mesh\":0,\"name\":\"Scan\"}],")
            append("\"meshes\":[{\"name\":\"Scan\",\"primitives\":[{")
            append("\"attributes\":{\"POSITION\":0,\"NORMAL\":1,\"TEXCOORD_0\":2,\"COLOR_0\":3},")
            append("\"indices\":4,\"material\":0,\"mode\":4}]}],")
            append("\"materials\":[{\"name\":\"ScanTexture\",\"doubleSided\":true,")
            append("\"pbrMetallicRoughness\":{\"baseColorTexture\":{\"index\":0},")
            append("\"metallicFactor\":0,\"roughnessFactor\":1}}],")
            append("\"textures\":[{\"sampler\":0,\"source\":0}],")
            append("\"samplers\":[{\"magFilter\":9729,\"minFilter\":9987,\"wrapS\":33071,\"wrapT\":33071}],")
            append("\"images\":[{\"bufferView\":5,\"mimeType\":\"image/jpeg\"}],")
            append("\"buffers\":[{\"byteLength\":$binLength}],")
            append("\"bufferViews\":[")
            append("{\"buffer\":0,\"byteOffset\":0,\"byteLength\":$posBytes,\"target\":34962},")
            append("{\"buffer\":0,\"byteOffset\":$o1,\"byteLength\":$nrmBytes,\"target\":34962},")
            append("{\"buffer\":0,\"byteOffset\":$o2,\"byteLength\":$uvBytes,\"target\":34962},")
            append("{\"buffer\":0,\"byteOffset\":$o3,\"byteLength\":$colBytes,\"target\":34962},")
            append("{\"buffer\":0,\"byteOffset\":$o4,\"byteLength\":$idxBytes,\"target\":34963},")
            append("{\"buffer\":0,\"byteOffset\":$o5,\"byteLength\":$imgBytes}],")
            append("\"accessors\":[")
            append("{\"bufferView\":0,\"componentType\":5126,\"count\":$n,\"type\":\"VEC3\",")
            append("\"min\":[${m.min[0]},${m.min[1]},${m.min[2]}],\"max\":[${m.max[0]},${m.max[1]},${m.max[2]}]},")
            append("{\"bufferView\":1,\"componentType\":5126,\"count\":$n,\"type\":\"VEC3\"},")
            append("{\"bufferView\":2,\"componentType\":5126,\"count\":$n,\"type\":\"VEC2\"},")
            append("{\"bufferView\":3,\"componentType\":5121,\"normalized\":true,\"count\":$n,\"type\":\"VEC4\"},")
            append("{\"bufferView\":4,\"componentType\":5125,\"count\":${m.indexCount},\"type\":\"SCALAR\"}]}")
        }
        val jsonBytes = json.toByteArray(Charsets.UTF_8)
        val jsonPadded = (jsonBytes.size + 3) / 4 * 4
        val w = LeWriter(out)
        w.int(0x46546C67)
        w.int(2)
        w.int(12 + 8 + jsonPadded + 8 + binLength)
        w.int(jsonPadded)
        w.int(0x4E4F534A)
        w.bytes(jsonBytes)
        repeat(jsonPadded - jsonBytes.size) { w.byte(0x20) }
        w.int(binLength)
        w.int(0x004E4942)
        for (i in 0 until n * 3) w.float(m.positions[i])
        for (i in 0 until n * 3) w.float(m.normals[i])
        for (i in 0 until n * 2) w.float(m.uvs[i])
        for (i in 0 until n) {
            val c = m.colors[i]
            w.byte(SRGB_TO_LINEAR[(c shr 16) and 0xFF])
            w.byte(SRGB_TO_LINEAR[(c shr 8) and 0xFF])
            w.byte(SRGB_TO_LINEAR[c and 0xFF])
            w.byte(0xFF)
        }
        for (i in 0 until m.indexCount) w.int(m.indices[i])
        w.bytes(m.atlasJpeg)
        repeat(imgPadded - imgBytes) { w.byte(0) }
        w.flush()
    }

    fun writeObjTextured(out: OutputStream, m: TexturedMesh, mtlName: String) {
        val w = BufferedWriter(OutputStreamWriter(out, Charsets.US_ASCII), 1 shl 16)
        w.write("# Phone LiDAR scan\nmtllib $mtlName\n")
        for (i in 0 until m.vertexCount) {
            w.write("v "); w.write(fmt(m.positions[i * 3])); w.write(" ")
            w.write(fmt(m.positions[i * 3 + 1])); w.write(" "); w.write(fmt(m.positions[i * 3 + 2]))
            val c = m.colors[i]
            if (c != 0xFFFFFF) {
                w.write(" "); w.write(fmt(((c shr 16) and 0xFF) / 255f))
                w.write(" "); w.write(fmt(((c shr 8) and 0xFF) / 255f))
                w.write(" "); w.write(fmt((c and 0xFF) / 255f))
            }
            w.write("\n")
        }
        for (i in 0 until m.vertexCount) {
            // OBJ texture coordinates start at the bottom-left.
            w.write("vt "); w.write(fmt(m.uvs[i * 2])); w.write(" "); w.write(fmt(1f - m.uvs[i * 2 + 1])); w.write("\n")
        }
        for (i in 0 until m.vertexCount) {
            w.write("vn "); w.write(fmt(m.normals[i * 3])); w.write(" ")
            w.write(fmt(m.normals[i * 3 + 1])); w.write(" "); w.write(fmt(m.normals[i * 3 + 2])); w.write("\n")
        }
        w.write("usemtl scan\n")
        var i = 0
        while (i < m.indexCount) {
            val a = m.indices[i] + 1; val b = m.indices[i + 1] + 1; val c = m.indices[i + 2] + 1
            w.write("f $a/$a/$a $b/$b/$b $c/$c/$c\n")
            i += 3
        }
        w.flush()
    }

    fun writeMtl(out: OutputStream, textureName: String) {
        val text = "newmtl scan\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nillum 1\nmap_Kd $textureName\n"
        out.write(text.toByteArray(Charsets.US_ASCII))
        out.flush()
    }

    fun writePly(out: OutputStream, meshes: List<ChunkMesh>) {
        val vertexCount = meshes.sumOf { it.vertexCount }
        val faceCount = meshes.sumOf { it.indexCount } / 3
        val header = buildString {
            append("ply\nformat binary_little_endian 1.0\n")
            append("comment Phone LiDAR scan\n")
            append("element vertex $vertexCount\n")
            append("property float x\nproperty float y\nproperty float z\n")
            append("property float nx\nproperty float ny\nproperty float nz\n")
            append("property uchar red\nproperty uchar green\nproperty uchar blue\n")
            append("element face $faceCount\n")
            append("property list uchar int vertex_indices\n")
            append("end_header\n")
        }
        val w = LeWriter(out)
        w.bytes(header.toByteArray(Charsets.US_ASCII))
        for (m in meshes) {
            for (i in 0 until m.vertexCount) {
                val o = i * ChunkMesh.STRIDE
                for (f in 0 until 6) w.float(m.vertices.getFloat(o + f * 4))
                for (c in 0 until 3) w.byte(m.vertices.get(o + ChunkMesh.COLOR_OFFSET + c).toInt())
            }
        }
        var base = 0
        for (m in meshes) {
            var i = 0
            while (i < m.indexCount) {
                w.byte(3)
                w.int(base + m.indices.get(i))
                w.int(base + m.indices.get(i + 1))
                w.int(base + m.indices.get(i + 2))
                i += 3
            }
            base += m.vertexCount
        }
        w.flush()
    }

    fun writeObj(out: OutputStream, meshes: List<ChunkMesh>) {
        val w = BufferedWriter(OutputStreamWriter(out, Charsets.US_ASCII), 1 shl 16)
        w.write("# Phone LiDAR scan\n# v x y z r g b (vertex colors)\n")
        for (m in meshes) {
            for (i in 0 until m.vertexCount) {
                val o = i * ChunkMesh.STRIDE
                w.write("v ")
                w.write(fmt(m.vertices.getFloat(o))); w.write(" ")
                w.write(fmt(m.vertices.getFloat(o + 4))); w.write(" ")
                w.write(fmt(m.vertices.getFloat(o + 8)))
                for (c in 0 until 3) {
                    val byte = m.vertices.get(o + ChunkMesh.COLOR_OFFSET + c).toInt() and 0xFF
                    w.write(" ")
                    w.write(fmt(byte / 255f))
                }
                w.write("\n")
            }
        }
        for (m in meshes) {
            for (i in 0 until m.vertexCount) {
                val o = i * ChunkMesh.STRIDE + ChunkMesh.NORMAL_OFFSET
                w.write("vn ")
                w.write(fmt(m.vertices.getFloat(o))); w.write(" ")
                w.write(fmt(m.vertices.getFloat(o + 4))); w.write(" ")
                w.write(fmt(m.vertices.getFloat(o + 8))); w.write("\n")
            }
        }
        var base = 1
        for (m in meshes) {
            var i = 0
            while (i < m.indexCount) {
                val a = base + m.indices.get(i)
                val b = base + m.indices.get(i + 1)
                val c = base + m.indices.get(i + 2)
                w.write("f $a//$a $b//$b $c//$c\n")
                i += 3
            }
            base += m.vertexCount
        }
        w.flush()
    }

    private val SRGB_TO_LINEAR = IntArray(256) { i ->
        val c = i / 255.0
        val linear = if (c <= 0.04045) c / 12.92 else Math.pow((c + 0.055) / 1.055, 2.4)
        Math.round(linear * 255.0).toInt()
    }

    /** Compact fixed-point formatting (4 decimals) that is locale independent. */
    private fun fmt(value: Float): String {
        val scaled = Math.round(value * 10000.0)
        val neg = scaled < 0
        val abs = if (neg) -scaled else scaled
        val intPart = abs / 10000
        val frac = (abs % 10000).toString().padStart(4, '0')
        return (if (neg) "-" else "") + intPart + "." + frac
    }

    /** Little-endian buffered binary writer. */
    private class LeWriter(private val out: OutputStream) {
        private val buf = ByteBuffer.allocate(1 shl 16).order(ByteOrder.LITTLE_ENDIAN)

        private fun ensure(n: Int) {
            if (buf.remaining() < n) flushBuffer()
        }

        private fun flushBuffer() {
            out.write(buf.array(), 0, buf.position())
            buf.clear()
        }

        fun int(v: Int) { ensure(4); buf.putInt(v) }
        fun float(v: Float) { ensure(4); buf.putFloat(v) }
        fun byte(v: Int) { ensure(1); buf.put(v.toByte()) }
        fun bytes(b: ByteArray) { flushBuffer(); out.write(b) }

        fun flush() {
            flushBuffer()
            out.flush()
        }
    }
}
