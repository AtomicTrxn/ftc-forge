package simrunner;

import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/** Minimal binary/ASCII STL reader for collision meshes emitted by CAD URDF exporters. */
final class StlMeshLoader {
    private StlMeshLoader() {}

    static Mesh load(Path path, double[] scale) throws IOException {
        return load(path, scale, 200_000);
    }

    static Mesh load(Path path, double[] scale, int maxTriangles) throws IOException {
        byte[] bytes = Files.readAllBytes(path);
        if (bytes.length < 15) throw new IOException("Empty STL: " + path);
        float[] positions;
        boolean binary = false;
        if (bytes.length >= 84) {
            long count = Integer.toUnsignedLong(ByteBuffer.wrap(bytes, 80, 4).order(ByteOrder.LITTLE_ENDIAN).getInt());
            binary = count > 0 && 84 + count * 50 == bytes.length;
        }
        if (binary) {
            ByteBuffer data = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN);
            int count = data.getInt(80);
            if (count > maxTriangles) throw new IOException("STL exceeds " + maxTriangles + " triangles: " + path);
            positions = new float[count * 9];
            for (int i = 0; i < count; i++) {
                int start = 84 + i * 50;
                for (int vertex = 0; vertex < 3; vertex++) {
                    int pos = start + 12 + vertex * 12;
                    transform(positions, i * 9 + vertex * 3, data.getFloat(pos),
                        data.getFloat(pos + 4), data.getFloat(pos + 8), scale);
                }
            }
        } else {
            List<Float> coordinates = new ArrayList<>();
            String ascii = new String(bytes, StandardCharsets.US_ASCII);
            for (String line : ascii.split("\\R")) {
                String trimmed = line.trim();
                if (!trimmed.startsWith("vertex ")) continue;
                String[] parts = trimmed.split("\\s+");
                if (parts.length != 4) throw new IOException("Invalid STL vertex: " + line);
                float[] point = new float[3];
                transform(point, 0, Float.parseFloat(parts[1]), Float.parseFloat(parts[2]), Float.parseFloat(parts[3]), scale);
                for (float coordinate : point) coordinates.add(coordinate);
                if (coordinates.size() > maxTriangles * 9) throw new IOException("STL exceeds " + maxTriangles + " triangles: " + path);
            }
            positions = new float[coordinates.size()];
            for (int i = 0; i < positions.length; i++) positions[i] = coordinates.get(i);
        }
        if (positions.length == 0 || positions.length % 9 != 0)
            throw new IOException("STL has no complete triangles: " + path);
        int[] indices = new int[positions.length / 3];
        for (int i = 0; i < indices.length; i++) indices[i] = i;
        float[] normals = new float[positions.length];
        for (int i = 0; i < positions.length; i += 9) {
            var a = new com.jme3.math.Vector3f(positions[i + 3] - positions[i], positions[i + 4] - positions[i + 1], positions[i + 5] - positions[i + 2]);
            var b = new com.jme3.math.Vector3f(positions[i + 6] - positions[i], positions[i + 7] - positions[i + 1], positions[i + 8] - positions[i + 2]);
            var normal = a.cross(b).normalizeLocal();
            for (int vertex = 0; vertex < 3; vertex++) {
                normals[i + vertex * 3] = normal.x;
                normals[i + vertex * 3 + 1] = normal.y;
                normals[i + vertex * 3 + 2] = normal.z;
            }
        }
        Mesh mesh = new Mesh();
        mesh.setMode(Mesh.Mode.Triangles);
        mesh.setBuffer(VertexBuffer.Type.Position, 3, positions);
        mesh.setBuffer(VertexBuffer.Type.Index, 3, indices);
        mesh.setBuffer(VertexBuffer.Type.Normal, 3, normals);
        mesh.setStatic();
        mesh.updateBound();
        return mesh;
    }

    private static void transform(float[] output, int offset, float x, float y, float z, double[] scale) throws IOException {
        float a = (float) (x * scale[0]), b = (float) (z * scale[2]), c = (float) (-y * scale[1]);
        if (!Float.isFinite(a) || !Float.isFinite(b) || !Float.isFinite(c))
            throw new IOException("STL contains non-finite vertex");
        output[offset] = a; output[offset + 1] = b; output[offset + 2] = c;
    }
}
