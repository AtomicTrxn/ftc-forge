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
        byte[] bytes = Files.readAllBytes(path);
        if (bytes.length < 15) throw new IOException("Empty STL: " + path);
        List<Float> coordinates = new ArrayList<>();
        boolean binary = false;
        if (bytes.length >= 84) {
            long count = Integer.toUnsignedLong(ByteBuffer.wrap(bytes, 80, 4).order(ByteOrder.LITTLE_ENDIAN).getInt());
            binary = count > 0 && 84 + count * 50 == bytes.length;
        }
        if (binary) {
            ByteBuffer data = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN);
            int count = data.getInt(80);
            if (count > 200_000) throw new IOException("STL exceeds 200,000 triangles: " + path);
            for (int i = 0; i < count; i++) {
                int start = 84 + i * 50;
                for (int vertex = 0; vertex < 3; vertex++) {
                    int pos = start + 12 + vertex * 12;
                    add(coordinates, data.getFloat(pos), data.getFloat(pos + 4), data.getFloat(pos + 8), scale);
                }
            }
        } else {
            String ascii = new String(bytes, StandardCharsets.US_ASCII);
            for (String line : ascii.split("\\R")) {
                String trimmed = line.trim();
                if (!trimmed.startsWith("vertex ")) continue;
                String[] parts = trimmed.split("\\s+");
                if (parts.length != 4) throw new IOException("Invalid STL vertex: " + line);
                add(coordinates, Float.parseFloat(parts[1]), Float.parseFloat(parts[2]),
                    Float.parseFloat(parts[3]), scale);
                if (coordinates.size() > 200_000 * 9) throw new IOException("STL exceeds 200,000 triangles: " + path);
            }
        }
        if (coordinates.isEmpty() || coordinates.size() % 9 != 0)
            throw new IOException("STL has no complete triangles: " + path);
        float[] positions = new float[coordinates.size()];
        int[] indices = new int[coordinates.size() / 3];
        for (int i = 0; i < positions.length; i++) positions[i] = coordinates.get(i);
        for (int i = 0; i < indices.length; i++) indices[i] = i;
        Mesh mesh = new Mesh();
        mesh.setMode(Mesh.Mode.Triangles);
        mesh.setBuffer(VertexBuffer.Type.Position, 3, positions);
        mesh.setBuffer(VertexBuffer.Type.Index, 3, indices);
        mesh.updateBound();
        return mesh;
    }

    private static void add(List<Float> output, float x, float y, float z, double[] scale) throws IOException {
        float a = (float) (x * scale[0]), b = (float) (z * scale[2]), c = (float) (-y * scale[1]);
        if (!Float.isFinite(a) || !Float.isFinite(b) || !Float.isFinite(c))
            throw new IOException("STL contains non-finite vertex");
        output.add(a); output.add(b); output.add(c);
    }
}
