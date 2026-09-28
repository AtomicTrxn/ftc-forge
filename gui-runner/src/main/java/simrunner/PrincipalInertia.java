package simrunner;

import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;

/** Symmetric 3x3 CAD inertia assembly and principal-axis decomposition. */
final class PrincipalInertia {
    private final double[][] tensor = new double[3][3];

    void addRotated(double[][] local, Quaternion rotation) {
        Matrix3f r = rotation.toRotationMatrix();
        for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++)
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
                tensor[a][b] += r.get(a, i) * local[i][j] * r.get(b, j);
    }

    void addParallelAxis(double mass, Vector3f offset) {
        double[] p = {offset.x, offset.y, offset.z};
        double radius2 = p[0] * p[0] + p[1] * p[1] + p[2] * p[2];
        for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++)
            tensor[a][b] += mass * ((a == b ? radius2 : 0) - p[a] * p[b]);
    }

    double get(int row, int column) { return tensor[row][column]; }

    record Principal(Quaternion rotation, Vector3f moments) { }

    Principal diagonalize(String name) {
        double[][] a = new double[3][3], vectors = new double[3][3];
        for (int i = 0; i < 3; i++) {
            vectors[i][i] = 1;
            for (int j = 0; j < 3; j++) {
                a[i][j] = (tensor[i][j] + tensor[j][i]) / 2;
                if (!Double.isFinite(a[i][j])) throw new IllegalArgumentException("Non-finite inertia: " + name);
            }
        }
        for (int iteration = 0; iteration < 50; iteration++) {
            int p = 0, q = 1;
            double largest = Math.abs(a[p][q]);
            for (int i = 0; i < 3; i++) for (int j = i + 1; j < 3; j++) {
                if (Math.abs(a[i][j]) > largest) { p = i; q = j; largest = Math.abs(a[i][j]); }
            }
            double scale = Math.max(1, Math.max(Math.abs(a[0][0]), Math.max(Math.abs(a[1][1]), Math.abs(a[2][2]))));
            if (largest < 1e-12 * scale) break;
            double theta = .5 * Math.atan2(2 * a[p][q], a[q][q] - a[p][p]);
            double c = Math.cos(theta), s = Math.sin(theta);
            // Columns p/q of R undergo a proper rotation; A' = R^T A R.
            double app = c * c * a[p][p] - 2 * s * c * a[p][q] + s * s * a[q][q];
            double aqq = s * s * a[p][p] + 2 * s * c * a[p][q] + c * c * a[q][q];
            for (int k = 0; k < 3; k++) {
                if (k != p && k != q) {
                    double akp = c * a[k][p] - s * a[k][q];
                    double akq = s * a[k][p] + c * a[k][q];
                    a[k][p] = a[p][k] = akp;
                    a[k][q] = a[q][k] = akq;
                }
                double vkp = c * vectors[k][p] - s * vectors[k][q];
                double vkq = s * vectors[k][p] + c * vectors[k][q];
                vectors[k][p] = vkp;
                vectors[k][q] = vkq;
            }
            a[p][p] = app; a[q][q] = aqq; a[p][q] = a[q][p] = 0;
        }
        double[] eigenvalues = {a[0][0], a[1][1], a[2][2]};
        for (double value : eigenvalues) {
            if (!Double.isFinite(value) || value <= 1e-10 || value > Float.MAX_VALUE)
                throw new IllegalArgumentException("Inertia must be positive definite: " + name);
        }
        Matrix3f basis = new Matrix3f();
        for (int col = 0; col < 3; col++) basis.setColumn(col, new Vector3f(
            (float) vectors[0][col], (float) vectors[1][col], (float) vectors[2][col]));
        return new Principal(new Quaternion().fromRotationMatrix(basis).normalizeLocal(),
            new Vector3f((float) eigenvalues[0], (float) eigenvalues[1], (float) eigenvalues[2]));
    }
}
