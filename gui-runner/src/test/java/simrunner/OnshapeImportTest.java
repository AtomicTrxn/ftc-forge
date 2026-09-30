package simrunner;

import com.jme3.asset.DesktopAssetManager;
import com.jme3.scene.Geometry;
import com.jme3.scene.VertexBuffer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import simcore.RobotUrdf;

import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.*;

class OnshapeImportTest {
    @TempDir Path temp;

    @Test void loadsPackageVisualsWithMasslessAssemblyFrameAndCachesRepeatedMesh() throws Exception {
        Path pkg = temp.resolve("pkg_robot");
        Files.createDirectories(pkg.resolve("meshes"));
        Files.createDirectories(pkg.resolve("urdf"));
        Files.writeString(pkg.resolve("meshes/part.stl"), "solid p\nfacet normal 0 0 1\nouter loop\n"
            + "vertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid p\n");
        Path path = pkg.resolve("urdf/robot.urdf");
        String part = "<inertial><mass value='1'/><inertia ixx='1' iyy='1' izz='1' ixy='0' ixz='0' iyz='0'/></inertial>"
            + "<visual><origin xyz='.1 .2 .3'/><geometry><mesh filename='package://pkg_robot/meshes/part.stl' scale='2 3 4'/></geometry>"
            + "<material name='red'/></visual><collision><geometry><box size='.1 .1 .1'/></geometry></collision>";
        Files.writeString(path, "<robot name='cad'><material name='red'><color rgba='1 0 0 1'/></material><link name='root'/>"
            + "<link name='a'>" + part + "</link><link name='b'>" + part + "</link>"
            + "<joint name='a_mount' type='fixed'><parent link='root'/><child link='a'/></joint>"
            + "<joint name='b_mount' type='fixed'><parent link='root'/><child link='b'/><origin xyz='1 0 0'/></joint></robot>");
        RobotUrdf robot = RobotUrdf.parse(path);
        assertEquals(0, robot.links.get("root").massKg());
        assertEquals(2, robot.totalMassKg());
        assertEquals(1, robot.links.get("a").visuals().size());
        assertEquals(1, robot.links.get("a").collisions().size());
        ImportedRobotScene scene = new ImportedRobotScene(robot, path, new HardwareMap(), new DesktopAssetManager(true), 8);
        var a = (Geometry) scene.linkNodes.get("a").getChild(0);
        var b = (Geometry) scene.linkNodes.get("b").getChild(0);
        assertEquals(1, scene.linkNodes.get("a").getQuantity(), "Collision proxy must not replace or duplicate visual mesh");
        assertSame(a.getMesh(), b.getMesh(), "Repeated CAD hardware should share immutable mesh buffers");
        assertEquals(1, a.getMesh().getTriangleCount());
        var positions = a.getMesh().getFloatBuffer(VertexBuffer.Type.Position);
        assertEquals(2, positions.get(3));
        assertEquals(-3, positions.get(8));
        var normals = a.getMesh().getFloatBuffer(VertexBuffer.Type.Normal);
        assertEquals(1, normals.get(1), .0001);
        assertEquals(.1, a.getLocalTranslation().x, .0001);
        assertEquals(.3, a.getLocalTranslation().y, .0001);
        assertEquals(-.2, a.getLocalTranslation().z, .0001);
        assertEquals(new com.jme3.math.ColorRGBA(1, 0, 0, 1), a.getMaterial().getParam("Diffuse").getValue());
        assertEquals(4, robot.withTotalMassKg(8).links.get("a").massKg());
    }

    @Test void missingInertiaStillRejectsLinksWithGeometry() throws Exception {
        Path file = temp.resolve("invalid.urdf");
        Files.writeString(file, "<robot name='bad'><link name='part'><visual><geometry><box size='1 1 1'/></geometry></visual></link></robot>");
        assertTrue(assertThrows(IllegalArgumentException.class, () -> RobotUrdf.parse(file)).getMessage().contains("no <inertial>"));
    }
}
