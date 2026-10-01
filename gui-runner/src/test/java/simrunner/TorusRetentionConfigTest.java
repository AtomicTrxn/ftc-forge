package simrunner;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import simcore.MiniJson;
import java.nio.file.*;
import java.util.*;
import static org.junit.jupiter.api.Assertions.*;

class TorusRetentionConfigTest {
    @TempDir Path temp;
    private static final String JSON="""
            {"seat_xyz_m":[0.23,0.01,0.145],"exit_xyz_m":[0.33,0,0.045],
             "contact_dwell_s":0.015,"travel_m_per_rad":0.012,"stiffness_n_per_m":150,
             "damping_ns_per_m":4,"max_force_n":8,"break_distance_m":0.12,
             "reflected_shaft_inertia_kg_m2":0.0015,"tilt_rad":1.0471975512,
             "angular_stiffness_nm_per_rad":0.15,"angular_damping_nms_per_rad":0.012,
             "max_torque_nm":0.08,"angular_travel_rad_per_rad":0.15,
             "contact_stiffness_n_per_m":100,"contact_damping_ns_per_m":1}
            """;
    private static Map<String,Object> values(){return new LinkedHashMap<>(MiniJson.parseObject(JSON));}
    @Test void coordinatesAreUrdfAndReturnedVectorsCannotMutateConfiguration() {
        var spec=TorusRetentionConfig.parse(values());
        assertEquals(.145,spec.seat().y,1e-6);assertEquals(-.01,spec.seat().z,1e-6);
        spec.seat().zero();assertEquals(.23,spec.seat().x,1e-6);
    }
    @Test void rejectsMissingNonfiniteOrUnboundedPhysicsAndWrongExitDirection() {
        for(var change:List.of(Map.entry("max_force_n",Double.NaN),Map.entry("max_force_n",21d),
                Map.entry("contact_stiffness_n_per_m",0d),Map.entry("contact_stiffness_n_per_m",1e100),Map.entry("damping_ns_per_m",1e-100),Map.entry("reflected_shaft_inertia_kg_m2",-1d),
                Map.entry("tilt_rad",2d),Map.entry("max_torque_nm",3d))) {
            var v=values();v.put(change.getKey(),change.getValue());assertThrows(IllegalArgumentException.class,()->TorusRetentionConfig.parse(v));
        }
        var missing=values();missing.remove("contact_dwell_s");assertThrows(IllegalArgumentException.class,()->TorusRetentionConfig.parse(missing));
        var wrong=values();wrong.put("exit_xyz_m",List.of(.1,0,.04));assertThrows(IllegalArgumentException.class,()->TorusRetentionConfig.parse(wrong));
    }
    @Test void profileRequiresSeatInsideEnvelopeAndExitOutsideAndValidatesSpawn() throws Exception {
        String base="""
            {"urdf":"robot.urdf","intake":{"motor":"intake","shaft_sign":1,"min_speed_rad_s":1,
            "point_xyz_m":[0.245,0,0.065],"capture_radius_m":0.12},
            "flexible_intake":{"links":["flap"],"segments_per_arm":3,"flex_mass_fraction":0.9,
            "width_m":0.0116118,"arm_length_m":0.0507965,"thickness_m":0.003,
            "stiffness_nm_per_rad":0.003,"damping_ratio":0.5,"max_bend_rad":1.2,"friction":1,
            "contact_stiffness_n_per_m":500,"contact_damping_ns_per_m":0.3,
            "containment_min_xyz_m":[0.18,-0.08,0.08],"containment_max_xyz_m":[0.28,0.08,0.20]},
            "game_piece_start_xyz_m":[0.42,0,0.034],"torus_retention":
            """+JSON+"}";
        Files.writeString(temp.resolve("sim.config"),base);
        assertEquals(.034,SimConfig.load(temp).gamePieceStart.y,1e-6);
        for(String bad:List.of(base.replace("0.23,0.01,0.145","0.5,0.01,0.145"),
                base.replace("0.33,0,0.045","0.27,0,0.045"),base.replace("0.42,0,0.034","0.42,0,-0.034"))) {
            Files.writeString(temp.resolve("sim.config"),bad);assertThrows(IllegalArgumentException.class,()->SimConfig.load(temp));
        }
    }
    @Test void cannotEnableRetentionWithoutFlexiblePaddles() throws Exception {
        Files.writeString(temp.resolve("sim.config"),"{\"torus_retention\":"+JSON+"}");
        assertThrows(IllegalArgumentException.class,()->SimConfig.load(temp));
    }
}
