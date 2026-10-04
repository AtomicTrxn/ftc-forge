package simrunner;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import java.nio.file.*;
import java.util.*;
import simcore.MiniJson;
import static org.junit.jupiter.api.Assertions.*;

class SimulatorValidationTest {
    @TempDir Path tmp;
    @Test void productionImporterAndNativeScenarioSuiteProduceCompleteEvidence()throws Exception {
        var report=SimulatorValidation.suite(tmp);report.write(tmp);
        assertEquals(12,report.cases.size());assertTrue(report.passed(),Files.readString(tmp.resolve("report.md")));
        assertEquals(12,report.cases.stream().map(c->c.get("id")).distinct().count());
        var data=MiniJson.parseObject(Files.readString(tmp.resolve("report.json")));assertEquals(true,data.get("passed"));assertTrue(data.get("scope").toString().contains("no measured"));
        assertTrue(Files.exists(tmp.resolve("differential/robot.zip")));assertTrue(Files.exists(tmp.resolve("stl/millimeter-stl.zip")));
    }
    @Test void brokenExpectationAndImportFailureStayFailedWhileLaterScenariosRun()throws Exception {
        var r=new ValidationReport();r.check("wrong-direction","Positive movement",()->{ValidationReport.require(-.1>0,"Wrong direction: -0.1 m");return Map.of();});
        r.check("bad-import","Valid package",()->{throw new IllegalArgumentException("Malformed CAD");});r.check("next","Still runs",()->Map.of("completed",true));r.write(tmp);
        assertFalse(r.passed());assertEquals(List.of("fail","fail","pass"),r.cases.stream().map(c->c.get("status")).toList());assertTrue(Files.readString(tmp.resolve("report.md")).contains("Wrong direction"));
        assertThrows(IllegalArgumentException.class,()->r.check("next","Duplicate",Map::of));assertFalse(new ValidationReport().passed());
    }
    @Test void nativeNoMotionCannotPassTheConfiguredMovementCheck()throws Exception {
        com.jme3.system.NativeLibraryLoader.loadNativeLibrary("bulletjme",true);
        var c=SyntheticRobots.importRobot(tmp,true,false,false,SyntheticRobots.Dimensions.standard());var setup=c.motionSetup();
        for(var m:setup.hardware().getAll(simcore.SimDcMotorEx.class)){m.setTargetPosition(0);m.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);}
        assertThrows(IllegalStateException.class,()->SimulatorValidation.movements(setup,4));
    }
    @Test void bareStlRequiresExporterPreparationRatherThanInventingAssemblyOrBindings()throws Exception {
        var c=new GuidedSetupController(tmp.resolve("library"),null);c.session.start("robot",null);
        var packagePath=SyntheticRobots.zip(tmp.resolve("bare.zip"),"body.stl",SyntheticRobots.stlBox());
        var error=assertThrows(IllegalArgumentException.class,()->c.load("robot","fresh",packagePath,null));assertTrue(error.getMessage().contains("Expected one URDF"));
    }
    @Test void profileCommandIsReadOnlyAndFailedRunStillWritesEvidence()throws Exception {
        var c=SyntheticRobots.importRobot(tmp.resolve("model"),true,false,false,SyntheticRobots.Dimensions.standard());Path profile=c.session.modelPath("robot");
        String original=Files.readString(profile),receipt=Files.readString(profile.getParent().resolve("receipt.json"));Path output=tmp.resolve("checks");
        SimulatorValidation.main(new String[]{"--profile",profile.toString(),"--output",output.toString()});assertEquals(original,Files.readString(profile));assertEquals(receipt,Files.readString(profile.getParent().resolve("receipt.json")));
        assertThrows(IllegalArgumentException.class,()->SimulatorValidation.main(new String[]{"--profile",profile.toString(),"--output",profile.getParent().toString()}));
        var p=c.session.profile("robot");FieldPackage.map(FieldPackage.map(FieldPackage.map(p.get("entities")).get("wheel0")).get("settings")).put("actuators",List.of());c.update("robot",p);c.compile("robot");
        assertThrows(IllegalStateException.class,()->SimulatorValidation.main(new String[]{"--profile",profile.toString(),"--output",tmp.resolve("failed").toString()}));
        try(var files=Files.walk(tmp.resolve("failed"))){var report=files.filter(f->f.getFileName().toString().equals("report.json")).findFirst().orElseThrow();assertEquals(false,MiniJson.parseObject(Files.readString(report)).get("passed"));}
    }
}
