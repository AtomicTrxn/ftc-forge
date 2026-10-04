package simrunner;

import com.jme3.input.*;
import com.jme3.input.controls.*;
import java.nio.file.Path;
import java.util.function.*;

/** Same tested controls in both renderer entry points; supplier follows demo replay. */
final class PhysicsDiagnosticControls {
    static void bind(InputManager input,Supplier<PhysicsDiagnosticsOverlay> overlay,Path output,Consumer<Exception> failure) {
        input.addMapping("TogglePhysicsDiagnostics",new KeyTrigger(KeyInput.KEY_D));input.addMapping("SavePhysicsDiagnostics",new KeyTrigger(KeyInput.KEY_P));
        input.addListener((ActionListener)(name,pressed,tpf)->{var current=overlay.get();if(!pressed||current==null)return;if(name.equals("TogglePhysicsDiagnostics"))current.setVisible(!current.visible());else try{System.out.println("[DIAGNOSTICS] "+current.save(output));}catch(Exception e){failure.accept(e);}},"TogglePhysicsDiagnostics","SavePhysicsDiagnostics");
    }
}
