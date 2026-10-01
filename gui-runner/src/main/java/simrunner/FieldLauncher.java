package simrunner;

import javax.swing.*;
import java.awt.*;
import java.nio.file.*;
import java.util.ArrayList;

/** Separate pre-run Swing process; GLFW runs in its own macOS first-thread process. */
public final class FieldLauncher {
    public static void main(String[] args) throws Exception {
        if(args.length<2||args.length>3)throw new IllegalArgumentException("Usage: FieldLauncher <projectDir> <OpMode> [prepared field.json]");
        Path project=Path.of(args[0]).toAbsolutePath();var cfg=SimConfig.load(project);
        if(args.length==3)cfg.field=cfg.field.withSource(Path.of(args[2]).toAbsolutePath().toString());
        SwingUtilities.invokeLater(()->show(project,args[1],cfg));
    }
    private static void show(Path project,String opMode,SimConfig config) {
        JFrame window=new JFrame("FTC Forge — select field");window.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
        JPanel panel=new JPanel(new GridLayout(0,1,8,8));panel.setBorder(BorderFactory.createEmptyBorder(16,16,16,16));window.add(panel);
        panel.add(new JLabel("Robot project: "+project.getFileName()+" | "+opMode));
        JButton models=new JButton("Guided model and scene setup…");panel.add(models);models.addActionListener(e->{try{new ProcessBuilder(Path.of(System.getProperty("java.home"),"bin/java").toString(),"-cp",System.getProperty("java.class.path"),GuidedSetupWizard.class.getName(),Path.of(".local/models").toAbsolutePath().toString(),project.toString()).inheritIO().start();}catch(Exception error){JOptionPane.showMessageDialog(window,error.getMessage());}});
        JComboBox<String> source=new JComboBox<>(new String[]{"Generic field","Imported CAD field"});source.setSelectedIndex(config.field.source().equals("imported")?1:0);panel.add(source);
        JTextField packagePath=new JTextField(config.field.packagePath()==null?"":project.resolve(config.field.packagePath()).toString(),45);panel.add(packagePath);
        JButton browse=new JButton("Choose saved field profile.json or prepared field.json…");panel.add(browse);
        JComboBox<String> mode=new JComboBox<>(new String[]{"Field only","Field + game pieces"});mode.setSelectedIndex(config.field.hasPieces()?1:0);panel.add(mode);
        JComboBox<String> pieces=new JComboBox<>(new String[]{"BIOBUZZ balls","Practice torus (generic field)"});pieces.setSelectedIndex(config.field.pieceSet().equals("torus")?1:0);panel.add(pieces);
        JLabel status=new JLabel("No import selected: Generic field is available. All geometry uses meters.");panel.add(status);
        Runnable details=()->{
            pieces.setEnabled(source.getSelectedIndex()==0 && mode.getSelectedIndex()==1);
            if(source.getSelectedIndex()==1 && !packagePath.getText().isBlank())try {
                Path selected=Path.of(packagePath.getText());if(selected.getFileName().toString().equals("profile.json")){var profile=new ModelProfile(selected,true);status.setText(profile.name+" | saved reviewed revision | meters");}else {var field=new FieldPackage(selected);status.setText(String.format("%s | %.5f × %.5f m | practice layout",field.name,field.halfExtents.x*2,field.halfExtents.z*2));}
            }catch(Exception e){status.setText(e.getMessage());}
        };
        source.addActionListener(e->details.run());mode.addActionListener(e->details.run());details.run();
        browse.addActionListener(e->{
            JFileChooser chooser=new JFileChooser();chooser.setDialogTitle("Select BIOBUZZ CAD or prepared field.json");
            if(chooser.showOpenDialog(window)!=JFileChooser.APPROVE_OPTION)return;
            Path selected=chooser.getSelectedFile().toPath().toAbsolutePath();
            if(selected.getFileName().toString().equals("field.json")||selected.getFileName().toString().equals("profile.json")){packagePath.setText(selected.toString());source.setSelectedIndex(1);details.run();return;}
            JOptionPane.showMessageDialog(window,"Use Prepare models to import CAD and review its collision settings, then choose the saved profile.json.");
        });
        for(String action:new String[]{"Preview robot + field","Review collisions","Run OpMode","Run saved scene"}) {
            JButton button=new JButton(action);panel.add(button);button.addActionListener(e->{
                try {
                    String selected=source.getSelectedIndex()==0?"generic":packagePath.getText();
                    if(selected.isBlank())throw new IllegalArgumentException("Import or choose a prepared field first");
                    if(!selected.equals("generic")){if(Path.of(selected).getFileName().toString().equals("profile.json"))new ModelProfile(Path.of(selected),true);else new FieldPackage(Path.of(selected));}
                    var args=new ArrayList<String>();args.add(Path.of(System.getProperty("java.home"),"bin/java").toString());
                    if(System.getProperty("os.name").contains("Mac"))args.add("-XstartOnFirstThread");
                    args.add("-Djava.awt.headless=true");args.add("-Xmx2g");args.add("-cp");args.add(System.getProperty("java.class.path"));args.add(SimulatorApp.class.getName());args.add(project.toString());args.add(opMode);
                    if(!action.equals("Run saved scene")){args.add("--field");args.add(selected);args.add("--mode");args.add(mode.getSelectedIndex()==0?"field-only":"game-pieces");
                    args.add("--piece-set");args.add(source.getSelectedIndex()==1 || pieces.getSelectedIndex()==0?"biobuzz":"torus");}
                    if(action.startsWith("Preview"))args.add("--preview");
                    if(action.equals("Review collisions"))args.add("--collision-review");
                    new ProcessBuilder(args).inheritIO().start();
                }catch(Exception error){JOptionPane.showMessageDialog(window,error.getMessage(),"Cannot start",JOptionPane.ERROR_MESSAGE);}
            });
        }
        window.pack();window.setLocationRelativeTo(null);window.setVisible(true);
    }
}
