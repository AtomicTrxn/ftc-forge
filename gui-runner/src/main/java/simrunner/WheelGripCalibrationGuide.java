package simrunner;

import physics.calibration.GripCalibrator;
import physics.calibration.GripCalibrator.*;
import java.awt.*;
import java.nio.file.*;
import java.util.*;
import java.util.List;
import javax.swing.*;
import javax.swing.table.DefaultTableModel;

/** One wheel's physical force readings, fit preview, and explicit application. */
final class WheelGripCalibrationGuide {
    record Snapshot(String joint,String label,double currentCoefficient,Map<String,Object> evidence) {}
    static Snapshot read(GuidedSetupController controller,String joint)throws Exception {
        String label=controller.wheelGripChoices().get(joint);if(label==null)throw new IllegalArgumentException("Choose a configured continuous drive wheel.");
        var profile=controller.session.profile("robot");Object contacts=FieldPackage.map(profile.get("runtime")).get("drive_contacts");
        if(contacts==null||!DriveContactConfig.parse(FieldPackage.map(contacts),false).enabled())throw new IllegalArgumentException("Choose native contacts in Wheel support model before measuring wheel grip.");
        var cfg=FieldPackage.map(contacts);var parsed=DriveContactConfig.parse(cfg,false);
        var root=FieldPackage.map(FieldPackage.map(FieldPackage.map(profile.get("entities")).get(FieldPackage.map(profile.get("source")).get("root"))).get("settings"));
        double inherited=FieldPackage.num(Boolean.TRUE.equals(root.get("material_override"))?root:FieldPackage.map(profile.get("parameters")),"friction");
        var evidence=FieldPackage.maps(cfg.getOrDefault("wheel_friction",List.of())).stream().filter(g->joint.equals(g.get("joint"))&&g.containsKey("measurement")).map(g->FieldPackage.map(g.get("measurement"))).findFirst().orElse(Map.of());
        return new Snapshot(joint,label,parsed.friction(joint,inherited),evidence);
    }
    private final Snapshot snapshot;
    private static final class GuidePanel extends JPanel implements Scrollable {
        public Dimension getPreferredScrollableViewportSize(){return new Dimension(980,680);}
        public int getScrollableUnitIncrement(Rectangle visible,int orientation,int direction){return 24;}
        public int getScrollableBlockIncrement(Rectangle visible,int orientation,int direction){return visible.height-24;}
        public boolean getScrollableTracksViewportWidth(){return true;}
        public boolean getScrollableTracksViewportHeight(){return false;}
    }
    private final JPanel panel=new GuidePanel();
    private final JTextField surface=new JTextField(24);
    private final JSpinner surfaceFriction=new JSpinner(new SpinnerNumberModel(.6,.000001,2.,.05));
    private final JComboBox<WheelGripMeasurement.LoadUnit> loadUnit=new JComboBox<>(WheelGripMeasurement.LoadUnit.values());
    private final JComboBox<WheelGripMeasurement.ForceUnit> forceUnit=new JComboBox<>(WheelGripMeasurement.ForceUnit.values());
    private WheelGripMeasurement.LoadUnit previousLoad=WheelGripMeasurement.LoadUnit.N;
    private WheelGripMeasurement.ForceUnit previousForce=WheelGripMeasurement.ForceUnit.N;
    private final DefaultTableModel rows=new DefaultTableModel(new String[]{"Physical trial ID","Set","Normal load (chosen units)","Sliding-onset force (chosen units)"},0);
    private final JTable table=new JTable(rows);
    private final JSpinner fitTrials=new JSpinner(new SpinnerNumberModel(3,3,100,1)),validationTrials=new JSpinner(new SpinnerNumberModel(2,2,100,1));
    private final JSpinner variation=new JSpinner(new SpinnerNumberModel(.2,.05,1.,.05)),error=new JSpinner(new SpinnerNumberModel(.15,.01,.5,.01)),trialError=new JSpinner(new SpinnerNumberModel(.25,.01,.75,.01));
    private final JCheckBox physical=new JCheckBox("These readings come from physical tread/surface tests, with whole trials reserved for validation.");
    private final JLabel status=new JLabel("Enter your readings or import CSV. No example measurements are supplied.");
    private WheelGripCalibrationGuide(Snapshot snapshot) {
        this.snapshot=snapshot;panel.setLayout(new BoxLayout(panel,BoxLayout.Y_AXIS));panel.setBorder(BorderFactory.createEmptyBorder(8,8,8,8));
        for(var spinner:List.of(surfaceFriction,fitTrials,validationTrials,variation,error,trialError))
            ((JSpinner.DefaultEditor)spinner.getEditor()).getTextField().setColumns(6);
        section("1. Measure one wheel and reference surface", "Selected: "+snapshot.joint()+" · "+snapshot.label()+". Current coefficient: "+snapshot.currentCoefficient()+". Use the actual tread on a flat reference surface. Restrain wheel rotation and isolate its contact from chassis scraping and other wheels. Measure normal load on this wheel and horizontal force at sliding onset in the intended traction direction. A rolling wheel's drag is a different measurement. Omni/Mecanum direction matters; this scalar test does not identify roller anisotropy.");
        panel.add(row("Reference surface name",surface));panel.add(row("Saved reference surface friction (dimensionless)",surfaceFriction));
        panel.add(text("Effective grip = force / normal load; wheel coefficient = effective grip / surface coefficient. This measures the pair, not two independent materials. Use this same surface coefficient in the simulator. The motion demo uses floor friction 0.6; other field materials may differ. Optional tire static/sliding curves remain an additional limit."));
        section("2. Record fitting and independent validation trials", "Reserve at least two fresh validation trials before fitting; never put the same physical trial in both sets. Use at least three fitting trials at different known load levels. Repeated readings share a trial ID and count once. Keep test direction and surface condition consistent. Blank readings must be filled or removed.");
        panel.add(row("Normal-load units",loadUnit));panel.add(row("Force units",forceUnit));
        table.setRowHeight(28);table.getColumnModel().getColumn(1).setCellEditor(new DefaultCellEditor(new JComboBox<>(new String[]{"fit","validate"})));table.getAccessibleContext().setAccessibleName("Wheel grip physical readings");
        var scroll=new JScrollPane(table);scroll.setPreferredSize(new Dimension(860,180));scroll.setMaximumSize(new Dimension(Integer.MAX_VALUE,200));panel.add(scroll);
        var actions=new JPanel(new GridLayout(0,3,6,6));actions.setAlignmentX(Component.LEFT_ALIGNMENT);button(actions,"Add reading",()->{String id="trial-"+(rows.getRowCount()+1);rows.addRow(new Object[]{id,"fit","",""});});button(actions,"Remove selected reading",()->{int r=table.getSelectedRow();if(r>=0)rows.removeRow(r);});button(actions,"Import readings CSV…",this::importCsv);button(actions,"Save blank CSV template…",()->writeCsv(true));button(actions,"Export entered readings…",()->writeCsv(false));actions.setMaximumSize(new Dimension(Integer.MAX_VALUE,actions.getPreferredSize().height));panel.add(actions);panel.add(status);
        section("3. Review quality criteria and preview the fit", "Normal-load variation is (largest − smallest fitting-trial mean) / largest mean. Errors use each trial equally, including all its readings. The fit uses trial means through the origin. Validation predicts forces in trials excluded from fitting. Tolerances are saved with evidence; relaxed tolerances do not establish better accuracy.");
        panel.add(row("Minimum fitting / validation trials",fitTrials,validationTrials));panel.add(row("Minimum load variation (fraction)",variation));panel.add(row("Maximum relative RMSE / per-trial error",error,trialError));panel.add(physical);
        panel.add(text("A passing fit can be applied only after confirming the source of the readings. Application saves a draft and requires collision review and fresh motion assessment. CAD/settings changes need review; real-world accuracy still depends on the measurement method and representativeness of the reserved trials."));
        surface.getAccessibleContext().setAccessibleName("Reference surface name");surfaceFriction.getAccessibleContext().setAccessibleName("Reference surface friction");
        if(!snapshot.evidence().isEmpty())try{
            var e=snapshot.evidence();surface.setText(FieldPackage.str(e,"reference_surface"));surfaceFriction.setValue(FieldPackage.num(e,"surface_friction"));
            var c=WheelGripMeasurement.criteria(FieldPackage.map(e.get("criteria")));fitTrials.setValue(c.minFitTrials());validationTrials.setValue(c.minValidationTrials());variation.setValue(c.minLoadVariation());error.setValue(c.maxRelativeRmse());trialError.setValue(c.maxTrialRelativeRmse());load(WheelGripMeasurement.readings(e));
            try{var checked=WheelGripMeasurement.check(e,snapshot.currentCoefficient());status.setText("Loaded saved readings; independent validation relative RMSE "+format(checked.result().validationRelativeRmse())+". Refit after changes.");}catch(Exception ex){status.setText("Saved evidence needs refitting: "+ex.getMessage());}
        }catch(Exception ex){blank();status.setText("Saved evidence could not be loaded. Enter new readings or discard it using Wheel grip: "+ex.getMessage());}
        else blank();
        loadUnit.addActionListener(e->{var next=(WheelGripMeasurement.LoadUnit)loadUnit.getSelectedItem();convert(2,previousLoad.newtons/next.newtons);previousLoad=next;});forceUnit.addActionListener(e->{var next=(WheelGripMeasurement.ForceUnit)forceUnit.getSelectedItem();convert(3,previousForce.newtons/next.newtons);previousForce=next;});
    }
    private void blank(){for(int i=0;i<5;i++)rows.addRow(new Object[]{i<3?"fit-"+(i+1):"check-"+(i-2),i<3?"fit":"validate","",""});}
    private void load(List<Reading> readings){rows.setRowCount(0);for(var r:readings)rows.addRow(new Object[]{r.trial(),r.set(),r.normalN()/previousLoad.newtons,r.forceN()/previousForce.newtons});}
    private void commit(){if(table.isEditing()&&!table.getCellEditor().stopCellEditing())throw new IllegalArgumentException("Finish editing the reading first.");}
    private void convert(int column,double factor){commit();for(int r=0;r<rows.getRowCount();r++)try{Object v=rows.getValueAt(r,column);if(v!=null&&!v.toString().isBlank())rows.setValueAt(Double.parseDouble(v.toString())*factor,r,column);}catch(NumberFormatException ignored){/* Keep invalid text for an actionable error before fitting. */}}
    private List<Reading> readings(){commit();var result=new ArrayList<Reading>();for(int i=0;i<rows.getRowCount();i++)try{result.add(new Reading(rows.getValueAt(i,0).toString().strip(),rows.getValueAt(i,1).toString().strip(),Double.parseDouble(rows.getValueAt(i,2).toString())*previousLoad.newtons,Double.parseDouble(rows.getValueAt(i,3).toString())*previousForce.newtons));}catch(RuntimeException e){throw new IllegalArgumentException("Reading "+(i+1)+": fill the trial, set, normal load and force using the chosen units. "+e.getMessage());}return result;}
    private Criteria criteria()throws Exception {for(var s:List.of(surfaceFriction,fitTrials,validationTrials,variation,error,trialError))s.commitEdit();return new Criteria(((Number)fitTrials.getValue()).intValue(),((Number)validationTrials.getValue()).intValue(),number(variation),number(error),number(trialError));}
    private static double number(JSpinner s){return ((Number)s.getValue()).doubleValue();}
    private void importCsv(){var chooser=new JFileChooser();if(chooser.showOpenDialog(panel)!=JFileChooser.APPROVE_OPTION)return;try{var data=WheelGripMeasurement.readCsv(chooser.getSelectedFile().toPath(),snapshot.joint());commit();load(data.readings());status.setText("Loaded "+data.readings().size()+" readings for "+snapshot.joint()+"; "+data.otherWheelRows()+" other-wheel rows omitted. Review surface and criteria before fitting.");}catch(Exception e){failure(e);}}
    private void writeCsv(boolean template){var chooser=new JFileChooser();chooser.setSelectedFile(new java.io.File(template?"wheel-grip-template.csv":"wheel-grip-readings.csv"));if(chooser.showSaveDialog(panel)!=JFileChooser.APPROVE_OPTION)return;Path file=chooser.getSelectedFile().toPath();try{if(Files.exists(file)&&JOptionPane.showConfirmDialog(panel,"Replace existing CSV: "+file.getFileName()+"?","Export readings",JOptionPane.OK_CANCEL_OPTION)!=JOptionPane.OK_OPTION)return;if(template)WheelGripMeasurement.template(file,snapshot.joint(),previousLoad,previousForce);else Files.writeString(file,WheelGripMeasurement.normalizedCsv(snapshot.joint(),readings()));status.setText((template?"Blank template":"Normalized N/N readings")+" saved: "+file.getFileName());}catch(Exception e){failure(e);}}
    private void failure(Exception e){JOptionPane.showMessageDialog(panel,e.getMessage(),"Check wheel measurements",JOptionPane.ERROR_MESSAGE);}
    private void section(String title,String value){var label=new JLabel(title);label.setFont(label.getFont().deriveFont(Font.BOLD,15));label.setBorder(BorderFactory.createEmptyBorder(10,0,5,0));label.setAlignmentX(Component.LEFT_ALIGNMENT);panel.add(label);panel.add(text(value));}
    private static JTextArea text(String value){var t=new JTextArea(value);t.setLineWrap(true);t.setWrapStyleWord(true);t.setEditable(false);t.setOpaque(false);t.setColumns(85);t.setRows((value.length()+109)/110);t.setAlignmentX(Component.LEFT_ALIGNMENT);t.setMaximumSize(new Dimension(Integer.MAX_VALUE,t.getPreferredSize().height));return t;}
    private static JPanel row(String title,Component...controls){var p=new JPanel(new FlowLayout(FlowLayout.LEFT));p.setAlignmentX(Component.LEFT_ALIGNMENT);p.add(new JLabel(title));for(var c:controls)p.add(c);p.setMaximumSize(new Dimension(Integer.MAX_VALUE,p.getPreferredSize().height));return p;}
    private static void button(JPanel p,String title,Runnable action){var b=new JButton(title);b.addActionListener(e->action.run());p.add(b);}
    private static String format(double value){return String.format(Locale.ROOT,"%.4f",value);}
    static String summary(Result r,String surface,double surfaceCoefficient){return (r.accepted()?"Quality checks passed for these readings.":"Quality checks need attention; settings cannot be applied.")+"\nReference: "+surface+" · surface friction "+surfaceCoefficient+"\nEffective pair grip: "+format(r.effectiveMu())+" · fitted wheel coefficient: "+format(r.wheelCoefficient())+"\nIndependent trials: "+r.fitTrials()+" fitting, "+r.validationTrials()+" validation\nFitting force RMSE: "+format(r.fitRmseN())+" N (relative "+format(r.fitRelativeRmse())+")\nValidation force RMSE: "+format(r.validationRmseN())+" N (relative "+format(r.validationRelativeRmse())+")\nWorst per-trial relative error: "+format(r.worstTrialRelativeRmse())+"\n"+String.join("\n",r.issues())+"\n\nThis checks agreement with the supplied trials, not the physical correctness of your measurements. Use the same reference surface setting; review geometry and motion after applying.";}
    static Map<String,Object> show(Component parent,Snapshot snapshot) {
        var guide=new WheelGripCalibrationGuide(snapshot);var scroll=new JScrollPane(guide.panel);scroll.setPreferredSize(new Dimension(980,680));scroll.getVerticalScrollBar().setUnitIncrement(24);
        while(true) {
            SwingUtilities.invokeLater(()->{guide.surface.requestFocusInWindow();scroll.getVerticalScrollBar().setValue(0);});
            int choice=JOptionPane.showOptionDialog(parent,scroll,"Measure wheel grip — "+snapshot.joint(),JOptionPane.DEFAULT_OPTION,JOptionPane.PLAIN_MESSAGE,null,new String[]{"Fit and preview","Cancel"},"Fit and preview");if(choice!=0)return null;
            try {
                var readings=guide.readings();var criteria=guide.criteria();String surface=guide.surface.getText().strip();if(surface.isBlank())throw new IllegalArgumentException("Name the actual reference surface.");double mu=number(guide.surfaceFriction);var result=GripCalibrator.fit(readings,mu,criteria);
                boolean apply=result.accepted()&&guide.physical.isSelected();String message=summary(result,surface,mu)+(result.accepted()&&!apply?"\nConfirm that the readings come from physical trials before applying.":"");
                String[] options=apply?new String[]{"Apply measured settings","Back to readings","Cancel"}:new String[]{"Back to readings","Cancel"};
                int answer=JOptionPane.showOptionDialog(parent,text(message),"Wheel grip fit preview",JOptionPane.DEFAULT_OPTION,JOptionPane.PLAIN_MESSAGE,null,options,options[0]);
                if(apply&&answer==0)return WheelGripMeasurement.evidence(snapshot.joint(),surface,mu,criteria,readings);
                if(answer<0||answer==options.length-1)return null;
            }catch(Exception e){guide.failure(e);}
        }
    }
}
