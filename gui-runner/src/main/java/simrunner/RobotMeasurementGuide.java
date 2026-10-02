package simrunner;

import java.awt.*;
import java.util.*;
import java.util.List;
import javax.swing.*;
import javax.swing.table.DefaultTableModel;
import simcore.RobotUrdf;

/** Optional, instructional manual-measurement form over the existing profile workflow. */
final class RobotMeasurementGuide {
    record Snapshot(double massKg, boolean differential, double diameterM, double trackM,
                    double wheelbaseM, String weightSource, String driveSource,
                    String motors, List<List<String>> bindings) { }
    record Result(RobotMeasurements.Entry entry, boolean openBindings) { }
    static Snapshot read(GuidedSetupController controller) throws Exception {
        var model=controller.session.checked("robot");
        var urdf=RobotUrdf.parse(model.artifact("robot"));
        var runtime=FieldPackage.map(model.data.get("runtime"));
        boolean differential=runtime.containsKey("drive");
        double diameter,track,wheelbase=0;
        String driveSource,motors;
        if (differential) {
            var drive=DifferentialDriveConfig.parse(FieldPackage.map(runtime.get("drive")));
            diameter=2*drive.wheelRadiusM();track=drive.trackWidthM();
            driveSource="Current differential settings. Check the real tread and wheel centers before marking them measured.";
            motors="Left: "+drive.leftMotor()+" (shaft sign "+(int)drive.leftShaftSign()+") · Right: "+drive.rightMotor()+" (shaft sign "+(int)drive.rightShaftSign()+").";
            if(runtime.containsKey("tires"))motors+=" Tire mode requires wheel joints coupled 1:1 to their side motor output shaft.";
        } else {
            var geometry=runtime.containsKey("drive_geometry")?DriveGeometry.parse(FieldPackage.map(runtime.get("drive_geometry"))):null;
            var dimensions=DriveGeometry.resolve(urdf,geometry);
            diameter=2*dimensions.wheelRadii().get(0);track=dimensions.trackWidthM();wheelbase=dimensions.wheelbaseM();
            driveSource=geometry!=null?"Current explicit Mecanum dimensions.":"Current dimensions use CAD where available, then simulator defaults. This guide applies one diameter to all four driven wheels.";
            motors="Mecanum slots: "+String.join(", ",DriveGeometry.MOTORS)+". Bind each continuous wheel joint to its matching slot in Parts and movement.";
        }
        List<String> names;boolean hardwareUnavailable=false;
        try{names=controller.hardware(null);}catch(Exception e){names=List.of();hardwareUnavailable=true;motors+=" Project hardware is unavailable; choose a valid project in Parts and movement. Manual measurements can still be saved.";}
        var bindings=new ArrayList<List<String>>();
        for(var part : FieldPackage.map(model.data.get("entities")).entrySet()) {
            var settings=FieldPackage.map(FieldPackage.map(part.getValue()).get("settings"));
            if(settings.get("joint")==null)continue;
            var joint=FieldPackage.map(settings.get("joint"));
            if("fixed".equals(joint.get("type")))continue;
            var actuators=FieldPackage.maps(settings.get("actuators"));
            var preparedJoint=urdf.joints.get(joint.get("name").toString());
            if(actuators.isEmpty())bindings.add(List.of(part.getKey(),joint.get("name").toString(),preparedJoint!=null&&preparedJoint.mimic()!=null?"Passive follower":"Unbound","—",preparedJoint!=null&&preparedJoint.mimic()!=null?"Coupled to "+preparedJoint.mimic():"Confirm passive or bind in Parts"));
            for(var actuator:actuators) {
                String name=actuator.get("name").toString();
                bindings.add(List.of(part.getKey(),joint.get("name").toString(),name,
                    actuator.get("mechanicalReduction").toString(),controller.session.project()==null?"Project not selected":hardwareUnavailable?"Hardware check unavailable":names.contains(name)?"Name found in project":"Missing from project"));
            }
        }
        if(bindings.isEmpty())motors+=" No movable joints are configured yet. Use Parts and movement to identify wheel/shaft joints first.";
        double mass=runtime.containsKey("total_mass_kg")?FieldPackage.num(runtime,"total_mass_kg"):urdf.totalMassKg();
        return new Snapshot(mass,differential,diameter,track,wheelbase,
            runtime.containsKey("total_mass_kg")?"Current total-weight override. Confirm it matches your operating robot.":"Current prepared CAD/fallback mass, which may omit the battery and electronics.",
            driveSource,motors,List.copyOf(bindings));
    }

    private static final class GuidePanel extends JPanel implements Scrollable {
        public Dimension getPreferredScrollableViewportSize(){return new Dimension(900,620);}
        public int getScrollableUnitIncrement(Rectangle visible,int orientation,int direction){return 22;}
        public int getScrollableBlockIncrement(Rectangle visible,int orientation,int direction){return visible.height-22;}
        public boolean getScrollableTracksViewportWidth(){return true;}
        public boolean getScrollableTracksViewportHeight(){return false;}
    }
    private final JPanel panel=new GuidePanel();
    private final JCheckBox useMass=new JCheckBox("Use my measured operating weight");
    private final JCheckBox useDimensions=new JCheckBox("Use my measured drive dimensions");
    private final JTextField mass=new JTextField(9),diameter=new JTextField(9),track=new JTextField(9),wheelbase=new JTextField(9);
    private final JComboBox<RobotMeasurements.MassUnit> massUnit=new JComboBox<>(RobotMeasurements.MassUnit.values());
    private final JComboBox<RobotMeasurements.LengthUnit> lengthUnit=new JComboBox<>(RobotMeasurements.LengthUnit.values());
    private RobotMeasurements.MassUnit oldMassUnit=RobotMeasurements.MassUnit.KG;
    private RobotMeasurements.LengthUnit oldLengthUnit=RobotMeasurements.LengthUnit.MM;
    private final boolean differential;

    private RobotMeasurementGuide(Snapshot snapshot) {
        differential=snapshot.differential();panel.setLayout(new BoxLayout(panel,BoxLayout.Y_AXIS));
        panel.setBorder(BorderFactory.createEmptyBorder(10,10,10,10));
        section("1. Weigh the operating robot", "Install the battery, electronics and mechanisms you normally run. Remove game pieces and weigh the whole robot on a scale. Enter the total weight; the simulator scales the prepared masses and inertias proportionally.");
        panel.add(text(snapshot.weightSource()));panel.add(useMass);
        mass.setText(format(snapshot.massKg()));panel.add(row("Operating weight",mass,massUnit));
        section("2. Measure the wheels and spacing", "Measure across the outer tread through the axle center for diameter, not radius. Measure track width between left and right wheel centers, not the outer robot edges."+(differential?" For a six-wheel/skid drive, use the center axle spacing as a geometric starting point; effective turning track still needs drive recordings.":" Measure wheelbase between front and rear axle centers. Confirm all four driven wheels use the same diameter."));
        panel.add(text(snapshot.driveSource()));panel.add(useDimensions);
        diameter.setText(format(snapshot.diameterM()*1000));track.setText(format(snapshot.trackM()*1000));wheelbase.setText(format(snapshot.wheelbaseM()*1000));
        panel.add(row("Length units",lengthUnit));panel.add(row("Wheel diameter",diameter));panel.add(row("Track width (center to center)",track));
        if(!differential)panel.add(row("Wheelbase (front to rear centers)",wheelbase));
        panel.add(text("These values change drive calculations. CAD visuals and collision shapes retain their imported size; inspect tread/floor alignment again after saving."));
        section("3. Verify motor bindings", "Compare each motor name with your project's Robot Configuration and the real powered shaft. Check motor-to-joint gearing: signed motor output-shaft turns per joint turn. TeamCode Direction and a drive's physical shaft sign are separate settings. Recording-based response calibration is a later step.");
        panel.add(text(snapshot.motors()));
        var tableModel=new DefaultTableModel(new String[]{"Part","Joint","Motor / servo","Signed ratio","Project name check"},0){public boolean isCellEditable(int r,int c){return false;}};
        for(var binding:snapshot.bindings())tableModel.addRow(binding.toArray());
        var table=new JTable(tableModel);table.setRowHeight(26);int[] widths={130,170,150,85,260};for(int i=0;i<widths.length;i++)table.getColumnModel().getColumn(i).setPreferredWidth(widths[i]);var scroll=new JScrollPane(table);scroll.setPreferredSize(new Dimension(830,120));scroll.setMaximumSize(new Dimension(Integer.MAX_VALUE,140));scroll.setAlignmentX(Component.LEFT_ALIGNMENT);panel.add(scroll);
        panel.add(text("A matching name does not verify wiring or rotation. Select Save and open motor bindings to map joints and inspect gearing/direction there. Without a project, measurements can still be saved and hardware mapping deferred."));
        mass.getAccessibleContext().setAccessibleName("Measured operating weight");diameter.getAccessibleContext().setAccessibleName("Measured wheel diameter");track.getAccessibleContext().setAccessibleName("Measured track width");wheelbase.getAccessibleContext().setAccessibleName("Measured wheelbase");
        Runnable enable=()->{mass.setEnabled(useMass.isSelected());massUnit.setEnabled(useMass.isSelected());for(var c:List.of(diameter,track,wheelbase))c.setEnabled(useDimensions.isSelected());lengthUnit.setEnabled(useDimensions.isSelected());};
        useMass.addActionListener(e->enable.run());useDimensions.addActionListener(e->enable.run());enable.run();
        massUnit.addActionListener(e->{var next=(RobotMeasurements.MassUnit)massUnit.getSelectedItem();convert(mass,oldMassUnit.kg/next.kg);oldMassUnit=next;});
        lengthUnit.addActionListener(e->{var next=(RobotMeasurements.LengthUnit)lengthUnit.getSelectedItem();for(var field:List.of(diameter,track,wheelbase))convert(field,oldLengthUnit.meters/next.meters);oldLengthUnit=next;});
    }
    private static String format(double value){return String.format(Locale.ROOT,"%.6f",value).replaceAll("0+$","").replaceAll("\\.$","");}
    private static void convert(JTextField field,double scale){try{field.setText(format(Double.parseDouble(field.getText().strip())*scale));}catch(NumberFormatException ignored){/* Retain invalid text for an actionable validation error. */}}
    private void section(String title,String instruction){var heading=new JLabel(title);heading.setFont(heading.getFont().deriveFont(Font.BOLD,16f));heading.setBorder(BorderFactory.createEmptyBorder(12,0,6,0));heading.setAlignmentX(Component.LEFT_ALIGNMENT);panel.add(heading);panel.add(text(instruction));}
    private static JTextArea text(String value){var t=new JTextArea(value);t.setEditable(false);t.setLineWrap(true);t.setWrapStyleWord(true);t.setOpaque(false);t.setColumns(72);t.setRows(Math.max(1,(value.length()+94)/95));t.setMinimumSize(new Dimension(0,t.getPreferredSize().height));t.setMaximumSize(new Dimension(Integer.MAX_VALUE,t.getPreferredSize().height));t.setAlignmentX(Component.LEFT_ALIGNMENT);return t;}
    private static JPanel row(String label,Component...controls){var row=new JPanel(new FlowLayout(FlowLayout.LEFT));row.setAlignmentX(Component.LEFT_ALIGNMENT);row.add(new JLabel(label));for(var c:controls)row.add(c);row.setMaximumSize(new Dimension(Integer.MAX_VALUE,row.getPreferredSize().height));return row;}
    private double number(JTextField field,String label,double scale){try{double value=Double.parseDouble(field.getText().strip())*scale;if(!Double.isFinite(value)||value<=0)throw new NumberFormatException();return value;}catch(NumberFormatException e){throw new IllegalArgumentException(label+" must be a positive, finite number.");}}
    private RobotMeasurements.Entry entry(){
        double kg=((RobotMeasurements.MassUnit)massUnit.getSelectedItem()).kg,meters=((RobotMeasurements.LengthUnit)lengthUnit.getSelectedItem()).meters;
        return new RobotMeasurements.Entry(useMass.isSelected()?number(mass,"Operating weight",kg):null,
            useDimensions.isSelected()?number(diameter,"Wheel diameter",meters):null,
            useDimensions.isSelected()?number(track,"Track width",meters):null,
            useDimensions.isSelected()&&!differential?number(wheelbase,"Wheelbase",meters):null);
    }
    static Result show(Component parent,Snapshot snapshot) {
        var guide=new RobotMeasurementGuide(snapshot);var scroll=new JScrollPane(guide.panel);scroll.setPreferredSize(new Dimension(900,620));scroll.getVerticalScrollBar().setUnitIncrement(22);
        var options=new String[]{"Save measurements","Save and open motor bindings","Cancel"};
        while(true){SwingUtilities.invokeLater(()->scroll.getVerticalScrollBar().setValue(0));int answer=JOptionPane.showOptionDialog(parent,scroll,"Robot measurements — calibration step 1",JOptionPane.DEFAULT_OPTION,JOptionPane.PLAIN_MESSAGE,null,options,options[0]);
            if(answer<0||answer==2)return null;
            try{var entry=guide.entry();if(answer==0&&entry.empty())throw new IllegalArgumentException("Select a measured section to save, or open motor bindings while keeping current dimensions.");return new Result(entry,answer==1);}
            catch(IllegalArgumentException e){JOptionPane.showMessageDialog(parent,e.getMessage(),"Check the measurement",JOptionPane.ERROR_MESSAGE);}
        }
    }
}
