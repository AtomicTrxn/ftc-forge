package simrunner;

import java.awt.*;
import java.util.*;
import java.util.List;
import javax.swing.*;
import javax.swing.table.DefaultTableModel;

/** Explicit review, selectable correction paths and retest handoff over a verified snapshot. */
final class RobotMotionReviewDialog {
    record Decision(String action,Map<String,String> choices,Map<String,String> notes,
                    RobotMotionReview.Row row,RobotMotionReview.Correction correction,String part) { }
    static Decision show(Frame owner,RobotMotionReview.Snapshot snapshot) {
        var dialog=new JDialog(owner,"Review robot movements",true);dialog.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
        var panel=new JPanel(new BorderLayout(8,8));panel.setBorder(BorderFactory.createEmptyBorder(12,12,12,12));
        var instructions=text(snapshot.summary(),4);panel.add(instructions,BorderLayout.NORTH);
        var dirty=new HashSet<Integer>();
        var model=new DefaultTableModel(new String[]{"Movement","Native outcome","Review status","Your assessment","Optional note"},0){
            public boolean isCellEditable(int r,int c){return c>=3&&snapshot.rows().get(r).reviewable();}
            public void setValueAt(Object value,int r,int c){super.setValueAt(value,r,c);if(c>=3)dirty.add(r);}
        };
        for(var row:snapshot.rows())model.addRow(new Object[]{row.observation().getOrDefault("action",row.item().label()),row.observation().getOrDefault("outcome","Not run"),row.state(),row.assessment(),row.note()});
        var table=new JTable(model);table.setRowHeight(29);table.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
        table.getColumnModel().getColumn(3).setCellEditor(new DefaultCellEditor(new JComboBox<>(RobotMotionReview.ASSESSMENTS.toArray(String[]::new))));
        int[] widths={250,150,260,140,180};for(int i=0;i<widths.length;i++)table.getColumnModel().getColumn(i).setPreferredWidth(widths[i]);
        panel.add(new JScrollPane(table));var bottom=new JPanel(new BorderLayout(8,8));var details=text("Select a movement to inspect actual travel and correction instructions.",5);bottom.add(new JScrollPane(details));
        table.getSelectionModel().addListSelectionListener(e->{int r=table.getSelectedRow();if(r<0)return;var row=snapshot.rows().get(r);details.setText(row.item().label()+"\n"+RobotMotionReview.travel(row.observation())+RobotMotionReview.contactSummary(row.observation())+"\n"+row.observation().getOrDefault("detail",row.state())+"\n"+(row.reviewable()?"Choose an assessment, then Save reviews. A target does not mean it was reached.":"Run a finished retest before saving this assessment.")+"\nRetest: "+(row.item().part().isEmpty()?"only this drivetrain movement":"both targets of this mechanism, with other motors stopped")+". Completed demos are replaced; close a running demo with Esc.");});
        var buttons=new JPanel(new FlowLayout(FlowLayout.RIGHT));Decision[] result=new Decision[1];
        java.util.function.BiConsumer<String,Boolean> choose=(action,selection)->{
            try {
                if(table.isEditing())table.getCellEditor().stopCellEditing();int r=table.getSelectedRow();if(selection&&r<0)throw new IllegalArgumentException("Select the movement to correct or retest.");
                var choices=new LinkedHashMap<String,String>();var notes=new LinkedHashMap<String,String>();
                for(int i:dirty){var row=snapshot.rows().get(i);String assessment=model.getValueAt(i,3).toString();if(assessment.equals("Not reviewed"))continue;choices.put(row.item().id(),assessment);notes.put(row.item().id(),model.getValueAt(i,4).toString());}
                RobotMotionReview.assess(snapshot,choices,notes);
                var row=r<0?null:snapshot.rows().get(r);RobotMotionReview.Correction correction=null;String part=row==null?"":row.item().part();
                if(action.equals("correct")) {
                    row=new RobotMotionReview.Row(row.item(),row.observation(),row.runId(),model.getValueAt(r,3).toString(),row.state(),row.reviewable(),model.getValueAt(r,4).toString());
                    var fixes=new JComboBox<>(RobotMotionReview.Correction.values());fixes.setSelectedItem(RobotMotionReview.recommended(row));
                    var candidates=parts(snapshot,row);var parts=new JComboBox<>(candidates.toArray(String[]::new));
                    var help=text(RobotMotionReview.advice(row,(RobotMotionReview.Correction)fixes.getSelectedItem()),5);var selected=row;
                    fixes.addActionListener(e->help.setText(RobotMotionReview.advice(selected,(RobotMotionReview.Correction)fixes.getSelectedItem())));
                    var form=new JPanel(new BorderLayout(8,8));var controls=new JPanel(new GridLayout(0,2,8,8));controls.add(new JLabel("Correction path:"));controls.add(fixes);controls.add(new JLabel("Part to focus:"));controls.add(parts);form.add(controls,BorderLayout.NORTH);form.add(new JScrollPane(help));
                    if(JOptionPane.showConfirmDialog(dialog,form,"Choose a correction path",JOptionPane.OK_CANCEL_OPTION,JOptionPane.PLAIN_MESSAGE)!=JOptionPane.OK_OPTION)return;
                    correction=(RobotMotionReview.Correction)fixes.getSelectedItem();part=Objects.toString(parts.getSelectedItem(),"");
                }
                result[0]=new Decision(action,choices,notes,row,correction,part);dialog.dispose();
            }catch(Exception e){JOptionPane.showMessageDialog(dialog,e.getMessage(),"Movement review needs attention",JOptionPane.ERROR_MESSAGE);}
        };
        var save=new JButton("Save reviews");save.addActionListener(e->choose.accept("save",false));buttons.add(save);
        var correct=new JButton("Guided correction…");correct.addActionListener(e->choose.accept("correct",true));buttons.add(correct);
        var retest=new JButton("Retest selected movement…");retest.addActionListener(e->choose.accept("retest",true));buttons.add(retest);
        var close=new JButton("Close");close.addActionListener(e->dialog.dispose());buttons.add(close);bottom.add(buttons,BorderLayout.SOUTH);panel.add(bottom,BorderLayout.SOUTH);
        dialog.setContentPane(panel);dialog.setSize(1080,670);dialog.setMinimumSize(new Dimension(900,580));dialog.setLocationRelativeTo(owner);if(!snapshot.rows().isEmpty())table.setRowSelectionInterval(0,0);dialog.setVisible(true);return result[0];
    }
    static List<String> parts(RobotMotionReview.Snapshot snapshot,RobotMotionReview.Row row) {
        if(!row.item().part().isEmpty())return List.of(row.item().part());
        var parts=new TreeSet<String>();var urdf=snapshot.setup().urdf();
        for(var tx:urdf.transmissions.values())if(tx.actuators().stream().anyMatch(a->snapshot.setup().plan().driveNames.contains(a.name())))parts.add(urdf.joints.get(tx.joint()).child());
        return List.copyOf(parts);
    }
    private static JTextArea text(String value,int rows){var t=new JTextArea(value,rows,75);t.setEditable(false);t.setLineWrap(true);t.setWrapStyleWord(true);return t;}
}
