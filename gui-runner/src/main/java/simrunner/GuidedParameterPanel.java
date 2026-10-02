package simrunner;

import javax.swing.*;
import javax.swing.table.AbstractTableModel;
import java.awt.BorderLayout;
import java.util.*;
import java.util.List;

/** Focused, friendly parameter controls over the same editable model data. */
final class GuidedParameterPanel extends JPanel {
    private record Row(Object parent,Object key,Object value,String path,String name,String help) { }
    private final List<Row> rows=new ArrayList<>();
    private final Map<String,Object> profile;
    private final GuidedSetupController controller;
    private final String kind;
    private final Runnable changed;
    private final JTable table;
    GuidedParameterPanel(GuidedSetupController controller,String kind,Map<String,Object> profile,Object values,String path,String page,Runnable changed) {
        super(new BorderLayout());this.controller=controller;this.kind=kind;this.profile=profile;this.changed=changed;
        flatten(values,path,page);
        var model=new AbstractTableModel(){
            public int getRowCount(){return rows.size();}public int getColumnCount(){return 4;}
            public String getColumnName(int col){return new String[]{"Setting","Value","Where it came from","What this changes"}[col];}
            public Object getValueAt(int row,int col){var r=rows.get(row);return switch(col){case 0->r.name;case 1->r.value;case 2->origin(r.path);default->r.help;};}
            public boolean isCellEditable(int row,int col){if(col!=1)return false;var r=rows.get(row);if(r.path.startsWith("entities/")&&(base(r.path).contains("friction")||base(r.path).contains("restitution")||base(r.path).equals("material_override"))){String name=r.path.split("/")[1].replace("~1","/").replace("~0","~");try{var owners=FieldPackage.map(controller.session.checked(kind).receipt.get("body_grouping"));if(!name.equals(owners.getOrDefault(name,name)))return false;}catch(Exception ignored){}}return true;}
            @SuppressWarnings("unchecked") public void setValueAt(Object value,int row,int col){
                var r=rows.get(row);try {
                    Object parsed=r.value instanceof Boolean?value instanceof Boolean?value:Boolean.parseBoolean(value.toString()):r.value instanceof Number?Double.parseDouble(value.toString()):value.toString();
                    if(parsed instanceof Number n&&!Double.isFinite(n.doubleValue()))throw new IllegalArgumentException("Enter a finite number.");
                    if(Objects.equals(parsed,r.value))return;
                    if(r.parent instanceof Map m)m.put(r.key,parsed);else ((List)r.parent).set((int)r.key,parsed);
                    FieldPackage.map(profile.get("provenance")).put(r.path,"user supplied");rows.set(row,new Row(r.parent,r.key,parsed,r.path,r.name,r.help));changed.run();fireTableRowsUpdated(row,row);
                }catch(Exception e){JOptionPane.showMessageDialog(GuidedParameterPanel.this,e.getMessage(),"Check this value",JOptionPane.ERROR_MESSAGE);}
            }
        };
        table=new JTable(model){@Override public javax.swing.table.TableCellRenderer getCellRenderer(int row,int col){return col==1&&rows.get(row).value instanceof Boolean?getDefaultRenderer(Boolean.class):super.getCellRenderer(row,col);}@Override public javax.swing.table.TableCellEditor getCellEditor(int row,int col){if(col==1){var r=rows.get(row);String[] choices=choices(r);if(choices!=null)return new DefaultCellEditor(new JComboBox<>(choices));if(r.value instanceof Boolean)return new DefaultCellEditor(new JCheckBox());}return super.getCellEditor(row,col);}};
        table.setRowHeight(29);table.getColumnModel().getColumn(0).setPreferredWidth(240);table.getColumnModel().getColumn(1).setPreferredWidth(100);table.getColumnModel().getColumn(2).setPreferredWidth(180);table.getColumnModel().getColumn(3).setPreferredWidth(420);add(new JScrollPane(table));
        var detail=new JTextArea("Select a setting to read its full explanation and provenance.",3,40);detail.setEditable(false);detail.setLineWrap(true);detail.setWrapStyleWord(true);detail.setOpaque(false);detail.setBorder(BorderFactory.createEmptyBorder(8,2,2,2));add(detail,BorderLayout.SOUTH);
        Runnable explain=()->{int selected=table.getSelectedRow();if(selected>=0&&selected<rows.size()){var r=rows.get(selected);detail.setText(r.name+" — "+origin(r.path)+"\n"+r.help);}};table.getSelectionModel().addListSelectionListener(e->explain.run());model.addTableModelListener(e->explain.run());
    }
    void commit(){if(table.isEditing())table.getCellEditor().stopCellEditing();}
    private String origin(String path){String parent=path;var provenance=FieldPackage.map(profile.get("provenance"));while(!provenance.containsKey(parent)&&parent.contains("/"))parent=parent.substring(0,parent.lastIndexOf('/'));String text=provenance.getOrDefault(parent,"").toString();if(text.contains("user"))return "User supplied";if(text.contains("calibrat")||text.contains("measured")||path.startsWith("runtime/calibration_data/"))return "Measured / calibrated";if(text.contains("source"))return "From CAD";return "Default — provisional";}
    private void flatten(Object o,String path,String page){
        if(o instanceof Map<?,?> map)for(var entry:map.entrySet())flatten(entry.getValue(),path+"/"+entry.getKey(),page,map,entry.getKey());
        else if(o instanceof List<?> list)for(int i=0;i<list.size();i++)flatten(list.get(i),path+"/"+i,page,list,i);
    }
    private void flatten(Object value,String path,String page,Object parent,Object key){
        if(value instanceof Map||value instanceof List){flatten(value,path,page);return;}
        if(value==null||!include(path,page))return;rows.add(new Row(parent,key,value,path,label(path),help(path)));
    }
    private boolean include(String path,String page){
        String key=base(path);if(Set.of("id","name","file","version","triangles","prepared_sha256","units").contains(key)&&!path.contains("/actuators/"))return false;
        if(page.equals("parts"))return path.contains("/joint/")||path.contains("/actuators/")||key.equals("role");
        if(page.equals("scale"))return Set.of("up_axis","origin_xyz_m","origin_rpy_rad").contains(key);
        return !path.contains("/joint/")&&!path.contains("/actuators/")&&!Set.of("role","length_unit","up_axis","origin_xyz_m","origin_rpy_rad").contains(key);
    }
    private static String base(String path){String[] bits=path.split("/");int i=bits.length-1;if(bits[i].matches("[0-9]+"))i--;return bits[i];}
    static String label(String path){
        String key=base(path);String text=switch(key){
            case "box_size_m"->"Box dimensions (m)";case "collision_xyz_m"->"Collision position (m)";case "collision_rpy_rad"->"Collision rotation (rad)";
            case "mass_kg"->"Mass (kg)";case "fallback_mass_kg"->"Missing-mass default (kg)";case "mass_mode"->"Mass source";case "inertia_mode"->"Inertia source";
            case "inertia_kg_m2"->"Inertia tensor (kg·m²)";case "com_xyz_m"->"Center of mass (m)";case "collision_strategy"->"Collision shape";
            case "origin_xyz_m"->"Model origin (m)";case "origin_rpy_rad"->"Model orientation (rad)";case "up_axis"->"CAD upward axis";
            case "xyz"->path.contains("/joint/")?"Joint position (source units)":"Position (m)";case "rpy"->"Joint rotation (rad)";case "axis"->"Joint axis direction";
            case "lower","upper"->"Joint "+key+" limit (rad; sliders in source units)";case "type"->"Joint movement";case "parent"->"Attached parent part";
            case "role"->"Part behavior";case "mechanicalReduction"->"Motor-to-joint ratio";case "name"->"Motor / servo name";
            case "collision_margin_m"->"Collision skin thickness (m)";case "max_hulls"->"Maximum convex pieces";case "field_half_extents_m"->"Field half-widths (m)";
            case "use_contact_compliance"->"Use compliant contact";case "material_override"->"Use this body's contact values";
            default->Character.toUpperCase(key.charAt(0))+key.substring(1).replace('_',' ');
        };
        String tail=path.substring(path.lastIndexOf('/')+1);if(tail.matches("[0-9]+")){int i=Integer.parseInt(tail);String[] axes=key.contains("rpy")?new String[]{"roll","pitch","yaw"}:new String[]{"X","Y","Z"};text+=" · "+(i<3?axes[i]:"component "+(i+1));}return text;
    }
    static String help(String path){String key=base(path);return switch(key){
        case "collision_strategy"->"Source keeps CAD collisions; mesh preserves more detail; boxes may block openings.";
        case "mass_mode"->"Use CAD mass, your override, or the saved missing-mass default.";
        case "inertia_mode"->"Rotational resistance: preserve CAD, derive from a box, or supply a tensor.";
        case "friction"->"Higher values resist sliding. CAD normally does not measure this.";
        case "restitution","fixed_restitution"->"Bounce fraction: 0 is no bounce, 1 is fully elastic.";
        case "role"->"Pieces move independently; decoration/reference parts do not collide.";
        case "type"->"Fixed moves with its parent; continuous rotates freely; revolute/slider use limits.";
        case "mechanicalReduction"->"Signed shaft turns per joint turn; verify direction and gearing.";
        case "material_override"->"A welded body has one material; set this on its body root.";
        case "max_hulls"->"More pieces may preserve holes but cost more and need collision review.";
        case "use_contact_compliance"->"Enable saved contact stiffness/damping instead of rigid default contact.";
        case "floor_strategy"->"Use the source floor, or generate a floor with the saved dimensions.";
        default->"Saved with this model. Keep provisional values for now or enter a known value.";
    };}
    private String[] choices(Row row){String key=base(row.path);return switch(key){
        case "role"->new String[]{"structure","surface","barrier","apparatus","piece","decoration","reference"};
        case "collision_strategy"->new String[]{"source","visual","box","sphere","cylinder","mesh","none"};
        case "mass_mode"->new String[]{"source","override","fallback"};case "inertia_mode"->new String[]{"source","box","override"};case "up_axis"->new String[]{"Z","Y"};
        case "floor_strategy"->new String[]{"source","generated"};case "type"->row.path.contains("/joint/")?new String[]{"fixed","continuous","revolute","prismatic"}:null;
        case "parent"->FieldPackage.map(profile.get("entities")).keySet().toArray(String[]::new);
        case "name"->{try{var names=controller.hardware(null);yield names.isEmpty()?null:names.toArray(String[]::new);}catch(Exception e){yield null;}}
        default->null;
    };}
}
