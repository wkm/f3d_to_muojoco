#Author-Gemini Agent
#Description-Exports Fusion 360 designs to MuJoCo MJCF format.

import adsk.core, adsk.fusion, adsk.cam, traceback
import os
import xml.etree.ElementTree as ET
import math

# Global variables to keep reference to the event handlers
_handlers = []
_app = None
_ui = None
CMD_ID = 'F3DToMuJoCo_Cmd_ID'
CMD_NAME = 'Export to MuJoCo'
CMD_Description = 'Export the current design to a MuJoCo MJCF (.xml) file and meshes.'

class Exporter:
    def __init__(self, export_path):
        self.export_path = export_path
        self.meshes_path = os.path.join(export_path, 'meshes')
        if not os.path.exists(self.meshes_path):
            os.makedirs(self.meshes_path)
        
        self.app = adsk.core.Application.get()
        self.design = self.app.activeProduct
        self.export_mgr = self.design.exportManager
        self.root_comp = self.design.rootComponent
        
        self._collect_all_joints()

    def _collect_all_joints(self):
        # Flatten all joints in the design into one list
        self.all_joints = []
        try:
            self.log("--- SCANNING ALL JOINTS ---")
            for comp in self.design.allComponents:
                for joint in comp.allJoints:
                    self.all_joints.append(joint)
                    
                    # Debug logging
                    try:
                        o1 = joint.occurrenceOne
                        o2 = joint.occurrenceTwo
                        path1 = o1.fullPathName if o1 else "None"
                        path2 = o2.fullPathName if o2 else "None"
                        self.log(f"  Joint found: '{joint.name}' in '{comp.name}' | Occ1: {path1} | Occ2: {path2}")
                    except:
                        pass
            self.log(f"--- TOTAL JOINTS FOUND: {len(self.all_joints)} ---")
        except:
            pass # Fail safe

    def log(self, message):
        try:
            print(f"[F3DToMuJoCo] {message}")
            # Write to the Text Commands palette
            text_palette = self.app.userInterface.palettes.itemById('TextCommands')
            if text_palette:
                text_palette.writeText(f"[F3DToMuJoCo] {message}")
        except:
            pass # Fail silently if palette not found

    def get_parent_occurrence(self, occ):
        # Helper to safely get the parent occurrence
        # Returns None if the parent is the Root Component
        try:
            return occ.assemblyContext
        except:
            return None

    def validate_design(self):
        # Check 1: Naming Conflicts
        name_counts = {}
        duplicates = []
        for comp in self.design.allComponents:
            name = comp.name
            if name in name_counts:
                name_counts[name] += 1
                if name_counts[name] == 2: # Only add to list once
                    duplicates.append(name)
            else:
                name_counts[name] = 1
        
        # Check 2: Flat Hierarchy (Joints between siblings)
        flat_joints = []
        for joint in self.root_comp.allJoints:
            if not joint.jointMotion: continue
            
            occ1 = joint.occurrenceOne
            occ2 = joint.occurrenceTwo
            
            # Skip if joint connects to Root (one occ is None) - this is usually fine (grounded)
            if not occ1 or not occ2: continue
            
            # Check if they share the same parent
            parent1 = self.get_parent_occurrence(occ1)
            parent2 = self.get_parent_occurrence(occ2)
            
            # If parents are same (both None=Root, or both same sub-assembly), it's a sibling joint
            if parent1 == parent2:
                flat_joints.append(f"{joint.name} (connects siblings '{occ1.name}' and '{occ2.name}')")

        # Construct Warning Message
        msg = ""
        if duplicates:
            msg += "CRITICAL: Duplicate Component Names found!\n"
            msg += "This will cause mesh files to overwrite each other.\n"
            msg += f"Duplicates: {', '.join(duplicates[:5])}...\n\n"
            
        if flat_joints:
            msg += "WARNING: Flat Hierarchy / Sibling Joints detected.\n"
            msg += "The exporter expects a nested hierarchy matching the kinematic chain.\n"
            msg += "Joints between sibling components may NOT be exported correctly.\n"
            msg += "Please drag child components INSIDE their parent components in the Browser.\n"
            msg += f"Affected Joints: {', '.join(flat_joints[:5])}...\n\n"

        if msg:
            msg += "Do you want to continue anyway?"
            res = self.app.userInterface.messageBox(msg, 'Pre-flight Validation', adsk.core.MessageBoxButtonTypes.YesNoButtonType, adsk.core.MessageBoxIconTypes.WarningIconType)
            return res == adsk.core.DialogResults.DialogYes
            
        return True

    def export_all(self):
        # 1. Validation
        if not self.validate_design():
            return

        progress = self.app.userInterface.createProgressDialog()
        try:
            self.log(f"Starting export to: {self.export_path}")
            
            # Count steps: Total Components + 1 (XML Build)
            total_steps = self.design.allComponents.count + 1
            
            progress.show('Exporting to MuJoCo', 'Initializing...', 0, total_steps, 0)
            
            # 1. Export Meshes
            if progress.wasCancelled: return
            self.save_meshes(progress)
            
            # 2. Build MJCF XML
            if progress.wasCancelled: return
            progress.message = "Building MJCF XML..."
            self.build_xml()
            progress.progressValue = total_steps
            
            self.log("Export finished successfully.")
            _ui.messageBox('Export Complete!')
            
        except Exception as e:
            self.log(f"Error: {str(e)}")
            _ui.messageBox(f"Export Failed:\n{str(e)}")
            traceback.print_exc()
        finally:
            progress.hide()

    def save_meshes(self, progress):
        # Iterate through all unique components
        current_step = 0
        for comp in self.design.allComponents:
            if progress.wasCancelled: return
            
            # Skip the root component mesh export (it usually just holds sub-components)
            # If root has bodies, we might want to export them? Let's allow it if it has bodies.
            # But usually root is just a container. Let's stick to skipping for now unless needed.
            if comp == self.design.rootComponent and comp.bRepBodies.count == 0:
                continue
            
            current_step += 1
            progress.progressValue = current_step
            progress.message = f"Processing component: {comp.name}"
            
            clean_comp_name = self.clean_name(comp.name)
            
            # Export EACH body in the component separately
            for i in range(comp.bRepBodies.count):
                body = comp.bRepBodies.item(i)
                clean_body_name = self.clean_name(body.name)
                
                # Unique filename: CompName_BodyName.stl
                # Note: If body names are not unique within a component, Fusion handles it, 
                # but cleaning might collide. Fusion default is "Body1", "Body2".
                stl_name = f"{clean_comp_name}_{clean_body_name}.stl"
                full_path = os.path.join(self.meshes_path, stl_name)
                
                self.log(f"Saving mesh: {stl_name}")
                
                # Create STL export options for the BODY
                stl_options = self.export_mgr.createSTLExportOptions(body, full_path)
                stl_options.sendToPrintUtility = False
                stl_options.isBinaryFormat = True
                stl_options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
                
                self.export_mgr.execute(stl_options)

    def clean_name(self, name):
        return name.replace(':', '_').replace(' ', '_')

    def matrix_to_quat(self, matrix):
        # Converts Fusion 360 Matrix3D to Quaternion [w, x, y, z]
        # Based on standard conversion algorithms
        
        # Get the cells
        # Fusion Matrix is [ R  t ]
        #                [ 0  1 ]
        # We only care about rotation R (top-left 3x3)
        # Row 0: 0, 1, 2
        # Row 1: 4, 5, 6
        # Row 2: 8, 9, 10
        
        # Access by (row, col)
        m00 = matrix.getCell(0, 0)
        m01 = matrix.getCell(0, 1)
        m02 = matrix.getCell(0, 2)
        m10 = matrix.getCell(1, 0)
        m11 = matrix.getCell(1, 1)
        m12 = matrix.getCell(1, 2)
        m20 = matrix.getCell(2, 0)
        m21 = matrix.getCell(2, 1)
        m22 = matrix.getCell(2, 2)

        tr = m00 + m11 + m22

        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (m21 - m12) / S
            qy = (m02 - m20) / S
            qz = (m10 - m01) / S
        elif (m00 > m11) and (m00 > m22):
            S = math.sqrt(1.0 + m00 - m11 - m22) * 2
            qw = (m21 - m12) / S
            qx = 0.25 * S
            qy = (m01 + m10) / S
            qz = (m02 + m20) / S
        elif m11 > m22:
            S = math.sqrt(1.0 + m11 - m00 - m22) * 2
            qw = (m02 - m20) / S
            qx = (m01 + m10) / S
            qy = 0.25 * S
            qz = (m12 + m21) / S
        else:
            S = math.sqrt(1.0 + m22 - m00 - m11) * 2
            qw = (m10 - m01) / S
            qx = (m02 + m20) / S
            qy = (m12 + m21) / S
            qz = 0.25 * S
        
        return f"{qw} {qx} {qy} {qz}"

    def build_xml(self):
        root_elem = ET.Element('mujoco', {'model': self.root_comp.name})
        
        # Add basic compiler and asset settings
        compiler = ET.SubElement(root_elem, 'compiler', {'angle': 'radian', 'meshdir': 'meshes'})
        
        # Add visual settings for better lighting
        visual = ET.SubElement(root_elem, 'visual')
        ET.SubElement(visual, 'headlight', {'ambient': '.4 .4 .4', 'diffuse': '.8 .8 .8', 'specular': '0.1 0.1 0.1'})
        ET.SubElement(visual, 'map', {'znear': '0.01'})
        ET.SubElement(visual, 'quality', {'shadowsize': '2048'})

        # Add assets (meshes)
        asset = ET.SubElement(root_elem, 'asset')
        for comp in self.design.allComponents:
            # Skip root if empty (consistent with save_meshes)
            if comp == self.root_comp and comp.bRepBodies.count == 0: continue
            
            clean_comp_name = self.clean_name(comp.name)
            
            # Register a mesh asset for EACH body
            for i in range(comp.bRepBodies.count):
                body = comp.bRepBodies.item(i)
                clean_body_name = self.clean_name(body.name)
                mesh_name = f"{clean_comp_name}_{clean_body_name}"
                
                ET.SubElement(asset, 'mesh', {'name': mesh_name, 'file': f"{mesh_name}.stl"})

        # Worldbody and recursively add bodies
        worldbody = ET.SubElement(root_elem, 'worldbody')
        
        # Add a floor and directional light
        ET.SubElement(worldbody, 'light', {'directional': 'true', 'diffuse': '.8 .8 .8', 'pos': '0 0 10', 'dir': '0 0 -1'})
        ET.SubElement(worldbody, 'geom', {'name': 'floor', 'type': 'plane', 'size': '5 5 0.1', 'rgba': '.9 .9 .9 1'})

        # Add skybox and ground textures/materials
        ET.SubElement(asset, 'texture', {
            'type': 'skybox',
            'builtin': 'gradient',
            'rgb1': '0.3 0.5 0.7',
            'rgb2': '0 0 0',
            'width': '512',
            'height': '3072'
        })
        ET.SubElement(asset, 'texture', {
            'type': '2d',
            'name': 'groundplane',
            'builtin': 'checker',
            'mark': 'edge',
            'rgb1': '0.2 0.3 0.4',
            'rgb2': '0.1 0.2 0.3',
            'markrgb': '0.8 0.8 0.8',
            'width': '300',
            'height': '300'
        })
        ET.SubElement(asset, 'material', {
            'name': 'groundplane',
            'texture': 'groundplane',
            'texuniform': 'true',
            'texrepeat': '5 5',
            'reflectance': '0.2'
        })

        # Initialize recursion with Identity matrix (World Frame)
        identity_transform = adsk.core.Matrix3D.create()
        
        # ROOT WRAPPER FOR COORDINATE CORRECTION
        # Fusion 360 often defaults to Y-Up. MuJoCo is Z-Up.
        # Since <compiler angle="radian"> is set, we use math.pi/2 (90 degrees).
        root_rot = math.pi / 2
        root_adapter = ET.SubElement(worldbody, 'body', {
            'name': 'root_adapter', 
            'pos': '0 0 0', 
            'euler': f"{root_rot} 0 0"
        })
        
        # Recursively process occurrences, attaching them to the ADAPTER
        for occ in self.root_comp.occurrences:
            # The parent of these top-level occurrences is the Root Component
            self.process_occurrence(occ, root_adapter, identity_transform, self.root_comp)

        # Write to file
        xml_path = os.path.join(self.export_path, f"{self.clean_name(self.root_comp.name)}.xml")
        tree = ET.ElementTree(root_elem)
        # Indent for pretty printing
        ET.indent(tree, space="  ", level=0) 
        tree.write(xml_path, encoding='utf-8', xml_declaration=True)

    def process_occurrence(self, occ, parent_xml_elem, parent_world_transform, parent_context):
        # Create body element
        clean_name = self.clean_name(occ.name)
        
        # 1. Get Current World Transform
        current_world_transform = occ.transform
        
        # 2. Calculate Relative Transform (Parent -> Child)
        parent_inv = parent_world_transform.copy()
        parent_inv.invert()
        rel_transform = parent_inv.copy()
        rel_transform.transformBy(current_world_transform)
        
        # 3. Extract Pos/Quat from Relative Transform
        trans = rel_transform.translation
        pos_str = f"{trans.x / 100.0} {trans.y / 100.0} {trans.z / 100.0}" # cm to m
        quat_str = self.matrix_to_quat(rel_transform)
        
        body = ET.SubElement(parent_xml_elem, 'body', {'name': clean_name, 'pos': pos_str, 'quat': quat_str})
        
        # 4. Add Inertial Properties
        self.process_inertial(occ.component, body)

        # 5. Add Joints
        self.process_joints(occ, body, parent_context)

        # 6. Add Geometry (Visual)
        # Add a geom for EACH body in the component
        clean_comp_name = self.clean_name(occ.component.name)
        for i in range(occ.component.bRepBodies.count):
            brep_body = occ.component.bRepBodies.item(i)
            clean_body_name = self.clean_name(brep_body.name)
            mesh_name = f"{clean_comp_name}_{clean_body_name}"
            
            # Get specific color for this body
            rgba_str = self.get_body_appearance_rgba(occ, brep_body)
            
            ET.SubElement(body, 'geom', {'type': 'mesh', 'mesh': mesh_name, 'rgba': rgba_str})

        # Process children
        for child_occ in occ.childOccurrences:
            self.process_occurrence(child_occ, body, current_world_transform, occ)

    def get_body_appearance_rgba(self, occ, body):
        # Emergency Orange (Fallback)
        default_rgba = "1.0 0.5 0.0 1.0"
        
        # 1. Check Body Override (Highest Priority for granular visuals)
        app = body.appearance
        if app: self.log(f"Found appearance on Body: {app.name}")

        # 2. Check Occurrence Override
        if not app:
            app = occ.appearance
            if app: self.log(f"Found appearance on Occurrence: {app.name}")
        
        # 3. Check Physical Material Appearance
        if not app and occ.component.material:
            app = occ.component.material.appearance
            if app: self.log(f"Found appearance on Material: {app.name}")

        # Note: We skipped 'component.appearance' because it doesn't exist.
        
        if not app:
            # self.log(f"No appearance found for {occ.name} / {body.name}") 
            # Reduced logging to avoid spam
            return default_rgba

        color_val = self.find_color_in_appearance(app)
        if color_val:
            return f"{color_val.red/255.0} {color_val.green/255.0} {color_val.blue/255.0} {color_val.opacity/255.0}"
        
        self.log(f"Appearance found ({app.name}) but could not extract RGB color.")
        return default_rgba

    def find_color_in_appearance(self, app):
        # Priority list of property names
        priority_names = ['color_base', 'Color', 'opaque_albedo', 'metal_f0']
        
        # 1. Search for priority properties
        for name in priority_names:
            prop = app.appearanceProperties.itemByName(name)
            if prop:
                # Check if it's a color property (some might be floats/ints)
                # We try to cast it
                color_prop = adsk.core.ColorProperty.cast(prop)
                if color_prop and color_prop.value:
                    return color_prop.value
        
        # 2. Fallback: Search ALL properties for ANY ColorProperty
        for prop in app.appearanceProperties:
            if prop.constructor.name == 'adsk::core::ColorProperty':
                 color_prop = adsk.core.ColorProperty.cast(prop)
                 if color_prop and color_prop.value:
                     return color_prop.value
        
        return None

    def process_inertial(self, comp, body_elem):
        try:
            props = comp.physicalProperties
            mass = props.mass # kg
            
            # Center of Mass
            # props.centerOfMass is relative to the Component's Coordinate System (Local)
            com = props.centerOfMass
            com_str = f"{com.x / 100.0} {com.y / 100.0} {com.z / 100.0}"
            
            # Moments of Inertia
            # getMomentsOfInertia returns (xx, yy, zz, xy, yz, xz) in kg/cm^2 ? 
            # Fusion units are internal (cm, kg).
            # We need to be careful about units. Fusion default is cm, kg.
            # Inertia is mass * dist^2. 
            # If mass is kg and dist is cm, then inertia is kg*cm^2.
            # MJCF expects kg*m^2.
            # Conversion factor: 1 cm^2 = 0.0001 m^2.
            
            (Ixx, Iyy, Izz, Ixy, Iyz, Ixz) = props.getMomentsOfInertia()
            
            scale = 0.0001
            full_inertia = f"{Ixx*scale} {Iyy*scale} {Izz*scale} {Ixy*scale} {Ixz*scale} {Iyz*scale}"
            
            ET.SubElement(body_elem, 'inertial', {
                'pos': com_str,
                'mass': str(mass),
                'fullinertia': full_inertia
            })
        except:
            # Fallback if physical properties fail (e.g. empty component)
            pass

    def get_token(self, obj):
        # Safely get entityToken for comparison
        try:
            return obj.entityToken if obj else "ROOT"
        except:
            return "UNKNOWN"

    def process_joints(self, occ, body_elem, target_parent):
        # Look for a joint that connects 'occ' to 'target_parent'
        self.log(f"Searching joints for occurrence: {occ.fullPathName}")
        target_name = target_parent.fullPathName if hasattr(target_parent, 'fullPathName') else "ROOT"
        self.log(f"  Target Parent: {target_name}")
        
        occ_token = self.get_token(occ)
        parent_token = self.get_token(target_parent)
        
        found_joint = False
        for joint in self.all_joints:
            if not joint.jointMotion: continue
            
            # Check if this joint connects occ to its parent
            o1 = joint.occurrenceOne
            o2 = joint.occurrenceTwo
            
            # Normalize 'None' to root_comp for comparison
            # Note: If joint is in a sub-component, None might mean "Containing Component"
            # But here we are comparing against occurrences in the assembly context.
            c1 = o1 if o1 else self.root_comp
            c2 = o2 if o2 else self.root_comp
            
            c1_token = self.get_token(c1)
            c2_token = self.get_token(c2)
            
            # Match tokens
            match = (c1_token == occ_token and c2_token == parent_token) or \
                    (c2_token == occ_token and c1_token == parent_token)
            
            if match:
                self.log(f"  MATCH FOUND: Joint '{joint.name}' connects '{occ.name}' to its parent.")
                self.add_joint_to_xml(joint, occ, body_elem)
                found_joint = True
                break 
        
        if not found_joint:
            self.log(f"  No joint found connecting '{occ.name}' to its expected parent.")

    def add_joint_to_xml(self, joint, child_occ, body_elem):
        motion = joint.jointMotion
        
        mj_type = ""
        if motion.jointType == adsk.fusion.JointTypes.RevoluteJointType:
            mj_type = "hinge"
        elif motion.jointType == adsk.fusion.JointTypes.SliderJointType:
            mj_type = "slide"
        else:
            return # Rigid, Ball, etc. not handled yet
            
        # Determine if child_occ is occurrenceOne or Two
        # We need to grab the geometry associated with the CHILD side of the joint
        # Fusion Docs: "Returns the joint geometry or origin for the first occurrence."
        # If Child is Occ1, we use Geometry1.
        is_one = (joint.occurrenceOne == child_occ)
        
        # However, we need the anchor point.
        # Usually, for a hinge, the anchor is the same on both bodies (they overlap).
        # We can grab either, BUT we must know which coordinate space it is in.
        # Docs: "The point is returned in the coordinate space of the PARENT COMPONENT of the occurrence."
        
        geom = joint.geometryOrOriginOne if is_one else joint.geometryOrOriginTwo
        
        # Coordinate Space Logic:
        # geom.origin is in the Coordinate System of child_occ's PARENT.
        # We want the position in the Coordinate System of child_occ (Local).
        
        # 1. Get Parent -> World Transform
        parent_occ = self.get_parent_occurrence(child_occ)
        if parent_occ:
            parent_to_world = parent_occ.transform
        else:
            parent_to_world = adsk.core.Matrix3D.create() # Identity (Root)

        # 2. Get World -> Child Transform
        # child_occ.transform is Child -> World
        world_to_child = child_occ.transform.copy()
        world_to_child.invert()
        
        # 3. Transform Origin: Parent -> World -> Child
        origin = geom.origin.copy()
        origin.transformBy(parent_to_world) # Now in World
        origin.transformBy(world_to_child)  # Now in Child Local
        
        pos_str = f"{origin.x / 100.0} {origin.y / 100.0} {origin.z / 100.0}"
        
        # 4. Transform Axis: Parent -> World -> Child
        axis_vec = geom.primaryAxisVector.copy()
        axis_vec.transformBy(parent_to_world)
        axis_vec.transformBy(world_to_child)
        axis_str = f"{axis_vec.x} {axis_vec.y} {axis_vec.z}"
        
        # 5. Limits
        extra_attrs = {}
        if mj_type == "hinge":
            rev_motion = adsk.fusion.RevoluteJointMotion.cast(motion)
            limits = rev_motion.rotationLimits
            if limits.isMinimumValueEnabled and limits.isMaximumValueEnabled:
                # Fusion is radians? Yes, internal units for angles are radians.
                extra_attrs['range'] = f"{limits.minimumValue} {limits.maximumValue}"
        elif mj_type == "slide":
            slide_motion = adsk.fusion.SliderJointMotion.cast(motion)
            limits = slide_motion.slideLimits
            if limits.isMinimumValueEnabled and limits.isMaximumValueEnabled:
                # Slide limits are in cm, convert to m
                extra_attrs['range'] = f"{limits.minimumValue/100.0} {limits.maximumValue/100.0}"
        
        ET.SubElement(body_elem, 'joint', {
            'name': self.clean_name(joint.name),
            'type': mj_type,
            'pos': pos_str,
            'axis': axis_str,
            **extra_attrs
        })

class ExportCommandExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            command = args.firingEvent.sender
            inputs = command.commandInputs
            
            # 1. Open Folder Dialog to select output location
            folderDlg = _ui.createFolderDialog()
            folderDlg.title = 'Select Output Directory for MuJoCo Export'
            res = folderDlg.showDialog()
            
            if res == adsk.core.DialogResults.DialogOK:
                export_path = folderDlg.folder
                exporter = Exporter(export_path)
                exporter.export_all()
            else:
                _ui.messageBox('Export cancelled.')

        except:
            if _ui:
                _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

class ExportCommandDestroyHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            # When the command is done, terminate the script
            # This releases the python script from memory
            adsk.terminate()
        except:
            if _ui:
                _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

class ExportCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            cmd = args.command
            
            # Connect Execute Handler
            onExecute = ExportCommandExecuteHandler()
            cmd.execute.add(onExecute)
            _handlers.append(onExecute)
            
            # Connect Destroy Handler (Crucial for Script Lifecycle)
            onDestroy = ExportCommandDestroyHandler()
            cmd.destroy.add(onDestroy)
            _handlers.append(onDestroy)
            
            # Add a brief description in the dialog if needed
            inputs = cmd.commandInputs
            inputs.addTextBoxCommandInput('info', '', 'Click OK to select the export folder and begin the process.', 2, True)

        except:
            if _ui:
                _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def run(context):
    global _app, _ui
    try:
        _app = adsk.core.Application.get()
        _ui  = _app.userInterface
        
        # Confirmation that script is running
        _ui.palettes.itemById('TextCommands').writeText("[F3DToMuJoCo] SCRIPT RELOADED - VERSION 0.1.5 - DEBUG MODE ACTIVE")
        print("[F3DToMuJoCo] SCRIPT RELOADED - VERSION 0.1.5")
        
        # PREVENT EARLY TERMINATION
        adsk.autoTerminate(False)
        
        # Create the command definition
        cmdDef = _ui.commandDefinitions.itemById(CMD_ID)
        if not cmdDef:
            cmdDef = _ui.commandDefinitions.addButtonDefinition(CMD_ID, CMD_NAME, CMD_Description, '')
        
        # Connect to the command created event
        onCommandCreated = ExportCommandCreatedHandler()
        cmdDef.commandCreated.add(onCommandCreated)
        _handlers.append(onCommandCreated)
        
        # Execute the command immediately
        cmdDef.execute()

    except:
        if _ui:
            _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def stop(context):
    try:
        # Clean up the UI
        cmdDef = _ui.commandDefinitions.itemById(CMD_ID)
        if cmdDef:
            cmdDef.deleteMe()
    except:
        if _ui:
            _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
