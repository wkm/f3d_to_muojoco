# Author-Gemini Agent
# Description-Exports Fusion 360 designs to MuJoCo MJCF format.

import adsk.core
import adsk.fusion
import adsk.cam
import traceback
import os
import xml.etree.ElementTree as ET
import math

# Global variables to keep reference to the event handlers
_handlers = []
_app = None
_ui = None
CMD_ID = "F3DToMuJoCo_Cmd_ID"
CMD_NAME = "Export to MuJoCo"
CMD_Description = "Export the current design to a MuJoCo MJCF (.xml) file and meshes."


class Exporter:
    def __init__(
        self,
        export_path,
        debug_mode=False,
        enable_actuators=True,
        actuator_type="position",
        enable_sensors=True,
    ):
        self.export_path = export_path
        self.debug_mode = debug_mode
        self.enable_actuators = enable_actuators
        self.actuator_type = actuator_type  # 'position', 'velocity', 'motor'
        self.enable_sensors = enable_sensors
        self.exported_joints = []  # Track joints for actuator/sensor generation

        # Initialize log file
        self.log_file = None
        log_path = os.path.join(export_path, "export.log")
        try:
            self.log_file = open(log_path, "w", encoding="utf-8")
        except Exception:
            pass  # Fail silently if can't create log file

        # Manifest data - tracks everything for YAML output
        self.manifest = {
            "inputs": {},
            "files": {},
            "model": {},
            "components": [],
            "bodies": [],
            "joints": [],
            "actuators": [],
            "sensors": [],
            "warnings": [],
            "joint_processing": [],
            "appearance_extraction": [],
        }

        self.meshes_path = os.path.join(export_path, "meshes")
        if not os.path.exists(self.meshes_path):
            os.makedirs(self.meshes_path)

        self.app = adsk.core.Application.get()
        self.design = self.app.activeProduct
        self.export_mgr = self.design.exportManager
        self.root_comp = self.design.rootComponent

        # Calculate Unit Scale Factor
        # Fusion API always returns Centimeters.
        # STLs are exported in Document Units (e.g., mm, in, m).
        # MuJoCo expects Meters for everything in the XML (pos, mass, inertia).
        units_mgr = self.design.unitsManager
        
        # 1. Scale factor for XML values (Internal cm -> Meters)
        # API returns cm, we want m. 1 cm = 0.01 m.
        self.length_scale = units_mgr.convert(1, units_mgr.internalUnits, "m")
        
        # 2. Scale factor for Meshes (Document Units -> Meters)
        doc_units = units_mgr.defaultLengthUnits
        self.mesh_scale_factor = units_mgr.convert(1, doc_units, "m")

        # Initialize joints list before collecting (prevents crash if collection fails)
        self.all_joints = []
        self._collect_all_joints()

    def _collect_all_joints(self):
        # Collect all joints from the root component context.
        # root_comp.allJoints returns proxies for all joints in the hierarchy,
        # which means their geometry and occurrences are in the World (Root) context.
        self.all_joints = []
        try:
            units_mgr = self.design.unitsManager
            doc_units = units_mgr.defaultLengthUnits
            self.log(
                f"Initializing exporter (document units: {doc_units}, scale factor: {self.length_scale:.6f})"
            )

            self.log("Collecting joints from assembly...")
            for joint in self.root_comp.allJoints:
                self.all_joints.append(joint)

                # Log joint connections (always, for debugging)
                try:
                    o1 = joint.occurrenceOne
                    o2 = joint.occurrenceTwo
                    path1 = o1.fullPathName if o1 else "Root"
                    path2 = o2.fullPathName if o2 else "Root"
                    joint_type = self._get_joint_type_name(joint)
                    self.log(f"  Joint '{joint.name}' ({joint_type}): {path1} <-> {path2}")
                except Exception:
                    self.log(f"  Joint '{joint.name}': <error reading details>")

            self.log(f"Found {len(self.all_joints)} joint(s) in assembly")

            # Now collect model metadata (after joints are counted)
            self._collect_model_info(len(self.all_joints))
        except Exception as e:
            self.log(f"Warning: Failed to collect joints: {type(e).__name__}: {e}")

    def _collect_model_info(self, joint_count):
        """Collect model structure information for the manifest."""
        s = self.length_scale

        # Count totals
        total_components = self.design.allComponents.count
        total_bodies = sum(c.bRepBodies.count for c in self.design.allComponents)

        self.manifest["model"] = {
            "root_component": self.root_comp.name,
            "total_components": total_components,
            "total_joints": joint_count,
            "total_bodies": total_bodies,
            "occurrences": [],
        }

        self.log(f"Model: {self.root_comp.name} ({total_components} components, {total_bodies} bodies, {joint_count} joints)")

        # Collect occurrence details
        self._collect_occurrences_recursive(self.root_comp.occurrences, "")

    def _collect_occurrences_recursive(self, occurrences, parent_path):
        """Recursively collect occurrence information."""
        s = self.length_scale
        for occ in occurrences:
            try:
                full_path = occ.fullPathName
                transform = occ.transform

                # Extract transform components
                trans = transform.translation
                pos = [trans.x * s, trans.y * s, trans.z * s]

                # Get bounding box
                bb = occ.boundingBox
                bb_min = [bb.minPoint.x * s, bb.minPoint.y * s, bb.minPoint.z * s]
                bb_max = [bb.maxPoint.x * s, bb.maxPoint.y * s, bb.maxPoint.z * s]

                occ_info = {
                    "name": occ.name,
                    "full_path": full_path,
                    "world_position": pos,
                    "bounding_box_min": bb_min,
                    "bounding_box_max": bb_max,
                    "body_count": occ.component.bRepBodies.count,
                }
                self.manifest["model"]["occurrences"].append(occ_info)

                self.log(f"  Occurrence '{occ.name}': pos=({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}), {occ.component.bRepBodies.count} bodies")

                # Recurse into children
                self._collect_occurrences_recursive(occ.childOccurrences, full_path)
            except Exception as e:
                self.log(f"  Occurrence '{occ.name}': <error: {type(e).__name__}>")

    def _get_joint_type_name(self, joint):
        """Get a readable name for the joint type."""
        if not joint.jointMotion:
            return "Rigid"
        jt = joint.jointMotion.jointType
        type_names = {
            adsk.fusion.JointTypes.RevoluteJointType: "Revolute",
            adsk.fusion.JointTypes.SliderJointType: "Slider",
            adsk.fusion.JointTypes.CylindricalJointType: "Cylindrical",
            adsk.fusion.JointTypes.PinSlotJointType: "PinSlot",
            adsk.fusion.JointTypes.PlanarJointType: "Planar",
            adsk.fusion.JointTypes.BallJointType: "Ball",
            adsk.fusion.JointTypes.RigidJointType: "Rigid",
        }
        return type_names.get(jt, f"Unknown({jt})")

    def log(self, message):
        try:
            formatted_msg = f"[F3DToMuJoCo] {message}"
            print(formatted_msg)
            # Write to the Text Commands palette
            text_palette = self.app.userInterface.palettes.itemById("TextCommands")
            if text_palette:
                text_palette.writeText(formatted_msg)
            # Write to log file
            if self.log_file:
                self.log_file.write(formatted_msg + "\n")
                self.log_file.flush()
        except Exception:
            pass  # Fail silently if palette not available

    def get_parent_occurrence(self, occ):
        # Helper to safely get the parent occurrence
        # Returns None if the parent is the Root Component
        try:
            return occ.assemblyContext
        except Exception:
            return None

    def validate_design(self):
        # Check 1: Naming Conflicts
        name_counts = {}
        duplicates = []
        for comp in self.design.allComponents:
            name = comp.name
            if name in name_counts:
                name_counts[name] += 1
                if name_counts[name] == 2:  # Only add to list once
                    duplicates.append(name)
            else:
                name_counts[name] = 1

        # Check 2: Flat Hierarchy (Joints between siblings)
        flat_joints = []
        for joint in self.root_comp.allJoints:
            if not joint.jointMotion:
                continue

            occ1 = joint.occurrenceOne
            occ2 = joint.occurrenceTwo

            # Skip if joint connects to Root (one occ is None) - this is usually fine (grounded)
            if not occ1 or not occ2:
                continue

            # Check if they share the same parent
            parent1 = self.get_parent_occurrence(occ1)
            parent2 = self.get_parent_occurrence(occ2)

            # If parents are same (both None=Root, or both same sub-assembly), it's a sibling joint
            if parent1 == parent2:
                flat_joints.append(
                    f"{joint.name} (connects siblings '{occ1.name}' and '{occ2.name}')"
                )

        # Check 3: Missing Physical Materials
        missing_materials = []
        for comp in self.design.allComponents:
            # Skip empty components (no bodies)
            if comp.bRepBodies.count == 0:
                continue

            # Skip root component if it's just a container
            if comp == self.design.rootComponent and comp.bRepBodies.count == 0:
                continue

            # Try to get physical properties
            has_valid_material = False
            try:
                props = comp.physicalProperties
                if props and props.mass > 0:
                    has_valid_material = True
            except Exception:
                pass

            if not has_valid_material:
                missing_materials.append(comp.name)

        # Construct Warning Message and track in manifest
        msg = ""
        if duplicates:
            msg += "CRITICAL: Duplicate Component Names found!\n"
            msg += "This will cause mesh files to overwrite each other.\n"
            msg += f"Duplicates: {', '.join(duplicates[:5])}...\n\n"
            for dup in duplicates:
                self.manifest["warnings"].append(f"Duplicate component name: {dup}")

        if missing_materials:
            msg += "CRITICAL: Components without Physical Materials!\n"
            msg += "MuJoCo requires mass and inertia for proper dynamics.\n"
            msg += "Right-click each component → Physical Material → Assign material\n"
            msg += f"Missing materials: {', '.join(missing_materials[:10])}\n"
            if len(missing_materials) > 10:
                msg += f"... and {len(missing_materials) - 10} more\n"
            msg += "\n"
            for mat in missing_materials:
                self.manifest["warnings"].append(f"Missing physical material: {mat}")

        if flat_joints:
            msg += "WARNING: Flat Hierarchy / Sibling Joints detected.\n"
            msg += "The exporter expects a nested hierarchy matching the kinematic chain.\n"
            msg += "Joints between sibling components may NOT be exported correctly.\n"
            msg += "Please drag child components INSIDE their parent components in the Browser.\n"
            msg += f"Affected Joints: {', '.join(flat_joints[:5])}...\n\n"
            for fj in flat_joints:
                self.manifest["warnings"].append(f"Sibling joint: {fj}")

        if msg:
            msg += "Do you want to continue anyway?"
            res = self.app.userInterface.messageBox(
                msg,
                "Pre-flight Validation",
                adsk.core.MessageBoxButtonTypes.YesNoButtonType,
                adsk.core.MessageBoxIconTypes.WarningIconType,
            )
            return res == adsk.core.DialogResults.DialogYes

        return True

    def export_all(self):
        # 1. Validation
        if not self.validate_design():
            return

        progress = self.app.userInterface.createProgressDialog()
        try:
            self.log(f"Beginning export to {self.export_path}")

            # Count steps: Total Components + 1 (XML Build)
            total_steps = self.design.allComponents.count + 1

            progress.show("Exporting to MuJoCo", "Initializing...", 0, total_steps, 0)

            # 1. Export Meshes
            if progress.wasCancelled:
                return
            self.save_meshes(progress)

            # 2. Build MJCF XML
            if progress.wasCancelled:
                return
            progress.message = "Building MJCF XML..."
            self.build_xml()
            progress.progressValue = total_steps

            # 3. Save manifest
            progress.message = "Saving manifest..."
            self.save_manifest()

            self.log("Export completed successfully")
            _ui.messageBox("Export Complete!")

        except Exception as e:
            self.log(f"Export failed: {str(e)}")
            _ui.messageBox(f"Export Failed:\n{str(e)}")
            traceback.print_exc()
        finally:
            progress.hide()
            self.close_log_file()

    def save_meshes(self, progress):
        # Iterate through all unique components
        current_step = 0
        for comp in self.design.allComponents:
            if progress.wasCancelled:
                return

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

                if self.debug_mode:
                    self.log(f"Exporting mesh: {stl_name}")

                # Create STL export options for the BODY
                stl_options = self.export_mgr.createSTLExportOptions(body, full_path)
                stl_options.sendToPrintUtility = False
                stl_options.isBinaryFormat = True
                stl_options.meshRefinement = (
                    adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
                )

                self.export_mgr.execute(stl_options)

    def clean_name(self, name):
        return name.replace(":", "_").replace(" ", "_")

    def format_vec3(self, x, y, z, decimals=6):
        """Format a 3D vector with specified decimal precision."""
        return f"{x:.{decimals}f} {y:.{decimals}f} {z:.{decimals}f}"

    def format_quat(self, w, x, y, z, decimals=6):
        """Format a quaternion with specified decimal precision."""
        return f"{w:.{decimals}f} {x:.{decimals}f} {y:.{decimals}f} {z:.{decimals}f}"

    def format_inertia(self, ixx, iyy, izz, ixy, ixz, iyz, decimals=6):
        """Format a full inertia tensor with specified decimal precision."""
        # Use scientific notation for inertia as values can be very small (scaling with Length^2)
        return f"{ixx:.{decimals}e} {iyy:.{decimals}e} {izz:.{decimals}e} {ixy:.{decimals}e} {ixz:.{decimals}e} {iyz:.{decimals}e}"

    def format_range(self, min_val, max_val, decimals=6):
        """Format a range (min, max) with specified decimal precision."""
        return f"{min_val:.{decimals}f} {max_val:.{decimals}f}"

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

        return self.format_quat(qw, qx, qy, qz)

    def build_xml(self):
        """Build and save both bot.xml and scene.xml files."""
        clean_model_name = self.clean_name(self.root_comp.name)

        # Build bot.xml (robot model)
        self._build_bot_xml(clean_model_name)

        # Build scene.xml (environment that includes bot.xml)
        self._build_scene_xml(clean_model_name)

    def _build_bot_xml(self, model_name):
        """Build the robot model XML (bot.xml)."""
        root_elem = ET.Element("mujoco", {"model": model_name})

        # Add basic compiler and asset settings
        ET.SubElement(
            root_elem, 
            "compiler", 
            {"angle": "radian", "meshdir": "meshes"}
        )

        # Add default section (if actuators enabled)
        if self.enable_actuators:
            self._add_default_section(root_elem)

        # Add assets (meshes)
        asset = ET.SubElement(root_elem, "asset")
        for comp in self.design.allComponents:
            # Skip root if empty (consistent with save_meshes)
            if comp == self.root_comp and comp.bRepBodies.count == 0:
                continue

            clean_comp_name = self.clean_name(comp.name)

            # Register a mesh asset for EACH body
            for i in range(comp.bRepBodies.count):
                body = comp.bRepBodies.item(i)
                clean_body_name = self.clean_name(body.name)
                mesh_name = f"{clean_comp_name}_{clean_body_name}"
                mesh_scale_str = f"{self.mesh_scale_factor:.6f} {self.mesh_scale_factor:.6f} {self.mesh_scale_factor:.6f}"

                ET.SubElement(
                    asset, "mesh", {"name": mesh_name, "file": f"{mesh_name}.stl", "scale": mesh_scale_str}
                )

        # Worldbody with robot hierarchy
        worldbody = ET.SubElement(root_elem, "worldbody")

        # Initialize recursion with Identity matrix (World Frame)
        identity_transform = adsk.core.Matrix3D.create()

        # ROOT WRAPPER FOR COORDINATE CORRECTION
        # Fusion 360 often defaults to Y-Up. MuJoCo is Z-Up.
        # The adapter is part of the robot so it works in any scene.
        root_rot = math.pi / 2
        root_adapter = ET.SubElement(
            worldbody,
            "body",
            {"name": "base", "pos": "0 0 0", "euler": f"{root_rot} 0 0"},
        )

        # Recursively process occurrences, attaching them to the ADAPTER
        for occ in self.root_comp.occurrences:
            self.process_occurrence(
                occ, root_adapter, identity_transform, self.root_comp
            )

        # Add contacts (exclusions)
        self._add_contacts(root_elem)

        # Add actuators and sensors
        if self.enable_actuators:
            self._add_actuators(root_elem)
        if self.enable_sensors:
            self._add_sensors(root_elem)

        # Write bot.xml
        xml_path = os.path.join(self.export_path, "bot.xml")
        tree = ET.ElementTree(root_elem)
        ET.indent(tree, space="  ", level=0)
        tree.write(xml_path, encoding="utf-8", xml_declaration=True)
        self.log(f"Saved robot model to bot.xml")

    def _build_scene_xml(self, model_name):
        """Build the scene XML that includes the robot (scene.xml)."""
        root_elem = ET.Element("mujoco", {"model": f"{model_name}_scene"})

        # Include the robot model
        ET.SubElement(root_elem, "include", {"file": "bot.xml"})

        # Visual settings for better lighting
        visual = ET.SubElement(root_elem, "visual")
        ET.SubElement(
            visual,
            "headlight",
            {"ambient": ".4 .4 .4", "diffuse": ".8 .8 .8", "specular": "0.1 0.1 0.1"},
        )
        ET.SubElement(visual, "map", {"znear": "0.01"})
        ET.SubElement(visual, "quality", {"shadowsize": "2048"})

        # Scene assets (textures, materials)
        asset = ET.SubElement(root_elem, "asset")
        ET.SubElement(
            asset,
            "texture",
            {
                "type": "skybox",
                "builtin": "gradient",
                "rgb1": "0.3 0.5 0.7",
                "rgb2": "0 0 0",
                "width": "512",
                "height": "3072",
            },
        )
        ET.SubElement(
            asset,
            "texture",
            {
                "type": "2d",
                "name": "groundplane",
                "builtin": "checker",
                "mark": "edge",
                "rgb1": "0.2 0.3 0.4",
                "rgb2": "0.1 0.2 0.3",
                "markrgb": "0.8 0.8 0.8",
                "width": "300",
                "height": "300",
            },
        )
        ET.SubElement(
            asset,
            "material",
            {
                "name": "groundplane",
                "texture": "groundplane",
                "texuniform": "true",
                "texrepeat": "5 5",
                "reflectance": "0.2",
            },
        )

        # Worldbody with environment
        worldbody = ET.SubElement(root_elem, "worldbody")
        ET.SubElement(
            worldbody,
            "light",
            {
                "directional": "true",
                "diffuse": ".8 .8 .8",
                "pos": "0 0 10",
                "dir": "0 0 -1",
            },
        )
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": "floor",
                "type": "plane",
                "size": "5 5 0.1",
                "material": "groundplane",
            },
        )

        # Write scene.xml
        xml_path = os.path.join(self.export_path, "scene.xml")
        tree = ET.ElementTree(root_elem)
        ET.indent(tree, space="  ", level=0)
        tree.write(xml_path, encoding="utf-8", xml_declaration=True)
        self.log(f"Saved scene to scene.xml")

    def process_occurrence(
        self, occ, parent_xml_elem, parent_world_transform, parent_context
    ):
        # Create body element
        clean_name = self.clean_name(occ.name)

        # 1. Get World Transform
        # When traversing the hierarchy via childOccurrences starting from root,
        # Fusion returns the transform relative to the assembly context (Root).
        current_world_transform = occ.transform

        # 2. Calculate Relative Transform (Parent -> Child)
        # MuJoCo expects the position relative to the parent body frame.
        parent_inv = parent_world_transform.copy()
        parent_inv.invert()
        rel_transform = parent_inv.copy()
        rel_transform.transformBy(current_world_transform)

        # 3. Extract Pos/Quat from Relative Transform
        trans = rel_transform.translation
        # Scale to match Document Units (e.g., cm -> mm)
        s = self.length_scale
        pos_str = self.format_vec3(trans.x * s, trans.y * s, trans.z * s)
        quat_str = self.matrix_to_quat(rel_transform)

        body = ET.SubElement(
            parent_xml_elem,
            "body",
            {"name": clean_name, "pos": pos_str, "quat": quat_str},
        )

        # Track component in manifest
        parent_name = (
            parent_xml_elem.get("name") if parent_xml_elem.tag == "body" else "worldbody"
        )
        self.manifest["components"].append(
            {
                "name": clean_name,
                "parent": parent_name,
                "position": pos_str,
                "quaternion": quat_str,
            }
        )

        # 4. Add Inertial Properties
        self.process_inertial(occ.component, body, clean_name)

        # 5. Add Joints
        self.process_joints(occ, body, parent_context, parent_name)

        # 6. Add Geometry (Visual)
        # Add a geom for EACH body in the component
        clean_comp_name = self.clean_name(occ.component.name)
        for i in range(occ.component.bRepBodies.count):
            brep_body = occ.component.bRepBodies.item(i)
            clean_body_name = self.clean_name(brep_body.name)
            mesh_name = f"{clean_comp_name}_{clean_body_name}"

            # Get specific color for this body
            rgba_str = self.get_body_appearance_rgba(occ, brep_body, mesh_name)

            ET.SubElement(
                body, "geom", {"type": "mesh", "mesh": mesh_name, "rgba": rgba_str}
            )

            # Track body/mesh in manifest
            self.manifest["bodies"].append(
                {
                    "name": mesh_name,
                    "component": clean_name,
                    "mesh": f"meshes/{mesh_name}.stl",
                    "color": rgba_str,
                }
            )

        # Process children
        for child_occ in occ.childOccurrences:
            self.process_occurrence(child_occ, body, current_world_transform, occ)

    def get_body_appearance_rgba(self, occ, body, mesh_name=None):
        """Extract RGBA color from body appearance, tracking the source for debugging."""
        # Emergency Orange (Fallback)
        default_rgba = "1.0 0.5 0.0 1.0"

        source = "fallback"
        app = None
        app_name = None

        # 1. Check Body Override (Highest Priority for granular visuals)
        if body.appearance:
            app = body.appearance
            source = "body_appearance"

        # 2. Check Occurrence Override
        if not app and occ.appearance:
            app = occ.appearance
            source = "occurrence_appearance"

        # 3. Check Physical Material Appearance
        if not app and occ.component.material and occ.component.material.appearance:
            app = occ.component.material.appearance
            source = "material_appearance"

        if app:
            app_name = app.name

        if not app:
            # Track fallback case
            if mesh_name:
                self.manifest["appearance_extraction"].append({
                    "body": mesh_name,
                    "source": "fallback",
                    "appearance_name": None,
                    "property_used": None,
                    "rgba": [1.0, 0.5, 0.0, 1.0],
                })
            return default_rgba

        color_val, prop_used = self.find_color_in_appearance(app)
        if color_val:
            rgba = [
                color_val.red / 255.0,
                color_val.green / 255.0,
                color_val.blue / 255.0,
                color_val.opacity / 255.0,
            ]
            rgba_str = f"{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}"

            # Track successful extraction
            if mesh_name:
                self.manifest["appearance_extraction"].append({
                    "body": mesh_name,
                    "source": source,
                    "appearance_name": app_name,
                    "property_used": prop_used,
                    "rgba": rgba,
                })
            return rgba_str

        # Couldn't extract color from appearance
        self.log(f"  Warning: Could not extract color from appearance '{app_name}' on {occ.name}")
        if mesh_name:
            self.manifest["appearance_extraction"].append({
                "body": mesh_name,
                "source": "fallback",
                "appearance_name": app_name,
                "property_used": None,
                "rgba": [1.0, 0.5, 0.0, 1.0],
                "error": f"No color property found in appearance '{app_name}'",
            })
        return default_rgba

    def find_color_in_appearance(self, app):
        """Find color property in appearance, returning (color_value, property_name) tuple."""
        # Priority list of property names
        priority_names = ["color_base", "Color", "opaque_albedo", "metal_f0"]

        # 1. Search for priority properties
        for name in priority_names:
            prop = app.appearanceProperties.itemByName(name)
            if prop:
                # Check if it's a color property (some might be floats/ints)
                # We try to cast it
                color_prop = adsk.core.ColorProperty.cast(prop)
                if color_prop and color_prop.value:
                    return (color_prop.value, name)

        # 2. Fallback: Search ALL properties for ANY ColorProperty
        for prop in app.appearanceProperties:
            if prop.constructor.name == "adsk::core::ColorProperty":
                color_prop = adsk.core.ColorProperty.cast(prop)
                if color_prop and color_prop.value:
                    return (color_prop.value, prop.name)

        return (None, None)

    def process_inertial(self, comp, body_elem, comp_name=None):
        try:
            props = comp.physicalProperties
            mass = props.mass  # kg

            # Center of Mass
            # props.centerOfMass is relative to the Component's Coordinate System (Local)
            com = props.centerOfMass
            s = self.length_scale
            com_str = self.format_vec3(com.x * s, com.y * s, com.z * s)

            # Moments of Inertia
            # Fusion units are internal (cm, kg).
            # Inertia scales with length squared.
            s2 = s * s

            (Ixx, Iyy, Izz, Ixy, Iyz, Ixz) = props.getMomentsOfInertia()

            full_inertia = self.format_inertia(
                Ixx * s2, Iyy * s2, Izz * s2, Ixy * s2, Ixz * s2, Iyz * s2
            )

            ET.SubElement(
                body_elem,
                "inertial",
                {"pos": com_str, "mass": str(mass), "fullinertia": full_inertia},
            )

            # Track inertial data in manifest for all bodies of this component
            if comp_name:
                for body_data in self.manifest["bodies"]:
                    if body_data["component"] == comp_name:
                        body_data["mass"] = mass
                        body_data["center_of_mass"] = com_str
                        body_data["inertia"] = full_inertia

        except Exception:
            # Fallback: Estimate inertial properties from bounding box
            # This is critical - without inertia, MuJoCo dynamics don't work!
            self.log(
                f"No physical material on {comp.name}, using estimated inertial properties"
            )

            # Try to get bounding box for estimation
            try:
                if comp.bRepBodies.count > 0:
                    # Get first body's bounding box
                    bb = comp.bRepBodies.item(0).boundingBox
                    s = self.length_scale

                    # Estimate center of mass from bounding box center
                    cx = (bb.minPoint.x + bb.maxPoint.x) / 2.0 * s
                    cy = (bb.minPoint.y + bb.maxPoint.y) / 2.0 * s
                    cz = (bb.minPoint.z + bb.maxPoint.z) / 2.0 * s
                    com_str = self.format_vec3(cx, cy, cz)

                    # Estimate mass from volume (assume density ~1000 kg/m³ for plastic/wood)
                    # Volume in cm³, convert to m³, multiply by density
                    dx = abs(bb.maxPoint.x - bb.minPoint.x)
                    dy = abs(bb.maxPoint.y - bb.minPoint.y)
                    dz = abs(bb.maxPoint.z - bb.minPoint.z)
                    volume_cm3 = dx * dy * dz  # cm³
                    volume_m3 = volume_cm3 / 1e6  # m³
                    mass = max(0.01, volume_m3 * 1000)  # kg, minimum 10g

                    # Estimate inertia using box approximation: I = (1/12) * m * (d1² + d2²)
                    # Scale dimensions to document units
                    s2 = s * s
                    dx_scaled = dx * s
                    dy_scaled = dy * s
                    dz_scaled = dz * s

                    Ixx = (mass / 12.0) * (dy_scaled**2 + dz_scaled**2)
                    Iyy = (mass / 12.0) * (dx_scaled**2 + dz_scaled**2)
                    Izz = (mass / 12.0) * (dx_scaled**2 + dy_scaled**2)

                    # Use diagonal inertia (no cross terms)
                    full_inertia = self.format_inertia(Ixx, Iyy, Izz, 0, 0, 0)

                    ET.SubElement(
                        body_elem,
                        "inertial",
                        {
                            "pos": com_str,
                            "mass": str(mass),
                            "fullinertia": full_inertia,
                        },
                    )

                    # Track estimated inertial data in manifest
                    if comp_name:
                        for body_data in self.manifest["bodies"]:
                            if body_data["component"] == comp_name:
                                body_data["mass"] = mass
                                body_data["center_of_mass"] = com_str
                                body_data["inertia"] = full_inertia
                                body_data["estimated_inertia"] = True

                    if self.debug_mode:
                        self.log(
                            f"  Estimated: mass={mass:.4f}kg, volume={volume_m3 * 1e6:.1f}cm³"
                        )
                else:
                    # No bodies - use minimal inertia
                    self.log(
                        f"  Warning: {comp.name} has no bodies, using minimal inertia"
                    )
                    ET.SubElement(
                        body_elem,
                        "inertial",
                        {
                            "pos": "0 0 0",
                            "mass": "0.01",
                            "diaginertia": "0.001 0.001 0.001",
                        },
                    )
            except Exception as fallback_error:
                # Last resort: minimal inertia
                if self.debug_mode:
                    self.log(
                        f"  Fallback estimation failed: {type(fallback_error).__name__}"
                    )
                ET.SubElement(
                    body_elem,
                    "inertial",
                    {
                        "pos": "0 0 0",
                        "mass": "0.01",
                        "diaginertia": "0.001 0.001 0.001",
                    },
                )

    def get_token(self, obj):
        # Safely get entityToken for comparison
        try:
            return obj.entityToken if obj else "ROOT"
        except Exception:
            return "UNKNOWN"

    def process_joints(self, occ, body_elem, target_parent, parent_body_name="world"):
        # Look for a joint that connects 'occ' to 'target_parent'
        target_name = (
            target_parent.fullPathName
            if hasattr(target_parent, "fullPathName")
            else "Root"
        )

        occ_token = self.get_token(occ)
        parent_token = self.get_token(target_parent)

        found_joint = False
        for joint in self.all_joints:
            if not joint.jointMotion:
                continue

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
            match = (c1_token == occ_token and c2_token == parent_token) or (
                c2_token == occ_token and c1_token == parent_token
            )

            if match:
                self.add_joint_to_xml(joint, occ, body_elem, parent_body_name)
                found_joint = True
                break

        if not found_joint:
            self.log(f"  No joint found for '{occ.name}' (rigidly attached to {target_name})")

    def add_joint_to_xml(self, joint, child_occ, body_elem, parent_body_name):
        motion = joint.jointMotion

        mj_type = ""
        fusion_type = self._get_joint_type_name(joint)
        if motion.jointType == adsk.fusion.JointTypes.RevoluteJointType:
            mj_type = "hinge"
        elif motion.jointType == adsk.fusion.JointTypes.SliderJointType:
            mj_type = "slide"
        else:
            self.log(f"  Skipping joint '{joint.name}' - type '{fusion_type}' not supported")
            return  # Rigid, Ball, etc. not handled yet

        # Strategy: Use World-to-World transformation.
        # Since we are using joint proxies from the root component,
        # both child_occ.transform and geom.origin are in the World (Root) context.
        # MuJoCo expects the joint position relative to the child body frame.

        is_child_occ_one = joint.occurrenceOne == child_occ
        geom = (
            joint.geometryOrOriginOne if is_child_occ_one else joint.geometryOrOriginTwo
        )

        # Get occurrence names for logging
        occ_one_name = joint.occurrenceOne.name if joint.occurrenceOne else "Root"
        occ_two_name = joint.occurrenceTwo.name if joint.occurrenceTwo else "Root"

        # 1. Get Child World Transform
        child_world = child_occ.transform
        child_world_inv = child_world.copy()
        child_world_inv.invert()

        # 2. Capture World Origin before transform
        s = self.length_scale
        world_origin = geom.origin.copy()
        world_origin_scaled = [world_origin.x * s, world_origin.y * s, world_origin.z * s]
        world_axis = geom.primaryAxisVector.copy()
        world_axis_vec = [world_axis.x, world_axis.y, world_axis.z]

        # 3. Transform Origin: World -> Child Local
        origin = geom.origin.copy()
        origin.transformBy(child_world_inv)
        pos_str = self.format_vec3(origin.x * s, origin.y * s, origin.z * s)
        local_origin_scaled = [origin.x * s, origin.y * s, origin.z * s]

        # 4. Transform Axis: World -> Child Local
        axis_vec = geom.primaryAxisVector.copy()
        axis_vec.transformBy(child_world_inv)

        # Normalize axis to ensure it's unit-length (required by MuJoCo)
        axis_mag = math.sqrt(axis_vec.x**2 + axis_vec.y**2 + axis_vec.z**2)
        if axis_mag > 0:
            axis_vec.x /= axis_mag
            axis_vec.y /= axis_mag
            axis_vec.z /= axis_mag

        axis_str = self.format_vec3(axis_vec.x, axis_vec.y, axis_vec.z)
        local_axis_vec = [axis_vec.x, axis_vec.y, axis_vec.z]

        # Log joint processing details (always, for debugging)
        self.log(f"  Processing joint '{joint.name}' ({fusion_type} -> {mj_type}):")
        self.log(f"    Connects: {occ_one_name} <-> {occ_two_name}")
        self.log(f"    Attached to: {child_occ.name}")
        self.log(f"    World origin: ({world_origin_scaled[0]:.1f}, {world_origin_scaled[1]:.1f}, {world_origin_scaled[2]:.1f})")
        self.log(f"    Local origin: ({local_origin_scaled[0]:.1f}, {local_origin_scaled[1]:.1f}, {local_origin_scaled[2]:.1f})")
        self.log(f"    Local axis: ({local_axis_vec[0]:.3f}, {local_axis_vec[1]:.3f}, {local_axis_vec[2]:.3f})")

        # Track joint processing in manifest
        self.manifest["joint_processing"].append({
            "joint": self.clean_name(joint.name),
            "fusion_type": fusion_type,
            "mujoco_type": mj_type,
            "occurrence_one": occ_one_name,
            "occurrence_two": occ_two_name,
            "attached_to": child_occ.name,
            "world_origin": world_origin_scaled,
            "world_axis": world_axis_vec,
            "local_origin": local_origin_scaled,
            "local_axis": local_axis_vec,
        })

        # 4. Limits
        extra_attrs = {}
        if mj_type == "hinge":
            rev_motion = adsk.fusion.RevoluteJointMotion.cast(motion)
            limits = rev_motion.rotationLimits
            if limits.isMinimumValueEnabled and limits.isMaximumValueEnabled:
                # Fusion is radians? Yes, internal units for angles are radians.
                extra_attrs["range"] = self.format_range(
                    limits.minimumValue, limits.maximumValue
                )
        elif mj_type == "slide":
            slide_motion = adsk.fusion.SliderJointMotion.cast(motion)
            limits = slide_motion.slideLimits
            if limits.isMinimumValueEnabled and limits.isMaximumValueEnabled:
                # Slide limits are in cm, scale to document units
                extra_attrs["range"] = self.format_range(
                    limits.minimumValue * s, limits.maximumValue * s
                )

        joint_name = self.clean_name(joint.name)
        ET.SubElement(
            body_elem,
            "joint",
            {
                "name": joint_name,
                "type": mj_type,
                "pos": pos_str,
                "axis": axis_str,
                **extra_attrs,
            },
        )

        # Track exported joint for actuator/sensor generation AND contact exclusion
        self.exported_joints.append(
            {
                "name": joint_name,
                "mj_type": mj_type,  # 'hinge' or 'slide'
                "has_limits": len(extra_attrs) > 0,
                "limits": extra_attrs.get("range", None),
                "parent_body": parent_body_name,
                "child_body": self.clean_name(child_occ.name),
            }
        )

        # Track joint in manifest
        # Determine parent name from the joint
        parent_occ = (
            joint.occurrenceTwo if joint.occurrenceOne == child_occ else joint.occurrenceOne
        )
        parent_name = self.clean_name(parent_occ.name) if parent_occ else "root"

        self.manifest["joints"].append(
            {
                "name": joint_name,
                "type": mj_type,
                "parent": parent_name,
                "child": self.clean_name(child_occ.name),
                "position": pos_str,
                "axis": axis_str,
                "limits": extra_attrs.get("range", None),
            }
        )

        if self.debug_mode:
            # DEBUG: Visual Sphere at Joint Location
            ET.SubElement(
                body_elem,
                "geom",
                {
                    "name": f"debug_joint_{self.clean_name(joint.name)}",
                    "type": "sphere",
                    "size": f"{5.0 * s:.6f}",  # Scale debug sphere too
                    "rgba": "1 0 0 1",
                    "pos": pos_str,
                },
            )

    def _add_contacts(self, root_elem):
        """Add contact exclusions for joint-connected bodies."""
        if not self.exported_joints:
            return

        # Insert after worldbody, before actuator/sensor
        # Standard MJCF order: worldbody, contact, actuator, sensor
        insert_idx = -1
        for i, child in enumerate(root_elem):
            if child.tag == "worldbody":
                insert_idx = i + 1
                break
        
        if insert_idx == -1:
            insert_idx = len(root_elem) # Append to end if worldbody not found (unlikely)

        contact_elem = ET.Element("contact")
        
        added_any = False
        for joint_info in self.exported_joints:
            p = joint_info.get("parent_body")
            c = joint_info.get("child_body")
            
            # If we have valid parent/child body names, exclude them
            if p and c and p != "world" and p != "worldbody":
                ET.SubElement(contact_elem, "exclude", {"body1": p, "body2": c})
                added_any = True
        
        if added_any:
            root_elem.insert(insert_idx, contact_elem)
            self.log(f"Generated {len(contact_elem)} contact exclusion(s) to prevent self-collision")

    def _add_default_section(self, root_elem):
        """Add MuJoCo default section with actuator parameters."""
        # Find insertion point (after compiler, before visual)
        compiler_idx = 0
        for i, child in enumerate(root_elem):
            if child.tag == "compiler":
                compiler_idx = i
                break

        default_elem = ET.Element("default")

        # Set defaults based on actuator type
        if self.actuator_type == "position":
            # PD control defaults (normalized control signal)
            ET.SubElement(default_elem, "position", {"kp": "100", "ctrlrange": "-1 1"})
        elif self.actuator_type == "velocity":
            # Velocity control defaults (control signal is desired velocity in rad/s or units/s)
            # Using reasonable velocity range for wheels/joints: ±20 rad/s (~3.2 rev/s)
            ET.SubElement(
                default_elem, "velocity", {"kv": "100", "ctrlrange": "-20 20"}
            )
        elif self.actuator_type == "motor":
            # Motor (torque/force) control defaults
            ET.SubElement(default_elem, "motor", {"ctrlrange": "-1 1", "gear": "1"})

        # Insert after compiler
        root_elem.insert(compiler_idx + 1, default_elem)

    def _add_actuators(self, root_elem):
        """Add actuator section with controls for all joints."""
        if not self.enable_actuators or not self.exported_joints:
            return

        actuator_elem = ET.SubElement(root_elem, "actuator")

        for joint_info in self.exported_joints:
            joint_name = joint_info["name"]
            has_limits = joint_info["has_limits"]
            limits = joint_info["limits"]

            # Generate actuator name
            actuator_suffix = {
                "position": "_pos",
                "velocity": "_vel",
                "motor": "_motor",
            }.get(self.actuator_type, "_act")

            actuator_name = f"{joint_name}{actuator_suffix}"

            # Base attributes
            attrs = {"name": actuator_name, "joint": joint_name}

            # Set control range based on actuator type and joint limits
            if self.actuator_type == "position":
                # Position actuators: use joint limits if available, else normalized range
                if has_limits and limits:
                    attrs["ctrlrange"] = limits
                else:
                    attrs["ctrlrange"] = "-1 1"
            elif self.actuator_type == "velocity":
                # Velocity actuators: control signal is desired velocity (rad/s or units/s)
                # Use reasonable velocity range (±20 rad/s for rotational, ±10 units/s for linear)
                if has_limits and limits:
                    # Could derive velocity from position limits, but just use default for now
                    attrs["ctrlrange"] = "-20 20"
                else:
                    attrs["ctrlrange"] = "-20 20"
            else:  # motor
                # Motor actuators: use normalized range
                attrs["ctrlrange"] = "-1 1"

            # Add appropriate actuator type
            ET.SubElement(actuator_elem, self.actuator_type, attrs)

            # Track actuator in manifest
            self.manifest["actuators"].append(
                {
                    "name": actuator_name,
                    "type": self.actuator_type,
                    "joint": joint_name,
                    "ctrlrange": attrs.get("ctrlrange"),
                }
            )

        self.log(
            f"Generated {len(self.exported_joints)} {self.actuator_type} actuator(s)"
        )

    def _add_sensors(self, root_elem):
        """Add sensor section with position and velocity sensors for all joints."""
        if not self.enable_sensors or not self.exported_joints:
            return

        sensor_elem = ET.SubElement(root_elem, "sensor")

        for joint_info in self.exported_joints:
            joint_name = joint_info["name"]

            # Add position sensor
            pos_sensor_name = f"{joint_name}_pos_sensor"
            ET.SubElement(
                sensor_elem,
                "jointpos",
                {"name": pos_sensor_name, "joint": joint_name},
            )

            # Add velocity sensor
            vel_sensor_name = f"{joint_name}_vel_sensor"
            ET.SubElement(
                sensor_elem,
                "jointvel",
                {"name": vel_sensor_name, "joint": joint_name},
            )

            # Track sensors in manifest
            self.manifest["sensors"].append(
                {"name": pos_sensor_name, "type": "jointpos", "joint": joint_name}
            )
            self.manifest["sensors"].append(
                {"name": vel_sensor_name, "type": "jointvel", "joint": joint_name}
            )

        self.log(
            f"Generated {len(self.exported_joints) * 2} sensor(s) ({len(self.exported_joints)} position + {len(self.exported_joints)} velocity)"
        )

    def save_manifest(self):
        """Generate and save YAML manifest with export metadata."""
        # Populate export metadata (no timestamp to avoid diffs)
        units_mgr = self.design.unitsManager
        self.manifest["export"] = {
            "document": self.root_comp.name,
            "units": units_mgr.defaultLengthUnits,
            "length_scale": self.length_scale,
        }

        # Populate inputs section
        self.manifest["inputs"] = {
            "export_path": self.export_path,
            "debug_mode": self.debug_mode,
            "actuators_enabled": self.enable_actuators,
            "actuator_type": self.actuator_type if self.enable_actuators else None,
            "sensors_enabled": self.enable_sensors,
        }

        # Add file references
        self.manifest["files"] = {
            "scene": "scene.xml",
            "robot": "bot.xml",
            "meshes": "meshes/",
            "log": "export.log",
        }

        # Write YAML manually (no PyYAML dependency)
        yaml_path = os.path.join(self.export_path, "manifest.yaml")

        with open(yaml_path, "w", encoding="utf-8") as f:
            f.write("# F3DToMuJoCo Export Manifest\n")
            f.write("# Generated automatically - compare with .xml and meshes/\n\n")

            # Export section
            f.write("export:\n")
            self._write_yaml_dict(f, self.manifest["export"], indent=2)

            # Files section
            f.write("\nfiles:\n")
            self._write_yaml_dict(f, self.manifest["files"], indent=2)

            # Inputs section
            f.write("\ninputs:\n")
            self._write_yaml_dict(f, self.manifest["inputs"], indent=2)

            # Model section
            if self.manifest.get("model"):
                f.write("\nmodel:\n")
                model = self.manifest["model"]
                f.write(f"  root_component: {self._yaml_str(model.get('root_component', ''))}\n")
                f.write(f"  total_components: {model.get('total_components', 0)}\n")
                f.write(f"  total_joints: {model.get('total_joints', 0)}\n")
                f.write(f"  total_bodies: {model.get('total_bodies', 0)}\n")

                if model.get("occurrences"):
                    f.write("  occurrences:\n")
                    for occ in model["occurrences"]:
                        f.write(f"    - name: {self._yaml_str(occ['name'])}\n")
                        f.write(f"      full_path: {self._yaml_str(occ['full_path'])}\n")
                        f.write(f"      world_position: [{self._format_float_list(occ['world_position'])}]\n")
                        f.write(f"      bounding_box_min: [{self._format_float_list(occ['bounding_box_min'])}]\n")
                        f.write(f"      bounding_box_max: [{self._format_float_list(occ['bounding_box_max'])}]\n")
                        f.write(f"      body_count: {occ['body_count']}\n")

            # Components section
            if self.manifest["components"]:
                f.write("\ncomponents:\n")
                for comp in self.manifest["components"]:
                    f.write(f"  - name: {self._yaml_str(comp['name'])}\n")
                    f.write(f"    parent: {self._yaml_str(comp.get('parent', '~'))}\n")
                    if comp.get("position"):
                        f.write(f"    position: [{comp['position']}]\n")
                    if comp.get("quaternion"):
                        f.write(f"    quaternion: [{comp['quaternion']}]\n")

            # Bodies (meshes) section
            if self.manifest["bodies"]:
                f.write("\nbodies:\n")
                for body in self.manifest["bodies"]:
                    f.write(f"  - name: {self._yaml_str(body['name'])}\n")
                    f.write(f"    component: {self._yaml_str(body['component'])}\n")
                    f.write(f"    mesh: {self._yaml_str(body['mesh'])}\n")
                    if body.get("mass") is not None:
                        f.write(f"    mass: {body['mass']:.6f}\n")
                    if body.get("center_of_mass"):
                        f.write(f"    center_of_mass: [{body['center_of_mass']}]\n")
                    if body.get("inertia"):
                        f.write(f"    inertia: [{body['inertia']}]\n")
                    if body.get("color"):
                        f.write(f"    color: [{body['color']}]\n")
                    if body.get("estimated_inertia"):
                        f.write(f"    estimated_inertia: true\n")

            # Joints section
            if self.manifest["joints"]:
                f.write("\njoints:\n")
                for joint in self.manifest["joints"]:
                    f.write(f"  - name: {self._yaml_str(joint['name'])}\n")
                    f.write(f"    type: {joint['type']}\n")
                    f.write(f"    parent: {self._yaml_str(joint['parent'])}\n")
                    f.write(f"    child: {self._yaml_str(joint['child'])}\n")
                    if joint.get("position"):
                        f.write(f"    position: [{joint['position']}]\n")
                    if joint.get("axis"):
                        f.write(f"    axis: [{joint['axis']}]\n")
                    if joint.get("limits"):
                        f.write(f"    limits: [{joint['limits']}]\n")

            # Actuators section
            if self.manifest["actuators"]:
                f.write("\nactuators:\n")
                for act in self.manifest["actuators"]:
                    f.write(f"  - name: {self._yaml_str(act['name'])}\n")
                    f.write(f"    type: {act['type']}\n")
                    f.write(f"    joint: {self._yaml_str(act['joint'])}\n")
                    if act.get("ctrlrange"):
                        f.write(f"    ctrlrange: [{act['ctrlrange']}]\n")

            # Sensors section
            if self.manifest["sensors"]:
                f.write("\nsensors:\n")
                for sensor in self.manifest["sensors"]:
                    f.write(f"  - name: {self._yaml_str(sensor['name'])}\n")
                    f.write(f"    type: {sensor['type']}\n")
                    f.write(f"    joint: {self._yaml_str(sensor['joint'])}\n")

            # Joint processing debug section
            if self.manifest.get("joint_processing"):
                f.write("\njoint_processing:\n")
                for jp in self.manifest["joint_processing"]:
                    f.write(f"  - joint: {self._yaml_str(jp['joint'])}\n")
                    f.write(f"    fusion_type: {jp['fusion_type']}\n")
                    f.write(f"    mujoco_type: {jp['mujoco_type']}\n")
                    f.write(f"    occurrence_one: {self._yaml_str(jp['occurrence_one'])}\n")
                    f.write(f"    occurrence_two: {self._yaml_str(jp['occurrence_two'])}\n")
                    f.write(f"    attached_to: {self._yaml_str(jp['attached_to'])}\n")
                    f.write(f"    world_origin: [{self._format_float_list(jp['world_origin'])}]\n")
                    f.write(f"    world_axis: [{self._format_float_list(jp['world_axis'])}]\n")
                    f.write(f"    local_origin: [{self._format_float_list(jp['local_origin'])}]\n")
                    f.write(f"    local_axis: [{self._format_float_list(jp['local_axis'])}]\n")

            # Appearance extraction debug section
            if self.manifest.get("appearance_extraction"):
                f.write("\nappearance_extraction:\n")
                for ae in self.manifest["appearance_extraction"]:
                    f.write(f"  - body: {self._yaml_str(ae['body'])}\n")
                    f.write(f"    source: {ae['source']}\n")
                    if ae.get("appearance_name"):
                        f.write(f"    appearance_name: {self._yaml_str(ae['appearance_name'])}\n")
                    if ae.get("property_used"):
                        f.write(f"    property_used: {ae['property_used']}\n")
                    f.write(f"    rgba: [{self._format_float_list(ae['rgba'])}]\n")
                    if ae.get("error"):
                        f.write(f"    error: {self._yaml_str(ae['error'])}\n")

            # Warnings section
            if self.manifest["warnings"]:
                f.write("\nwarnings:\n")
                for warning in self.manifest["warnings"]:
                    f.write(f"  - {self._yaml_str(warning)}\n")

        self.log(f"Saved manifest to {yaml_path}")

    def _write_yaml_dict(self, f, d, indent=0):
        """Write a dictionary as YAML with given indentation."""
        prefix = " " * indent
        for key, value in d.items():
            if value is None:
                f.write(f"{prefix}{key}: ~\n")
            elif isinstance(value, bool):
                f.write(f"{prefix}{key}: {str(value).lower()}\n")
            elif isinstance(value, (int, float)):
                f.write(f"{prefix}{key}: {value}\n")
            elif isinstance(value, str):
                f.write(f"{prefix}{key}: {self._yaml_str(value)}\n")
            elif isinstance(value, list):
                f.write(f"{prefix}{key}: [{', '.join(str(v) for v in value)}]\n")
            else:
                f.write(f"{prefix}{key}: {value}\n")

    def _yaml_str(self, s):
        """Escape a string for YAML output."""
        if s is None:
            return "~"
        s = str(s)
        # Quote if contains special chars
        if any(c in s for c in ":#[]{}|>&*!?,\\\"'"):
            return f'"{s}"'
        return s

    def _format_float_list(self, values, decimals=6):
        """Format a list of floats for YAML output."""
        return ", ".join(f"{v:.{decimals}f}" for v in values)

    def close_log_file(self):
        """Close the log file if it's open."""
        if self.log_file:
            try:
                self.log_file.close()
            except Exception:
                pass
            self.log_file = None


class ExportCommandExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            command = args.firingEvent.sender
            inputs = command.commandInputs

            # 1. Open Folder Dialog to select output location
            folderDlg = _ui.createFolderDialog()
            folderDlg.title = "Select Output Directory for MuJoCo Export"
            res = folderDlg.showDialog()

            if res == adsk.core.DialogResults.DialogOK:
                export_path = folderDlg.folder

                # Read Debug Mode Input
                debug_input = inputs.itemById("debug_mode")
                debug_mode = debug_input.value if debug_input else False

                # Read actuator options
                enable_actuators_input = inputs.itemById("enable_actuators")
                enable_actuators = (
                    enable_actuators_input.value if enable_actuators_input else True
                )

                actuator_type_input = inputs.itemById("actuator_type")
                actuator_type = "position"  # default
                if actuator_type_input:
                    idx = actuator_type_input.selectedItem.index
                    actuator_type = ["position", "velocity", "motor"][idx]

                enable_sensors_input = inputs.itemById("enable_sensors")
                enable_sensors = (
                    enable_sensors_input.value if enable_sensors_input else True
                )

                exporter = Exporter(
                    export_path,
                    debug_mode,
                    enable_actuators,
                    actuator_type,
                    enable_sensors,
                )
                exporter.export_all()
            else:
                _ui.messageBox("Export cancelled.")

        except Exception:
            if _ui:
                _ui.messageBox("Failed:\n{}".format(traceback.format_exc()))


class ExportCommandDestroyHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            # When the command is done, terminate the script
            # This releases the python script from memory
            adsk.terminate()
        except Exception:
            if _ui:
                _ui.messageBox("Failed:\n{}".format(traceback.format_exc()))


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

            # Define Inputs
            inputs = cmd.commandInputs
            inputs.addTextBoxCommandInput(
                "info",
                "",
                "Click OK to select the export folder and begin the process.",
                2,
                True,
            )

            # Add Debug Mode Checkbox
            inputs.addBoolValueInput(
                "debug_mode", "Debug Mode (Visual Spheres)", True, "", False
            )

            # Add Control Options Header
            inputs.addTextBoxCommandInput(
                "actuator_header", "", "<b>Control Options</b>", 1, True
            )

            # Actuators checkbox
            inputs.addBoolValueInput(
                "enable_actuators", "Generate Actuators", True, "", True
            )

            # Actuator type dropdown
            actuator_dropdown = inputs.addDropDownCommandInput(
                "actuator_type",
                "Actuator Type",
                adsk.core.DropDownStyles.LabeledIconDropDownStyle,
            )
            actuator_dropdown.listItems.add("Position (PD Control)", True, "")
            actuator_dropdown.listItems.add("Velocity", False, "")
            actuator_dropdown.listItems.add("Motor (Torque/Force)", False, "")

            # Sensors checkbox
            inputs.addBoolValueInput(
                "enable_sensors", "Generate Sensors", True, "", True
            )

        except Exception:
            if _ui:
                _ui.messageBox("Failed:\n{}".format(traceback.format_exc()))


def run(context):
    global _app, _ui
    try:
        _app = adsk.core.Application.get()
        _ui = _app.userInterface

        # PREVENT EARLY TERMINATION
        adsk.autoTerminate(False)

        # Create the command definition
        cmdDef = _ui.commandDefinitions.itemById(CMD_ID)
        if not cmdDef:
            cmdDef = _ui.commandDefinitions.addButtonDefinition(
                CMD_ID, CMD_NAME, CMD_Description, ""
            )

        # Connect to the command created event
        onCommandCreated = ExportCommandCreatedHandler()
        cmdDef.commandCreated.add(onCommandCreated)
        _handlers.append(onCommandCreated)

        # Execute the command immediately
        cmdDef.execute()

    except Exception:
        if _ui:
            _ui.messageBox("Failed:\n{}".format(traceback.format_exc()))


def stop(context):
    try:
        # Clean up the UI
        cmdDef = _ui.commandDefinitions.itemById(CMD_ID)
        if cmdDef:
            cmdDef.deleteMe()
    except Exception:
        if _ui:
            _ui.messageBox("Failed:\n{}".format(traceback.format_exc()))
