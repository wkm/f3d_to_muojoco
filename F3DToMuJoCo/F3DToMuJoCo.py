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
        # We must scale API values to match the Document Units.
        units_mgr = self.design.unitsManager
        self.length_scale = units_mgr.convert(
            1, units_mgr.internalUnits, units_mgr.defaultLengthUnits
        )

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

                # Detailed logging only in debug mode
                if self.debug_mode:
                    try:
                        o1 = joint.occurrenceOne
                        o2 = joint.occurrenceTwo
                        path1 = o1.fullPathName if o1 else "Root"
                        path2 = o2.fullPathName if o2 else "Root"
                        self.log(
                            f"  Found joint '{joint.name}' connecting {path1} → {path2}"
                        )
                    except Exception:
                        pass  # Ignore errors in debug logging
            self.log(f"Found {len(self.all_joints)} joint(s) in assembly")
        except Exception as e:
            self.log(f"Warning: Failed to collect joints: {type(e).__name__}: {e}")

    def log(self, message):
        try:
            print(f"[F3DToMuJoCo] {message}")
            # Write to the Text Commands palette
            text_palette = self.app.userInterface.palettes.itemById("TextCommands")
            if text_palette:
                text_palette.writeText(f"[F3DToMuJoCo] {message}")
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

            self.log("Export completed successfully")
            _ui.messageBox("Export Complete!")

        except Exception as e:
            self.log(f"Export failed: {str(e)}")
            _ui.messageBox(f"Export Failed:\n{str(e)}")
            traceback.print_exc()
        finally:
            progress.hide()

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
        return f"{ixx:.{decimals}f} {iyy:.{decimals}f} {izz:.{decimals}f} {ixy:.{decimals}f} {ixz:.{decimals}f} {iyz:.{decimals}f}"

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
        root_elem = ET.Element("mujoco", {"model": self.root_comp.name})

        # Add basic compiler and asset settings
        ET.SubElement(root_elem, "compiler", {"angle": "radian", "meshdir": "meshes"})

        # Add default section (if actuators enabled)
        if self.enable_actuators:
            self._add_default_section(root_elem)

        # Add visual settings for better lighting
        visual = ET.SubElement(root_elem, "visual")
        ET.SubElement(
            visual,
            "headlight",
            {"ambient": ".4 .4 .4", "diffuse": ".8 .8 .8", "specular": "0.1 0.1 0.1"},
        )
        ET.SubElement(visual, "map", {"znear": "0.01"})
        ET.SubElement(visual, "quality", {"shadowsize": "2048"})

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

                # Fusion exports STLs in cm. We use them as-is (unitless/consistent).
                ET.SubElement(
                    asset, "mesh", {"name": mesh_name, "file": f"{mesh_name}.stl"}
                )

        # Worldbody and recursively add bodies
        worldbody = ET.SubElement(root_elem, "worldbody")

        # Add a floor and directional light
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
            {"name": "floor", "type": "plane", "size": "5 5 0.1", "rgba": ".9 .9 .9 1"},
        )

        # Add skybox and ground textures/materials
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

        # Initialize recursion with Identity matrix (World Frame)
        identity_transform = adsk.core.Matrix3D.create()

        # ROOT WRAPPER FOR COORDINATE CORRECTION
        # Fusion 360 often defaults to Y-Up. MuJoCo is Z-Up.
        # Since <compiler angle="radian"> is set, we use math.pi/2 (90 degrees).
        root_rot = math.pi / 2
        root_adapter = ET.SubElement(
            worldbody,
            "body",
            {"name": "root_adapter", "pos": "0 0 0", "euler": f"{root_rot} 0 0"},
        )

        # Recursively process occurrences, attaching them to the ADAPTER
        for occ in self.root_comp.occurrences:
            # The parent of these top-level occurrences is the Root Component
            self.process_occurrence(
                occ, root_adapter, identity_transform, self.root_comp
            )

        # Add actuators and sensors (after worldbody is complete)
        if self.enable_actuators:
            self._add_actuators(root_elem)
        if self.enable_sensors:
            self._add_sensors(root_elem)

        # Write to file
        xml_path = os.path.join(
            self.export_path, f"{self.clean_name(self.root_comp.name)}.xml"
        )
        tree = ET.ElementTree(root_elem)
        # Indent for pretty printing
        ET.indent(tree, space="  ", level=0)
        tree.write(xml_path, encoding="utf-8", xml_declaration=True)

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

            ET.SubElement(
                body, "geom", {"type": "mesh", "mesh": mesh_name, "rgba": rgba_str}
            )

        # Process children
        for child_occ in occ.childOccurrences:
            self.process_occurrence(child_occ, body, current_world_transform, occ)

    def get_body_appearance_rgba(self, occ, body):
        # Emergency Orange (Fallback)
        default_rgba = "1.0 0.5 0.0 1.0"

        # 1. Check Body Override (Highest Priority for granular visuals)
        app = body.appearance

        # 2. Check Occurrence Override
        if not app:
            app = occ.appearance

        # 3. Check Physical Material Appearance
        if not app and occ.component.material:
            app = occ.component.material.appearance

        if not app:
            return default_rgba

        color_val = self.find_color_in_appearance(app)
        if color_val:
            return f"{color_val.red / 255.0} {color_val.green / 255.0} {color_val.blue / 255.0} {color_val.opacity / 255.0}"

        if self.debug_mode:
            self.log(
                f"Warning: Could not extract color from appearance '{app.name}' on {occ.name}"
            )
        return default_rgba

    def find_color_in_appearance(self, app):
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
                    return color_prop.value

        # 2. Fallback: Search ALL properties for ANY ColorProperty
        for prop in app.appearanceProperties:
            if prop.constructor.name == "adsk::core::ColorProperty":
                color_prop = adsk.core.ColorProperty.cast(prop)
                if color_prop and color_prop.value:
                    return color_prop.value

        return None

    def process_inertial(self, comp, body_elem):
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
        except Exception as e:
            # Fallback if physical properties fail (e.g. empty component)
            if self.debug_mode:
                self.log(
                    f"Warning: Could not extract inertial properties for {comp.name}: {type(e).__name__}"
                )

    def get_token(self, obj):
        # Safely get entityToken for comparison
        try:
            return obj.entityToken if obj else "ROOT"
        except Exception:
            return "UNKNOWN"

    def process_joints(self, occ, body_elem, target_parent):
        # Look for a joint that connects 'occ' to 'target_parent'
        if self.debug_mode:
            target_name = (
                target_parent.fullPathName
                if hasattr(target_parent, "fullPathName")
                else "Root"
            )
            self.log(f"Processing joints for {occ.name} (parent: {target_name})")

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
                if self.debug_mode:
                    self.log(f"  Found joint '{joint.name}' for {occ.name}")
                self.add_joint_to_xml(joint, occ, body_elem)
                found_joint = True
                break

        if not found_joint and self.debug_mode:
            self.log(f"  No joint found for {occ.name} (will be rigidly attached)")

    def add_joint_to_xml(self, joint, child_occ, body_elem):
        motion = joint.jointMotion

        mj_type = ""
        if motion.jointType == adsk.fusion.JointTypes.RevoluteJointType:
            mj_type = "hinge"
        elif motion.jointType == adsk.fusion.JointTypes.SliderJointType:
            mj_type = "slide"
        else:
            return  # Rigid, Ball, etc. not handled yet

        # Strategy: Use World-to-World transformation.
        # Since we are using joint proxies from the root component,
        # both child_occ.transform and geom.origin are in the World (Root) context.
        # MuJoCo expects the joint position relative to the child body frame.

        is_child_occ_one = joint.occurrenceOne == child_occ
        geom = (
            joint.geometryOrOriginOne if is_child_occ_one else joint.geometryOrOriginTwo
        )

        # 1. Get Child World Transform
        child_world = child_occ.transform
        child_world_inv = child_world.copy()
        child_world_inv.invert()

        # 2. Transform Origin: World -> Child Local
        origin = geom.origin.copy()
        origin.transformBy(child_world_inv)

        s = self.length_scale
        pos_str = self.format_vec3(origin.x * s, origin.y * s, origin.z * s)

        # 3. Transform Axis: World -> Child Local
        axis_vec = geom.primaryAxisVector.copy()
        axis_vec.transformBy(child_world_inv)

        # Normalize axis to ensure it's unit-length (required by MuJoCo)
        axis_mag = math.sqrt(axis_vec.x**2 + axis_vec.y**2 + axis_vec.z**2)
        if axis_mag > 0:
            axis_vec.x /= axis_mag
            axis_vec.y /= axis_mag
            axis_vec.z /= axis_mag

        axis_str = self.format_vec3(axis_vec.x, axis_vec.y, axis_vec.z)

        # Detailed joint position debugging
        if self.debug_mode:
            try:
                bb = child_occ.boundingBox
                self.log(f"  Joint '{joint.name}' on {child_occ.name}:")
                self.log(
                    f"    Bounding box: ({bb.minPoint.x * s:.1f}, {bb.minPoint.y * s:.1f}, {bb.minPoint.z * s:.1f}) to ({bb.maxPoint.x * s:.1f}, {bb.maxPoint.y * s:.1f}, {bb.maxPoint.z * s:.1f})"
                )
                self.log(f"    Local position: {pos_str}")

                # Transform Local Pos back to World for comparison
                check_pt = origin.copy()
                check_pt.transformBy(child_world)  # Local -> World
                self.log(
                    f"    World position: ({check_pt.x * s:.1f}, {check_pt.y * s:.1f}, {check_pt.z * s:.1f})"
                )
            except Exception:
                pass  # Ignore errors in debug logging

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

        # Track exported joint for actuator/sensor generation
        self.exported_joints.append(
            {
                "name": joint_name,
                "mj_type": mj_type,  # 'hinge' or 'slide'
                "has_limits": len(extra_attrs) > 0,
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
            # PD control defaults
            ET.SubElement(default_elem, "position", {"kp": "100", "ctrlrange": "-1 1"})
        elif self.actuator_type == "velocity":
            # Velocity control defaults
            ET.SubElement(default_elem, "velocity", {"kv": "10", "ctrlrange": "-1 1"})
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

            # Set control range
            if has_limits and limits:
                # Use joint limits for control range
                attrs["ctrlrange"] = limits
            else:
                # Use default range
                attrs["ctrlrange"] = "-1 1"

            # Add appropriate actuator type
            ET.SubElement(actuator_elem, self.actuator_type, attrs)

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
            ET.SubElement(
                sensor_elem,
                "jointpos",
                {"name": f"{joint_name}_pos_sensor", "joint": joint_name},
            )

            # Add velocity sensor
            ET.SubElement(
                sensor_elem,
                "jointvel",
                {"name": f"{joint_name}_vel_sensor", "joint": joint_name},
            )

        self.log(
            f"Generated {len(self.exported_joints) * 2} sensor(s) ({len(self.exported_joints)} position + {len(self.exported_joints)} velocity)"
        )


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
