# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**F3DToMuJoCo** is a Fusion 360 Add-in that exports CAD designs to MuJoCo MJCF format (`.xml`), a physics simulation format. The exporter handles complex nested assemblies, extracts kinematics (joints), physics properties (mass, inertia), and visual meshes (STL files with colors).

## Development Environment

This is a **Fusion 360 Python Add-in**, not a standalone Python project. The code runs inside Fusion 360's embedded Python environment using the Autodesk Fusion API.

### Running/Testing the Add-in

1. Open Fusion 360
2. Press `Shift + S` (or go to `Utilities > Scripts and Add-ins`)
3. Select the **Scripts** tab
4. Click the green **+ (Add)** button and select the `F3DToMuJoCo` folder
5. Select `F3DToMuJoCo` from the list and click **Run**

### Debugging

- **Logs**: All `self.log()` calls write to Fusion 360's **Text Commands** palette (`View > Show Text Commands` or `Alt+CMD+C`)
- **VS Code Debugging**: A `.vscode/launch.json` is configured for remote debugging on port 9000
- **Debug Mode**: The UI includes a "Debug Mode" checkbox that adds visual red spheres at joint locations in the exported MJCF

## Architecture

### Single-File Design

The entire exporter is contained in `F3DToMuJoCo/F3DToMuJoCo.py` (~730 lines). This is intentional for Fusion 360 add-in distribution.

### Core Classes

- **`Exporter`**: Main export logic. Initialized with an export path and optional debug mode.
- **Command Handlers** (`ExportCommandCreatedHandler`, `ExportCommandExecuteHandler`, `ExportCommandDestroyHandler`): Fusion 360 event handlers for the UI workflow.

### Key Export Flow

1. **Validation** (`validate_design()`): Checks for duplicate component names and flat hierarchies (sibling joints)
2. **Mesh Export** (`save_meshes()`): Exports each BRep body as a binary STL file to `meshes/` subdirectory
3. **MJCF XML Build** (`build_xml()`): Recursively constructs the MuJoCo XML tree
4. **Recursive Traversal** (`process_occurrence()`): Walks the Fusion assembly hierarchy, computing relative transforms for each body

### Critical Transform Logic

**Coordinate Systems:**

- **Fusion API** returns values in **centimeters (cm)** internally
- **Document Units** (e.g., mm, inches) are used for STL export
- **`self.length_scale`** converts API values (cm) to document units to match STL coordinates
- **Fusion is Y-Up, MuJoCo is Z-Up**: A `root_adapter` body with 90° rotation is added to the MJCF worldbody

**Transform Chain:**

- `process_occurrence()` uses **World-to-Local** transforms (F3DToMuJoCo.py:360-406)
- `parent_world_transform` tracks each parent's world transform
- Relative transform: `parent_inv * current_world_transform`
- Both position and quaternion are extracted from the relative transform

**Joint Transforms:**

- Joints are collected from `root_comp.allJoints` (proxies in World context)
- Joint origin and axis are transformed from World → Child Local frame (F3DToMuJoCo.py:549-573)
- Uses `child_world_inv` to convert joint geometry to the child body's local coordinates

### Joint Matching Strategy

The exporter matches joints to occurrences using **entity tokens** (`entityToken` property):

- `_collect_all_joints()` (F3DToMuJoCo.py:39) collects all joints from the root context
- `process_joints()` (F3DToMuJoCo.py:499) searches for a joint connecting the current occurrence to its parent
- Token-based matching handles `None` references to the root component

### Color Extraction

Priority order for appearance colors (F3DToMuJoCo.py:408-461):

1. Body appearance override (highest priority)
2. Occurrence appearance override
3. Component physical material appearance
4. Fallback: "Emergency Orange" (RGB: 1.0, 0.5, 0.0)

Property names searched: `color_base`, `Color`, `opaque_albedo`, `metal_f0`

### Physics Properties

- **Mass & Inertia**: Extracted via `comp.physicalProperties` (F3DToMuJoCo.py:463-490)
- **Center of Mass**: Relative to the component's local coordinate system
- **Inertia Tensor**: Scaled by `length_scale²` (inertia scales with length squared)
- **Units**: Mass in kg, inertia in kg·(document_units)²

## Common Pitfalls

### Duplicate Component Names

Will cause mesh files to overwrite each other. The exporter validates this before export.

### Flat Hierarchies (Sibling Joints)

Joints between components at the same hierarchy level may not export correctly. The exporter expects a nested parent-child structure matching the kinematic chain. Validation warns about this.

### Unit Scaling Issues

The recent commit history shows extensive work on unit scaling:

- Always use `self.length_scale` when converting positions/dimensions from API values
- Inertia requires `self.length_scale²` scaling
- Joint limits: revolute (already in radians), slider (needs length scaling)

### Transform Bugs

When joints appear "exploded" or misplaced:

- Verify the World-to-Local transform chain is correct
- Check that `parent_world_transform` is properly threaded through recursion
- Use Debug Mode to visualize joint locations with red spheres

## Modeling Guidelines for Users

### Naming

- Use unique `snake_case` component names
- Avoid spaces and special characters
- Root component name becomes the MJCF model name

### Joints

- Define all motion using Fusion 360 Joints (Revolute, Slider)
- Joints should connect child to parent, not siblings
- Supported types: Revolute (→ hinge), Slider (→ slide)

### Physics

- Assign physical materials (e.g., Aluminum, ABS) for correct mass/inertia
- Empty components without bodies are skipped

### Visuals

- Colors extracted from appearances or materials
- Each BRep body becomes a separate mesh and geom element
