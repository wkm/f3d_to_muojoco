# F3DToMuJoCo

A Fusion 360 Add-in to export designs directly to the MuJoCo MJCF (`.xml`) format, including meshes and kinematic definitions.

## Installation and Usage

1.  **Locate the Add-in**: Ensure the `F3DToMuJoCo` folder is accessible on your system.
2.  **Add to Fusion 360**:
    *   Open Fusion 360.
    *   Press `Shift + S` (or go to `Utilities > Scripts and Add-ins`).
    *   Select the **Add-Ins** tab.
    *   Click the **+** (Add) button under "My Add-Ins".
    *   Select the `F3DToMuJoCo` folder.
3.  **Run**:
    *   Select `F3DToMuJoCo` from the list and click **Run**.
    *   Select an output folder when prompted.
    *   The MJCF `.xml` file and `meshes/` folder will be created in the selected directory.

## Project Overview

This tool aims to streamline the workflow from CAD (Fusion 360) to Simulation (MuJoCo). It functions similarly to existing URDF exporters but targets the specific features and structure of MJCF.

## Modeling Guidelines

To ensure a successful export, follow these best practices in Fusion 360:

### 1. Naming Conventions
*   **Uniqueness**: Ensure every **Component** in your browser tree has a unique name. Duplicate names will cause STL files to overwrite each other.
*   **Characters**: Use `snake_case` (e.g., `base_link`, `arm_segment_1`). Avoid spaces, colons, and special symbols like `#`, `(`, `)`, or `/`.
*   **Root Name**: The name of the Root Component will be used as the MuJoCo model name.

### 2. Assembly Structure
*   **Components vs. Bodies**: The exporter creates a MuJoCo `<body>` for every **Component**. Multiple *Bodies* inside a single component will be merged into a single STL/Geom.
*   **Hierarchy**: The exporter respects your assembly hierarchy. Nested components in Fusion will become nested `<body />` elements in MJCF.

### 3. Kinematics (Joints)
*   **Supported Joints**: Use **Revolute** (mapped to `hinge`) and **Slider** (mapped to `slide`) joints.
*   **Rigid Groups**: Components that are rigidly attached (no joint) will be "welded" in MuJoCo by being nested without a `<joint>` tag.
*   **Anchor Points**: The joint origin in Fusion is used as the `pos` of the `<joint>` in MuJoCo. Ensure your joint origins are precisely placed.

## Architecture

The project is structured as a standard Fusion 360 Python Script/Add-in.

```text
F3DToMuJoCo/
├── F3DToMuJoCo.manifest  # Plugin metadata (UUID, version, author)
├── F3DToMuJoCo.py        # Entry point and main logic
└── resources/            # Icons and UI assets
```

## Technical Approach

### 1. The Export Workflow
1.  **UI Trigger**: The user clicks a button in the Fusion 360 "Utilities" or "Tools" tab.
2.  **Configuration**: A dialog asks for the export directory and optional settings (e.g., mesh precision).
3.  **Traversal**: The script recursively traverses the design hierarchy (Root Component -> Occurrences).
4.  **Mesh Export**: Each unique component is exported as an `.stl` file to a `meshes/` subdirectory.
5.  **MJCF Generation**: An XML tree is constructed in memory mirroring the assembly structure.
6.  **Final Write**: The `.xml` file and meshes are saved to the target directory.

### 2. Kinematic Mapping (Fusion -> MJCF)

| Fusion 360 Concept | MuJoCo MJCF Concept | Implementation Details |
| :--- | :--- | :--- |
| **Component** | `<body>` | Nested within `worldbody` or parent bodies. |
| **Body (Geometry)** | `<geom>` | Type `mesh`. References exported STLs. |
| **Joint (Revolute)** | `<joint type="hinge">` | Axis and limits extracted from Fusion joint motion. |
| **Joint (Slider)** | `<joint type="slide">` | Axis and limits extracted from Fusion joint motion. |
| **Rigid Group** | Parent/Child (No Joint) | In MJCF, bodies nested without a joint are rigidly attached. |

### 3. Coordinate Systems
*   **Fusion 360**: Defaults to Y-up (usually), but can be Z-up.
*   **MuJoCo**: strictly Z-up.
*   **Strategy**: The exporter will check the Fusion document's "Up Axis". If it is Y-up, a root rotation (typically -90 deg on X) might be applied to the `worldbody`, or we will transform coordinates during extraction.

### 4. Handling Joints
Fusion 360 defines joints as relationships between two components. MJCF defines joints as degrees of freedom *within* a body relative to its parent.
*   **Algorithm**:
    1.  Traverse the occurrence tree.
    2.  For each occurrence, check if it is the "Child" in any Fusion Joint.
    3.  If a joint exists, extract its type (Revolute/Slider), axis, and anchor point relative to the component's origin.
    4.  Insert the `<joint>` element into the corresponding MJCF `<body>`.

### 5. Physical Properties & Materials
*   **Inertia & Mass**: We will query the Fusion API for `physicalProperties` (mass, CoM, moment of inertia tensor) and generate an explicit `<inertial>` tag for each body. This is more accurate than letting MuJoCo approximate it from meshes.
*   **Visuals**: We will extract the RGB color from the component's **Appearance** and apply it to the MJCF `<geom>`.
*   **Collision**: For the MVP, we will use the visual meshes for collision. Future versions may support a "collision_*" naming convention to separate visual from collision geometry.
*   **Friction/Damping**: Fusion does not provide simulation-ready friction coefficients. We will apply reasonable defaults (e.g., standard MuJoCo friction, small damping) to prevent unstable simulations.

## Development Roadmap

1.  **Scaffold**: Create the basic Add-in structure and Manifest.
2.  **UI**: Implement a basic command button and "Select Folder" dialog.
3.  **Mesh Export**: Implement the loop to export all leaf components as STLs.
4.  **Tree Builder**: Implement the recursive XML builder for the component hierarchy.
5.  **Joint logic**: precise math to extract joint origins and axes in the correct frame.
6.  **Testing**: Verify against a simple 2-link arm and a mobile base.

## References
*   [Fusion 360 API Documentation](https://help.autodesk.com/view/fusion360/ENU/?guid=GUID-A92A4B10-3781-4925-94C6-47DA85A4F65A)
*   [MuJoCo XML Reference](https://mujoco.readthedocs.io/en/latest/XMLreference.html)