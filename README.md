# F3DToMuJoCo

A Fusion 360 Add-in that exports CAD designs to **MuJoCo MJCF** format (`.xml`). It extracts meshes, joints, physics properties, and colors from your Fusion assembly and produces a ready-to-simulate MuJoCo model.

## What It Does

The exporter walks your Fusion 360 assembly hierarchy and produces:

- **MJCF XML file** with bodies, joints, and inertial properties
- **STL meshes** for each body in your design
- **A default scene** with floor, lighting, and skybox

It handles coordinate system differences (Fusion is Y-up, MuJoCo is Z-up) and unit conversions automatically.

## Output Structure

```
output_folder/
├── YourModelName.xml           # MuJoCo MJCF file
└── meshes/
    ├── Component1_Body1.stl    # One STL per body
    ├── Component1_Body2.stl
    └── Component2_Body1.stl
```

Each BRep body in your design becomes a separate mesh file, named `ComponentName_BodyName.stl`.

## Supported Features

| Fusion 360 | MuJoCo |
|------------|--------|
| Revolute Joint | `<joint type="hinge">` |
| Slider Joint | `<joint type="slide">` |
| Joint Limits | `range` attribute |
| Physical Materials | `<inertial>` (mass, CoM, inertia tensor) |
| Appearances/Colors | `rgba` on `<geom>` |
| Nested Assemblies | Nested `<body>` elements |

## Installation

1. Clone or download this repository
2. In Fusion 360, press `Shift + S` (or `Utilities > Scripts and Add-ins`)
3. In the **Scripts** tab, click **+ (Add)** and select the `F3DToMuJoCo` folder
4. Select `F3DToMuJoCo` and click **Run**
5. Choose an output folder when prompted

### Debug Mode

The export dialog includes a "Debug Mode" checkbox. When enabled, red spheres are added at each joint location in the exported MJCF—useful for verifying joint placement.

## Modeling Guidelines

### Naming

- **Unique component names required.** Duplicate names cause mesh files to overwrite each other. The exporter will warn you before export if duplicates are found.
- Use `snake_case` (e.g., `base_link`, `shoulder_motor`). Avoid spaces and special characters.
- The root component name becomes the MJCF model name.

### Assembly Structure

- **Nest children inside parents.** The exporter expects your kinematic chain to match the component hierarchy. Joints between sibling components (same level in the browser) may not export correctly—the exporter will warn you about this.
- Components without joints are treated as rigidly attached to their parent.

### Joints

- Use Fusion 360's **Revolute** or **Slider** joints to define motion
- Enable joint limits in Fusion if you want `range` attributes in the MJCF
- Other joint types (Rigid, Ball, etc.) are not currently exported

### Physics

- Assign physical materials (e.g., Aluminum, ABS) for correct mass and inertia values
- Mass is exported in kg; inertia uses your document's length units squared

### Colors

Colors are extracted in priority order:
1. Body appearance override
2. Occurrence appearance override
3. Component physical material appearance
4. Fallback: "Emergency Orange" (`rgba="1 0.5 0 1"`)

If your model appears orange in MuJoCo, the exporter couldn't find a color property.

## Unit Handling

The exporter automatically handles Fusion 360's unit system:

- **Fusion API** returns values in centimeters internally
- **STL meshes** are exported in your document's units (mm, inches, etc.)
- **Positions and dimensions** in the MJCF are scaled to match your document units
- **Inertia tensors** are scaled appropriately (length² scaling)

This means the MJCF and meshes will be consistent with whatever units you're using in Fusion.

## Troubleshooting

### Logs

The exporter writes to Fusion 360's **Text Commands** palette. Open it with `View > Show Text Commands` (or `Alt+Cmd+C` on Mac).

### Pre-Export Validation

Before exporting, the add-in checks for common issues:

- **Duplicate component names** (critical): Will cause mesh files to overwrite each other
- **Sibling joints** (warning): Joints between components at the same hierarchy level may not export correctly

You'll see a dialog if issues are found, with the option to continue anyway.

### Common Issues

| Problem | Cause | Fix |
|---------|-------|-----|
| Model appears orange | No appearance/material found | Assign an appearance or physical material |
| Joints in wrong position | Sibling joint or hierarchy mismatch | Nest child components inside parents |
| Model orientation wrong | Y-up vs Z-up | Edit `root_adapter` euler angles in the XML |
| Missing inertia | No physical material | Assign a material (Aluminum, ABS, etc.) |

## License

MIT License
