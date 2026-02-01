# F3DToMuJoCo

A robust Fusion 360 Add-in that exports your CAD designs directly to **MuJoCo MJCF** format (`.xml`), complete with meshes, kinematics, physics, and visuals.

## Features

- **Recursive Export**: Handles complex nested component hierarchies.
- **Kinematics**:
  - Automatically extracts **Revolute (Hinge)** and **Slider (Slide)** joints.
  - Preserves joint limits and axes in the correct local coordinate frames.
  - Supports Relative Transforms for deeply nested assemblies.
- **Physics**:
  - Extracts **Mass**, **Center of Mass**, and **Inertia Tensors** directly from Fusion's physical properties.
- **Visuals**:
  - Exports binary **STLs** for each component.
  - Extracts **RGB Colors** from Fusion Appearances (supports Occurrences, Materials, and Bodies).
  - Sets up a default MJCF scene with lighting and a floor.
- **Coordinate Correction**:
  - Automatically rotates models to align Fusion 360's default **Y-Up** with MuJoCo's **Z-Up**.

## Installation

1.  **Download**: Clone or download this repository to a safe location on your computer.
2.  **Add to Fusion 360**:
    - Open Fusion 360.
    - Press `Shift + S` (or go to `Utilities > Scripts and Add-ins`).
    - Select the **Scripts** tab.
    - Click the green **+ (Add)** button.
    - Select the `F3DToMuJoCo` folder (the folder containing the `.manifest` file).
3.  **Run**:
    - Select `F3DToMuJoCo` from the list and click **Run**.
    - Follow the prompts to select an output folder.

## Modeling Guidelines

To ensure the best results, follow these practices in Fusion 360:

### 1. Naming & Structure

- **Unique Names**: Ensure every Component has a unique name. Duplicate names can cause mesh overwrites.
- **Valid Characters**: Use `snake_case` (e.g., `base_link`, `shoulder_motor`). Avoid spaces and special characters.
- **Root Name**: The Root Component name becomes the name of your MJCF model.

### 2. Joints

- **Define in Fusion**: Use standard Fusion 360 Joints (Revolute, Slider) to define motion.
- **Rigid Groups**: Parts that don't move relative to each other should be separate components or rigidly joined. The exporter treats parent-child relationships without joints as rigid connections.

### 3. Visuals & Physics

- **Physical Materials**: Assign materials (e.g., Aluminum, ABS) to your components to ensure correct Mass and Inertia values.
- **Appearances**: Colors are extracted from the component's appearance or material. If your model appears "Emergency Orange" in MuJoCo, it means the exporter couldn't find a valid color property.

## Troubleshooting

- **Logs**: The exporter writes detailed logs to the **Text Commands** palette in Fusion 360 (`View > Show Text Commands` or `Alt+CMD+C`).
- **Orientation**: If your model faces the wrong way, you can edit the `root_adapter` body rotation in the generated XML.

## License

MIT License
