#!/bin/bash

MODELS_DIR="$HOME/trailblazer_ws/src/trailblazer_gazebo/models"

echo "========================================"
echo " Trailblazer Gazebo Model Setup"
echo "========================================"
echo

for MODEL_DIR in "$MODELS_DIR"/*/; do

    MODEL_NAME=$(basename "$MODEL_DIR")
    MESH_DIR="$MODEL_DIR/Mesh"

    echo "Processing: $MODEL_NAME"

    if [ ! -d "$MESH_DIR" ]; then
        echo "  ERROR: Mesh folder missing."
        echo
        continue
    fi

    OBJ_FILE="$MESH_DIR/$MODEL_NAME.obj"
    MTL_FILE="$MESH_DIR/$MODEL_NAME.mtl"
    TEXTURE_FILE="$MESH_DIR/PP_Color_Palette.png"

    # Check required files
    if [ ! -f "$OBJ_FILE" ]; then
        echo "  ERROR: $MODEL_NAME.obj missing."
        echo
        continue
    fi

    if [ ! -f "$MTL_FILE" ]; then
        echo "  ERROR: $MODEL_NAME.mtl missing."
        echo
        continue
    fi

    if [ ! -f "$TEXTURE_FILE" ]; then
        echo "  WARNING: PP_Color_Palette.png missing."
    fi

    # Add texture reference if it doesn't already exist
    if ! grep -q "^map_Kd PP_Color_Palette.png" "$MTL_FILE"; then
        echo "map_Kd PP_Color_Palette.png" >> "$MTL_FILE"
        echo "  Added texture reference."
    else
        echo "  Texture reference already present."
    fi

    # Generate model.config
    cat > "$MODEL_DIR/model.config" <<EOF
<?xml version="1.0"?>

<model>
  <name>$MODEL_NAME</name>
  <version>1.0</version>

  <sdf version="1.7">model.sdf</sdf>

  <author>
    <name>Trailblazer Team</name>
  </author>

  <description>
    $MODEL_NAME environment model for Trailblazer.
  </description>
</model>
EOF

    # Generate model.sdf
    cat > "$MODEL_DIR/model.sdf" <<EOF
<?xml version="1.0"?>

<sdf version="1.7">

  <model name="$MODEL_NAME">

    <static>true</static>

    <link name="${MODEL_NAME}_link">

      <pose>0 0 0 1.5708 0 0</pose>

      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://$MODEL_NAME/Mesh/$MODEL_NAME.obj</uri>
          </mesh>
        </geometry>
      </visual>

      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://$MODEL_NAME/Mesh/$MODEL_NAME.obj</uri>
          </mesh>
        </geometry>
      </collision>

    </link>

  </model>

</sdf>
EOF

    echo "  Created model.config"
    echo "  Created model.sdf"
    echo "  DONE"
    echo

done

echo "========================================"
echo " All Trailblazer models processed."
echo "========================================"