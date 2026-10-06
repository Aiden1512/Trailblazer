#!/bin/bash

MODELS_DIR="$HOME/trailblazer_ws/src/trailblazer_gazebo/models"

echo "Scaling Trailblazer models to 50%..."
echo

for MODEL_DIR in "$MODELS_DIR"/*; do

    [ -d "$MODEL_DIR" ] || continue

    MODEL_NAME=$(basename "$MODEL_DIR")
    SDF_FILE="$MODEL_DIR/model.sdf"

    # Do NOT scale the 20x20 ground
    if [ "$MODEL_NAME" = "Trail_Ground" ]; then
        echo "Skipping Trail_Ground"
        continue
    fi

    if [ ! -f "$SDF_FILE" ]; then
        echo "No model.sdf found for $MODEL_NAME - skipping"
        continue
    fi

    # Don't add the scale again if it already exists
    if grep -q "<scale>0.5 0.5 0.5</scale>" "$SDF_FILE"; then
        echo "$MODEL_NAME already scaled - skipping"
        continue
    fi

    # Add scale directly after every mesh URI.
    # This affects both visual and collision meshes.
    sed -i '/<uri>model:\/\/.*\/Mesh\/.*\.obj<\/uri>/a\            <scale>0.5 0.5 0.5</scale>' "$SDF_FILE"

    echo "Scaled: $MODEL_NAME"

done

echo
echo "Finished."